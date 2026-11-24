/**
 * @file    line_sensor.c
 * @brief   Library 5: 5-Channel IR Line Sensor Array (Bare-Metal ADC1 + DMA2_Stream0)
 *
 * How It Works:
 *   - ADC1 continuously scans 5 analog input pins in order:
 *       SQ1 = PA3 (IN3), SQ2 = PC0 (IN10), SQ3 = PC3 (IN13), SQ4 = PA4 (IN4), SQ5 = PA5 (IN5).
 *   - After every single conversion, DMA2_Stream0 (Channel 0) automatically copies the 12-bit
 *     result from ADC1->DR into `line_adc_raw[0..4]` in RAM using Circular Mode — zero CPU load!
 *   - Note: `SCB_DisableDCache()` is called in main() so Cortex-M7 always reads fresh DMA data.
 */

#include "line_sensor.h"

/* Global DMA destination buffer (0..4095 for each of the 5 IR reflectance channels) */
volatile uint16_t line_adc_raw[LINE_CHANNELS] = {0};

/* Thresholds for distinguishing 19 mm black electrical tape from white track board */
#define LINE_BLACK_THRESHOLD 1400
#define LINE_NOISE_FLOOR     400

/**
 * @brief  Convert raw 12-bit ADC value into normalized "black-line strength" (0..3695),
 *         applying the LINE_BLACK_IS_HIGH polarity switch and subtracting white-floor noise.
 */
static int32_t line_channel_strength(uint32_t idx)
{
    int32_t raw = (int32_t)(line_adc_raw[idx] & 0x0FFFU);
#if LINE_BLACK_IS_HIGH
    int32_t strength = raw;           /* Higher voltage = darker surface (e.g. QTR-5A) */
#else
    int32_t strength = 4095 - raw;    /* Lower voltage  = darker surface (e.g. TCRT5000) */
#endif
    return (strength > LINE_NOISE_FLOOR) ? (strength - LINE_NOISE_FLOOR) : 0;
}

/**
 * @brief  Configure GPIOA/GPIOC analog pins, DMA2_Stream0 circular mode, and ADC1 5-ch scan.
 */
void LineSensor_Init(void)
{
    /* 1. Enable peripheral clocks for GPIOA, GPIOC, DMA2 (AHB1) and ADC1 (APB2) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    /* 2. Set PA3, PA4, PA5 and PC0, PC3 to Analog Mode (MODER = 0b11, no pull-up/pull-down) */
    GPIOA->MODER |=  (3UL << (3U * 2U)) | (3UL << (4U * 2U)) | (3UL << (5U * 2U));
    GPIOA->PUPDR &= ~((3UL << (3U * 2U)) | (3UL << (4U * 2U)) | (3UL << (5U * 2U)));
    GPIOC->MODER |=  (3UL << (0U * 2U)) | (3UL << (3U * 2U));
    GPIOC->PUPDR &= ~((3UL << (0U * 2U)) | (3UL << (3U * 2U)));

    /* 3. Disable DMA2 Stream 0 before modifying its configuration registers */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {
        /* Wait until hardware confirms stream is fully disabled */
    }

    /* Clear all pending interrupt/error status flags for Stream 0 */
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 |
                  DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

    /* Set peripheral source (ADC1->DR), RAM destination (line_adc_raw[]), and count (5) */
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)line_adc_raw;
    DMA2_Stream0->NDTR = LINE_CHANNELS;
    DMA2_Stream0->FCR  = 0U; /* Direct mode (FIFO disabled) */

    /* Configure Stream 0: Channel 0 (ADC1), High Priority, 16-bit MSIZE/PSIZE, Memory Inc, Circular */
    DMA2_Stream0->CR = (0UL << DMA_SxCR_CHSEL_Pos) |
                       DMA_SxCR_PL_1 |
                       DMA_SxCR_MSIZE_0 |
                       DMA_SxCR_PSIZE_0 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_CIRC;
    DMA2_Stream0->CR |= DMA_SxCR_EN; /* Arm DMA stream BEFORE starting ADC conversions */

    /* 4. Configure ADC1 for 5-channel continuous scan with endless DMA requests (DDS=1) */
    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC->CCR  = ADC_CCR_ADCPRE_0; /* ADC clock = PCLK2 / 4 (4 MHz @ 16 MHz HSI) */

    /* Set 84-cycle sample time (0b100) for clean readings on IR phototransistor impedance */
    ADC1->SMPR2 = (4UL << (3U * 3U)) | (4UL << (4U * 3U)) | (4UL << (5U * 3U)); /* IN3, IN4, IN5 */
    ADC1->SMPR1 = (4UL << (0U * 3U)) | (4UL << (3U * 3U));                     /* IN10, IN13 */

    /* Sequence length = 5 conversions (L field = N - 1 = 4) */
    ADC1->SQR1 = ((LINE_CHANNELS - 1UL) << ADC_SQR1_L_Pos);
    /* Conversion order left-to-right: SQ1=IN3, SQ2=IN10, SQ3=IN13, SQ4=IN4, SQ5=IN5 */
    ADC1->SQR3 = (3UL  << (0U * 5U)) |
                 (10UL << (1U * 5U)) |
                 (13UL << (2U * 5U)) |
                 (4UL  << (3U * 5U)) |
                 (5UL  << (4U * 5U));

    ADC1->CR1 = ADC_CR1_SCAN;                                          /* Enable multi-channel scan */
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT | ADC_CR2_ADON; /* DMA + Continuous + Power ON */

    /* Short stabilization delay before triggering the first regular conversion */
    for (volatile uint32_t i = 0U; i < 1000U; i++) {
        __NOP();
    }
    ADC1->CR2 |= ADC_CR2_SWSTART; /* Kick off continuous background ADC+DMA sampling! */
}

/**
 * @brief  Compute weighted centroid lateral error across all 5 IR channels.
 * @return Negative (-2000..-1) if line is to the Left, 0 if Centered, Positive (+1..+2000) if Right.
 */
int32_t LineSensor_Get_Error(void)
{
    static const int32_t weights[LINE_CHANNELS] = {-2000, -1000, 0, 1000, 2000};
    int32_t weighted_sum = 0;
    int32_t total_signal = 0;

    for (uint32_t i = 0U; i < LINE_CHANNELS; i++) {
        int32_t val = line_channel_strength(i);
        weighted_sum += val * weights[i];
        total_signal += val;
    }

    /* Protect against divide-by-zero when all 5 sensors are over white floor */
    if (total_signal < 200) {
        return 0;
    }
    return weighted_sum / total_signal;
}

/**
 * @brief  Check if the black track tape is currently under at least one IR sensor.
 *         Returns 0 when crossing the 10 mm white gaps of the 40 cm dashed line.
 */
uint8_t LineSensor_Is_Line_Visible(void)
{
    for (uint32_t i = 0U; i < LINE_CHANNELS; i++) {
        if (line_channel_strength(i) > (LINE_BLACK_THRESHOLD - LINE_NOISE_FLOOR)) {
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief  Detect a perpendicular ball-station branch junction or wide cross bar (>= 4 sensors black).
 */
uint8_t LineSensor_Is_Cross_Marker(void)
{
    uint8_t active_count = 0U;
    for (uint32_t i = 0U; i < LINE_CHANNELS; i++) {
        if (line_channel_strength(i) > (LINE_BLACK_THRESHOLD - LINE_NOISE_FLOOR)) {
            active_count++;
        }
    }
    return (active_count >= 4U) ? 1U : 0U;
}
