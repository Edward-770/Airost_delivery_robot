/**
 * @file    i2c_bus.c
 * @brief   Shared Bare-Metal I2C1 Bus & Exact TIM6 Millisecond Delay Implementation
 *
 * Registers Controlled Directly:
 *   - TIM6  : Basic 16-bit timer on APB1 used for exact 1.000 ms polling delays
 *             (never interferes with Cortex-M7 SysTick / HAL_IncTick!).
 *   - GPIOB : PB8 (SCL) and PB9 (SDA) configured as AF4, Open-Drain, Pull-Up, Very High Speed.
 *   - I2C1  : STM32F7 I2Cv2 peripheral clocked at 16 MHz HSI (TIMINGR = 0x30420F13 => 100 kHz).
 */

#include "i2c_bus.h"

/* Maximum polling iterations before aborting a stalled I2C transaction (~3 ms @ 16 MHz) */
#define I2C_TIMEOUT 15000UL

/**
 * @brief  Exact bare-metal millisecond delay using hardware Basic Timer TIM6.
 *         Assumes APB1 timer clock = 16 MHz (default HSI in SystemClock_Config).
 */
void Robot_Delay_Ms(uint32_t ms)
{
    if (ms == 0U) {
        return;
    }

    /* 1. Enable TIM6 peripheral clock on APB1 bus */
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    (void)RCC->APB1ENR; /* Dummy read to guarantee clock is active before register writes */

    /* 2. Prescale 16 MHz -> 1 MHz (1 us per timer tick), Auto-Reload = 999 (1000 us = 1.000 ms) */
    uint32_t tim_clk = (SystemCoreClock > 0U) ? SystemCoreClock : 16000000UL;
    TIM6->CR1 = 0U;                          /* Stop timer before configuring */
    TIM6->PSC = (tim_clk / 1000000UL) - 1UL; /* 1 us tick resolution */
    TIM6->ARR = 1000U - 1U;                  /* 1000 ticks = 1.000 ms overflow period */
    TIM6->EGR = TIM_EGR_UG;                  /* Force immediate load of PSC/ARR shadow registers */
    TIM6->SR  = 0U;                          /* Clear Update Interrupt Flag (UIF) */
    TIM6->CR1 = TIM_CR1_CEN;                 /* Start TIM6 counter */

    /* 3. Wait for `ms` hardware overflow events (exact timing regardless of -O0 or -O2!) */
    for (uint32_t i = 0U; i < ms; i++) {
        while ((TIM6->SR & TIM_SR_UIF) == 0U) {
            /* Busy-wait for 1.000 ms hardware tick */
        }
        TIM6->SR = 0U; /* Clear UIF for the next millisecond */
    }

    /* 4. Stop TIM6 when delay finishes */
    TIM6->CR1 = 0U;
}

/**
 * @brief  Initialize I2C1 on PB8 (SCL) and PB9 (SDA) at 100 kHz Standard Mode.
 */
void I2C1_Bus_Init(void)
{
    /* 1. Enable GPIOB (AHB1) and I2C1 (APB1) peripheral clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;

    /* 2. Configure PB8 (SCL) and PB9 (SDA) for Alternate Function 4 (I2C1) */
    GPIOB->MODER   &= ~((3UL << (8U * 2U)) | (3UL << (9U * 2U))); /* Clear mode bits */
    GPIOB->MODER   |=  ((2UL << (8U * 2U)) | (2UL << (9U * 2U))); /* 0b10 = Alternate Function */
    GPIOB->OTYPER  |=  (1UL << 8U) | (1UL << 9U);                 /* 1 = Open-Drain (required by I2C!) */
    GPIOB->OSPEEDR |=  (3UL << (8U * 2U)) | (3UL << (9U * 2U));   /* 0b11 = Very High Speed */
    GPIOB->PUPDR   &= ~((3UL << (8U * 2U)) | (3UL << (9U * 2U)));
    GPIOB->PUPDR   |=  ((1UL << (8U * 2U)) | (1UL << (9U * 2U))); /* 0b01 = Weak Internal Pull-Up */
    GPIOB->AFR[1]  &= ~((0xFUL << ((8U - 8U) * 4U)) | (0xFUL << ((9U - 8U) * 4U)));
    GPIOB->AFR[1]  |=  ((4UL   << ((8U - 8U) * 4U)) | (4UL   << ((9U - 8U) * 4U))); /* AF4 = I2C1 */

    /* 3. Pulse I2C1 peripheral reset so the bus starts cleanly after MCU warm reset */
    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    /* 4. Disable I2C1, program ST RM0385 Table 149 100 kHz timing for 16 MHz I2CCLK, then enable */
    I2C1->CR1     &= ~I2C_CR1_PE;
    I2C1->TIMINGR  = 0x30420F13UL; /* PRESC=3, SCLDEL=4, SDADEL=2, SCLH=15, SCLL=19 => 100 kHz */
    I2C1->CR1     |= I2C_CR1_PE;
}

/**
 * @brief  Recover I2C1 peripheral if a loose jumper wire left the BUSY flag latched.
 */
static void i2c1_recover_if_stuck(void)
{
    if (I2C1->ISR & I2C_ISR_BUSY) {
        uint32_t t = 2000U;
        while ((I2C1->ISR & I2C_ISR_BUSY) && (--t > 0U)) { }
        if (I2C1->ISR & I2C_ISR_BUSY) {
            /* Toggle PE bit to reset I2C1 internal finite state machine */
            I2C1->CR1 &= ~I2C_CR1_PE;
            __NOP(); __NOP();
            I2C1->CR1 |= I2C_CR1_PE;
        }
    }
}

/**
 * @brief  Wait for a specific I2C1->ISR status flag with NACK, bus error, and timeout safety.
 * @return 0 on success, -1 on NACK/bus error, -2 on timeout.
 */
static int i2c1_wait_flag(uint32_t flag)
{
    uint32_t t = I2C_TIMEOUT;
    while (!(I2C1->ISR & flag)) {
        if (I2C1->ISR & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO)) {
            /* Clear error flags so subsequent transactions on other sensors still succeed */
            I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_STOPCF;
            return -1;
        }
        if (--t == 0U) {
            /* Reset I2C1 peripheral on timeout so the main 100 Hz PID loop never freezes */
            I2C1->CR1 &= ~I2C_CR1_PE;
            __NOP(); __NOP();
            I2C1->CR1 |= I2C_CR1_PE;
            return -2;
        }
    }
    return 0;
}

/**
 * @brief  Write 1 byte to an 8-bit register address on a 7-bit I2C device.
 */
int I2C1_Write_Reg8(uint8_t dev_addr7, uint8_t reg, uint8_t data)
{
    i2c1_recover_if_stuck();
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

    /* Configure 2-byte write (reg + data) with automatic STOP after last byte */
    I2C1->CR2 = ((uint32_t)(dev_addr7 << 1U) & I2C_CR2_SADD) |
                (2UL << I2C_CR2_NBYTES_Pos) |
                I2C_CR2_AUTOEND |
                I2C_CR2_START;

    if (i2c1_wait_flag(I2C_ISR_TXIS) != 0) return -1;
    I2C1->TXDR = reg;

    if (i2c1_wait_flag(I2C_ISR_TXIS) != 0) return -1;
    I2C1->TXDR = data;

    if (i2c1_wait_flag(I2C_ISR_STOPF) != 0) return -1;
    I2C1->ICR = I2C_ICR_STOPCF;
    return 0;
}

/**
 * @brief  Read `len` consecutive bytes starting at an 8-bit register using Repeated START.
 */
int I2C1_Read_Multi8(uint8_t dev_addr7, uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c1_recover_if_stuck();
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

    /* Phase 1: Write starting register address without AUTOEND (holds bus for Repeated START) */
    I2C1->CR2 = ((uint32_t)(dev_addr7 << 1U) & I2C_CR2_SADD) |
                (1UL << I2C_CR2_NBYTES_Pos) |
                I2C_CR2_START;

    if (i2c1_wait_flag(I2C_ISR_TXIS) != 0) return -1;
    I2C1->TXDR = reg;
    if (i2c1_wait_flag(I2C_ISR_TC) != 0) return -1;

    /* Phase 2: Send Repeated START in Read mode (RD_WRN=1) for `len` bytes + AUTOEND STOP */
    I2C1->CR2 = ((uint32_t)(dev_addr7 << 1U) & I2C_CR2_SADD) |
                I2C_CR2_RD_WRN |
                ((uint32_t)len << I2C_CR2_NBYTES_Pos) |
                I2C_CR2_AUTOEND |
                I2C_CR2_START;

    for (uint8_t i = 0U; i < len; i++) {
        if (i2c1_wait_flag(I2C_ISR_RXNE) != 0) return -1;
        buf[i] = (uint8_t)I2C1->RXDR;
    }

    if (i2c1_wait_flag(I2C_ISR_STOPF) != 0) return -1;
    I2C1->ICR = I2C_ICR_STOPCF;
    return 0;
}

/**
 * @brief  Read 1 byte from an 8-bit register address.
 */
int I2C1_Read_Reg8(uint8_t dev_addr7, uint8_t reg, uint8_t *data)
{
    return I2C1_Read_Multi8(dev_addr7, reg, data, 1U);
}

/**
 * @brief  Write 1 byte to a 16-bit big-endian register address (used by VL53L1X).
 */
int I2C1_Write_Reg16(uint8_t dev_addr7, uint16_t reg16, uint8_t data)
{
    i2c1_recover_if_stuck();
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

    I2C1->CR2 = ((uint32_t)(dev_addr7 << 1U) & I2C_CR2_SADD) |
                (3UL << I2C_CR2_NBYTES_Pos) |
                I2C_CR2_AUTOEND |
                I2C_CR2_START;

    if (i2c1_wait_flag(I2C_ISR_TXIS) != 0) return -1;
    I2C1->TXDR = (uint8_t)(reg16 >> 8U);

    if (i2c1_wait_flag(I2C_ISR_TXIS) != 0) return -1;
    I2C1->TXDR = (uint8_t)(reg16 & 0xFFU);

    if (i2c1_wait_flag(I2C_ISR_TXIS) != 0) return -1;
    I2C1->TXDR = data;

    if (i2c1_wait_flag(I2C_ISR_STOPF) != 0) return -1;
    I2C1->ICR = I2C_ICR_STOPCF;
    return 0;
}

/**
 * @brief  Read 1 byte from a 16-bit big-endian register address (used by VL53L1X).
 */
int I2C1_Read_Reg16(uint8_t dev_addr7, uint16_t reg16, uint8_t *data)
{
    i2c1_recover_if_stuck();
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

    /* Phase 1: Transmit 16-bit register address MSB then LSB */
    I2C1->CR2 = ((uint32_t)(dev_addr7 << 1U) & I2C_CR2_SADD) |
                (2UL << I2C_CR2_NBYTES_Pos) |
                I2C_CR2_START;

    if (i2c1_wait_flag(I2C_ISR_TXIS) != 0) return -1;
    I2C1->TXDR = (uint8_t)(reg16 >> 8U);
    if (i2c1_wait_flag(I2C_ISR_TXIS) != 0) return -1;
    I2C1->TXDR = (uint8_t)(reg16 & 0xFFU);
    if (i2c1_wait_flag(I2C_ISR_TC) != 0) return -1;

    /* Phase 2: Repeated START read 1 byte */
    I2C1->CR2 = ((uint32_t)(dev_addr7 << 1U) & I2C_CR2_SADD) |
                I2C_CR2_RD_WRN |
                (1UL << I2C_CR2_NBYTES_Pos) |
                I2C_CR2_AUTOEND |
                I2C_CR2_START;

    if (i2c1_wait_flag(I2C_ISR_RXNE) != 0) return -1;
    *data = (uint8_t)I2C1->RXDR;

    if (i2c1_wait_flag(I2C_ISR_STOPF) != 0) return -1;
    I2C1->ICR = I2C_ICR_STOPCF;
    return 0;
}

/**
 * @brief  Read a 16-bit big-endian word from a 16-bit register address (VL53L1X distance mm).
 */
int I2C1_Read_Reg16_Word(uint8_t dev_addr7, uint16_t reg16, uint16_t *word)
{
    uint8_t hi = 0U, lo = 0U;
    if (I2C1_Read_Reg16(dev_addr7, reg16, &hi) != 0) return -1;
    if (I2C1_Read_Reg16(dev_addr7, reg16 + 1U, &lo) != 0) return -1;
    *word = ((uint16_t)hi << 8U) | lo;
    return 0;
}
