/**
 * @file    bluetooth.c
 * @brief   Library 4: ZS-040 (HC-05 / HC-06) Bluetooth UART Driver (Bare-Metal USART2 on PD5/PD6)
 *
 * Registers Controlled Directly:
 *   - GPIOD  : PD5 (USART2_TX) & PD6 (USART2_RX) configured as Alternate Function 7 with pull-ups.
 *   - USART2 : 9600 baud 8N1 (BRR = apb1_clk / baudrate), non-blocking RXNE polling + error clear.
 */

#include "bluetooth.h"

/**
 * @brief  Initialize USART2 on PD5 (TX) and PD6 (RX) for wireless Bluetooth communication.
 */
void Bluetooth_Init(uint32_t apb1_clk_hz, uint32_t baudrate)
{
    /* 1. Enable GPIOD (AHB1) and USART2 (APB1) peripheral clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    /* 2. Configure PD5 (TX) and PD6 (RX) as Alternate Function 7 (USART2) */
    GPIOD->MODER  &= ~((3UL << (5U * 2U)) | (3UL << (6U * 2U)));
    GPIOD->MODER  |=  ((2UL << (5U * 2U)) | (2UL << (6U * 2U)));
    GPIOD->PUPDR  &= ~((3UL << (5U * 2U)) | (3UL << (6U * 2U)));
    GPIOD->PUPDR  |=  ((1UL << (5U * 2U)) | (1UL << (6U * 2U))); /* Pull-up keeps idle RX line HIGH */
    GPIOD->AFR[0] &= ~((0xFUL << (5U * 4U)) | (0xFUL << (6U * 4U)));
    GPIOD->AFR[0] |=  ((7UL   << (5U * 4U)) | (7UL   << (6U * 4U)));

    /* 3. Configure USART2: 8 data bits, no parity, 1 stop bit, oversampling by 16 */
    USART2->CR1 = 0U;
    USART2->BRR = (apb1_clk_hz + (baudrate / 2U)) / baudrate; /* Rounded baud divider (1667 @ 16 MHz / 9600) */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; /* Enable TX, RX, and USART2 */
}

/**
 * @brief  Non-blocking single-byte read from USART2 RDR. Automatically clears overrun/noise flags.
 * @return 1 if a new command byte was read into `*ch`, 0 if no byte waiting.
 */
int Bluetooth_Read_Char_NonBlocking(char *ch)
{
    /* Clear Overrun (ORE), Framing (FE), or Noise (NE) errors so receiver never locks up */
    if (USART2->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
    }

    if (USART2->ISR & USART_ISR_RXNE) {
        *ch = (char)(USART2->RDR & 0xFFU);
        return 1;
    }
    return 0;
}

void Bluetooth_Send_Char(char ch)
{
    while (!(USART2->ISR & USART_ISR_TXE)) {
        /* Wait until transmit data register is ready */
    }
    USART2->TDR = (uint8_t)ch;
}

void Bluetooth_Send_String(const char *str)
{
    while (*str != '\0') {
        Bluetooth_Send_Char(*str++);
    }
}

/* Lightweight integer-to-ASCII helper (avoids heavy printf / sprintf Flash overhead) */
static void bt_send_int(int32_t val)
{
    char buf[12];
    int i = 0;
    if (val < 0) {
        Bluetooth_Send_Char('-');
        val = -val;
    }
    if (val == 0) {
        Bluetooth_Send_Char('0');
        return;
    }
    while (val > 0 && i < 10) {
        buf[i++] = (char)('0' + (val % 10));
        val /= 10;
    }
    while (i > 0) {
        Bluetooth_Send_Char(buf[--i]);
    }
}

/**
 * @brief  Transmit one compact live-telemetry line to the phone Bluetooth Serial Terminal.
 */
void Bluetooth_Send_Telemetry(int32_t line_err, uint8_t color_id, uint16_t front_mm, int16_t yaw_deg)
{
    Bluetooth_Send_String("ERR:");
    bt_send_int(line_err);
    Bluetooth_Send_String(" COL:");
    bt_send_int((int32_t)color_id);
    Bluetooth_Send_String(" TOF:");
    bt_send_int((int32_t)front_mm);
    Bluetooth_Send_String(" YAW:");
    bt_send_int((int32_t)yaw_deg);
    Bluetooth_Send_String("\r\n");
}
