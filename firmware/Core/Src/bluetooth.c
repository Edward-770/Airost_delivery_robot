/**
 * @file    bluetooth.c
 * @brief   Library 4: ZS-040 (HC-05 / HC-06) Bluetooth UART Driver (Bare-Metal USART2 on PD5/PD6)
 *
 * Registers Controlled Directly:
 *   - GPIOD  : PD5 (USART2_TX) & PD6 (USART2_RX) configured as Alternate Function 7 with pull-ups.
 *   - USART2 : 9600 baud 8N1 (BRR = apb1_clk / baudrate), non-blocking RXNE polling + error clear.
 */

#include "bluetooth.h"

#define BT_RX_BUF_SIZE 32U

static volatile char     bt_rx_buf[BT_RX_BUF_SIZE];
static volatile uint8_t  bt_rx_head = 0U;
static volatile uint8_t  bt_rx_tail = 0U;

volatile char     bt_last_char = 0;
volatile uint32_t bt_rx_count  = 0U;

/**
 * @brief  Initialize USART2 on PD5 (TX) and PD6 (RX) with RXNE interrupt for wireless Bluetooth.
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

    /* 3. Configure USART2: 8N1, oversampling by 16, RXNE interrupt enabled */
    USART2->CR1 = 0U;
    USART2->BRR = (apb1_clk_hz + (baudrate / 2U)) / baudrate; /* Rounded baud divider (1667 @ 16 MHz / 9600) */
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    /* 4. Enable USART2 interrupt in Cortex-M7 NVIC so incoming phone commands are never missed */
    NVIC_SetPriority(USART2_IRQn, 2U);
    NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * @brief  Hardware USART2 Interrupt Handler — captures every incoming byte into ring buffer.
 */
void USART2_IRQHandler(void)
{
    if (USART2->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
    }

    while (USART2->ISR & USART_ISR_RXNE) {
        char c = (char)(USART2->RDR & 0xFFU);
        bt_last_char = c;
        bt_rx_count++;

        uint8_t next = (uint8_t)((bt_rx_head + 1U) & (BT_RX_BUF_SIZE - 1U));
        if (next != bt_rx_tail) {
            bt_rx_buf[bt_rx_head] = c;
            bt_rx_head = next;
        }
    }
}

/**
 * @brief  Pop one received byte from the USART2 interrupt ring buffer (non-blocking).
 * @return 1 if a new command byte was read into `*ch`, 0 if ring buffer is empty.
 */
int Bluetooth_Read_Char_NonBlocking(char *ch)
{
    if (bt_rx_tail == bt_rx_head) {
        return 0;
    }
    *ch = bt_rx_buf[bt_rx_tail];
    bt_rx_tail = (uint8_t)((bt_rx_tail + 1U) & (BT_RX_BUF_SIZE - 1U));
    return 1;
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
