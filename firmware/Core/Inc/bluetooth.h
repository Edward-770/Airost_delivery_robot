/**
 * @file    bluetooth.h
 * @brief   Library 4: ZS-040 (HC-05 / HC-06) Bluetooth Serial Control & Telemetry
 *                     (Bare-Metal USART2 on PD5 = TX, PD6 = RX, 9600 Baud 8N1)
 *
 * Hardware Connections (STM32 NUCLEO-F756ZG):
 *   - ZS-040 VCC -> 5V Rail (CN8 Pin 9 or 5V Buck Regulator)
 *   - ZS-040 GND -> Common GND (CN8 Pin 11)
 *   - ZS-040 TXD -> PD6 (USART2_RX, CN9 Pin 4 — crossed TX->RX!)
 *   - ZS-040 RXD -> PD5 (USART2_TX, CN9 Pin 6 — crossed RX->TX!)
 *
 * Phone Bluetooth Controller App Commands:
 *   - 'F' / 'W' : Drive Forward   (Encoder Speed-PID Regulated)
 *   - 'B' / 'S' : Drive Backward  (Encoder Speed-PID Regulated)
 *   - 'L' / 'A' : Spin Left       (Encoder Speed-PID Regulated)
 *   - 'R' / 'D' : Spin Right      (Encoder Speed-PID Regulated)
 *   - '0' / 'X' : Stop Motors
 *   - 'G' / 'O' : Grab ('G') or Open ('O') Ball Gripper Claw
 *   - 'U' / 'J' : Lift Up ('U') or Lower Down ('J') Pickup Arm
 *   - 'M' / 'T' : Switch to Manual Mode ('M') or Autonomous Line Mode ('T')
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "stm32f7xx.h"
#include <stdint.h>

/* Live debug variables (add to STM32CubeIDE Live Expressions to verify incoming phone bytes!) */
extern volatile char     bt_last_char;
extern volatile uint32_t bt_rx_count;

/* Initialize USART2 on PD5 (TX) and PD6 (RX) with hardware RXNE interrupt at requested baudrate */
void Bluetooth_Init(uint32_t apb1_clk_hz, uint32_t baudrate);

/* Pop one buffered byte from the USART2 interrupt ring buffer; returns 1 if available, 0 if empty */
int  Bluetooth_Read_Char_NonBlocking(char *ch);

/* Transmit single character, null-terminated string, or compact robot telemetry line */
void Bluetooth_Send_Char(char ch);
void Bluetooth_Send_String(const char *str);
void Bluetooth_Send_Telemetry(int32_t line_err, uint8_t color_id, uint16_t front_mm, int16_t yaw_deg);

#endif /* BLUETOOTH_H */
