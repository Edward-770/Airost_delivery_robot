/**
 * @file    i2c_bus.h
 * @brief   Shared Bare-Metal I2C1 Bus & Exact TIM6 Millisecond Delay Header
 *
 * Hardware Connections (STM32 NUCLEO-F756ZG):
 *   - PB8 (CN7 Pin 2, labeled D15 / SCL) -> I2C1_SCL (Alternate Function 4, Open-Drain)
 *   - PB9 (CN7 Pin 4, labeled D14 / SDA) -> I2C1_SDA (Alternate Function 4, Open-Drain)
 *
 * I2C Devices Sharing This Bus:
 *   - 0x29 : TCS34725 RGB Colour Sensor (kept at default address)
 *   - 0x30 : Front VL53LXX-V2 ToF Sensor (reassigned from 0x29 via XSHUT PG0)
 *   - 0x31 : Left  VL53LXX-V2 ToF Sensor (reassigned from 0x29 via XSHUT PG1)
 *   - 0x32 : Right VL53LXX-V2 ToF Sensor (reassigned from 0x29 via XSHUT PG2)
 *   - 0x68 : MPU-6050 6-Axis Gyroscope / Accelerometer IMU
 */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "stm32f7xx.h"
#include <stdint.h>

/* Initialize I2C1 on PB8 (SCL) and PB9 (SDA) at 100 kHz standard mode */
void I2C1_Bus_Init(void);

/* 8-bit register address read/write (used by MPU-6050, TCS34725, and VL53L0X) */
int  I2C1_Write_Reg8(uint8_t dev_addr7, uint8_t reg, uint8_t data);
int  I2C1_Read_Reg8(uint8_t dev_addr7, uint8_t reg, uint8_t *data);
int  I2C1_Read_Multi8(uint8_t dev_addr7, uint8_t reg, uint8_t *buf, uint8_t len);

/* 16-bit register address read/write (used by VL53L1X) */
int  I2C1_Write_Reg16(uint8_t dev_addr7, uint16_t reg16, uint8_t data);
int  I2C1_Read_Reg16(uint8_t dev_addr7, uint16_t reg16, uint8_t *data);
int  I2C1_Read_Reg16_Word(uint8_t dev_addr7, uint16_t reg16, uint16_t *word);

/* Exact 1.000 ms bare-metal hardware delay using Basic Timer TIM6 (safe alongside HAL SysTick) */
void Robot_Delay_Ms(uint32_t ms);

#endif /* I2C_BUS_H */
