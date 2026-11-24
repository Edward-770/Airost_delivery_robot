/**
 * @file    tof.h
 * @brief   Library 2: 3x VL53LXX-V2 (VL53L0X / VL53L1X) Laser Time-of-Flight Sensors
 *
 * Hardware Connections (STM32 NUCLEO-F756ZG, power VIN from 3.3V):
 *   - Shared I2C1 Bus : PB8 (SCL / D15), PB9 (SDA / D14)
 *   - XSHUT Hardware Shutdown Pins (used to resolve default 0x29 address collision):
 *       Front ToF XSHUT -> PG0 (CN9 Pin 29) -> Reassigned to I2C Address 0x30
 *       Left  ToF XSHUT -> PG1 (CN9 Pin 30) -> Reassigned to I2C Address 0x31
 *       Right ToF XSHUT -> PG2 (CN8 Pin 14) -> Reassigned to I2C Address 0x32
 *
 * Note: Moving all three ToF sensors off 0x29 leaves 0x29 dedicated to the TCS34725 colour sensor!
 */

#ifndef TOF_H
#define TOF_H

#include "stm32f7xx.h"
#include <stdint.h>

#define TOF_ADDR_DEFAULT 0x29U
#define TOF_ADDR_FRONT   0x30U
#define TOF_ADDR_LEFT    0x31U
#define TOF_ADDR_RIGHT   0x32U

/* Latest distance measurements in millimetres (8190 = out of range / clear path / sensor unplugged) */
typedef struct {
    uint16_t front_mm;
    uint16_t left_mm;
    uint16_t right_mm;
} ToF_Distances_t;

/* Drive PG0, PG1, PG2 LOW before initializing I2C so no ToF collides on 0x29 */
void     ToF_Hold_All_In_Reset(void);

/* Sequentially wake each ToF, auto-detect VL53L0X vs VL53L1X, reassign address, and start ranging */
void     ToF_Init_All(void);

/* Read single sensor distance in millimetres (non-blocking) */
uint16_t ToF_Read_Single_Mm(uint8_t dev_addr7);

/* Read Front, Left, and Right distances into struct */
void     ToF_Read_All(ToF_Distances_t *d);

#endif /* TOF_H */
