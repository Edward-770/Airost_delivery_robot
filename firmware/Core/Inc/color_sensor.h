/**
 * @file    color_sensor.h
 * @brief   Library 3: TCS34725 RGB Colour Sensor (Bare-Metal I2C1, Address 0x29)
 *
 * Hardware Connections (STM32 NUCLEO-F756ZG, power VIN from 3.3V):
 *   - SCL -> PB8 (CN7 Pin 2, D15)
 *   - SDA -> PB9 (CN7 Pin 4, D14)
 *
 * AIROST Challenge Station Mapping:
 *   - COLOR_RED   (1) -> Ball 1 pickup branch
 *   - COLOR_BLUE  (2) -> Ball 2 pickup branch
 *   - COLOR_GREEN (3) -> Ball 3 pickup branch
 */

#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include "stm32f7xx.h"
#include <stdint.h>

typedef enum {
    COLOR_NONE  = 0,
    COLOR_RED   = 1, /* Ball 1 */
    COLOR_BLUE  = 2, /* Ball 2 */
    COLOR_GREEN = 3  /* Ball 3 */
} StationColor_t;

/* Raw 16-bit Clear/Red/Green/Blue channels + classified station colour */
typedef struct {
    uint16_t       clear;
    uint16_t       red;
    uint16_t       green;
    uint16_t       blue;
    StationColor_t detected;
} TCS34725_Data_t;

/* Power on TCS34725, set 24 ms integration time and 4x gain */
int            ColorSensor_Init(void);

/* Read 8-byte RGBC burst from TCS34725 and classify the tile colour */
int            ColorSensor_Read_RGBC(TCS34725_Data_t *c);

/* Classify raw RGBC values into Red, Blue, Green, or None */
StationColor_t ColorSensor_Classify_Tile(TCS34725_Data_t *c);

#endif /* COLOR_SENSOR_H */
