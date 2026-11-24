/**
 * @file    line_sensor.h
 * @brief   Library 5: 5-Channel IR Line Sensor Array (Bare-Metal ADC1 + DMA2_Stream0)
 *
 * Hardware Connections (STM32 NUCLEO-F756ZG, power sensor VCC from 3.3V CN8 Pin 7):
 *   - OUT1 (Far Left)  -> PA3 (ADC1_IN3,  CN9 Pin 1,  labeled A0)
 *   - OUT2 (Left)      -> PC0 (ADC1_IN10, CN9 Pin 3,  labeled A1)
 *   - OUT3 (Center)    -> PC3 (ADC1_IN13, CN9 Pin 5,  labeled A2)
 *   - OUT4 (Right)     -> PA4 (ADC1_IN4,  CN7 Pin 17, inner column across from D8)
 *   - OUT5 (Far Right) -> PA5 (ADC1_IN5,  CN7 Pin 10, labeled D13)
 *
 * Polarity Calibration Switch:
 *   - Set LINE_BLACK_IS_HIGH = 1 if black tape produces a HIGHER ADC reading than white floor
 *     (e.g. Pololu QTR-5A analog array).
 *   - Set LINE_BLACK_IS_HIGH = 0 if black tape produces a LOWER  ADC reading than white floor
 *     (e.g. TCRT5000 / BFD-1000 5-channel modules).
 */

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "stm32f7xx.h"
#include <stdint.h>

#define LINE_CHANNELS      5U
#define LINE_BLACK_IS_HIGH 1

/* Continuously updated by DMA2_Stream0 in circular mode (add to STM32CubeIDE Live Expressions!) */
extern volatile uint16_t line_adc_raw[LINE_CHANNELS];

/* Initialize ADC1 5-channel continuous scan + DMA2_Stream0 circular transfer */
void    LineSensor_Init(void);

/* Weighted centroid error from -2000 (line far left) to +2000 (line far right); 0 = centered */
int32_t LineSensor_Get_Error(void);

/* Returns 1 if at least one IR sensor sees the black tape, 0 if inside a dashed-line white gap */
uint8_t LineSensor_Is_Line_Visible(void);

/* Returns 1 if 4 or more sensors see black tape simultaneously (ball-station cross junction) */
uint8_t LineSensor_Is_Cross_Marker(void);

#endif /* LINE_SENSOR_H */
