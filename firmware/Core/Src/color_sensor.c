/**
 * @file    color_sensor.c
 * @brief   Library 3: TCS34725 RGB Colour Sensor Implementation (Bare-Metal I2C1 @ 0x29)
 *
 * Important Initialization Order:
 *   Always call `ToF_Init_All()` BEFORE `ColorSensor_Init()` so the 3x VL53LXX-V2 modules move
 *   from 0x29 to 0x30/0x31/0x32, leaving 0x29 exclusively for the TCS34725 colour sensor!
 */

#include "color_sensor.h"
#include "i2c_bus.h"

#define TCS34725_ADDR7       0x29U /* Fixed factory 7-bit I2C address */
#define TCS34725_COMMAND_BIT 0x80U /* Bit 7 must be 1 for all TCS34725 register commands */
#define TCS34725_REG_ENABLE  0x00U /* Power ON (PON) & RGBC ADC Enable (AEN) */
#define TCS34725_REG_ATIME   0x01U /* Integration Time: 0xF6 = 10 cycles = 24 ms */
#define TCS34725_REG_CONTROL 0x0FU /* Analog Gain: 0x00=1x, 0x01=4x, 0x02=16x, 0x03=60x */
#define TCS34725_REG_CDATAL  0x14U /* Start of 8-byte little-endian burst: C, R, G, B */

/**
 * @brief  Configure TCS34725 integration time (24 ms), 4x gain, and enable continuous RGBC ADC.
 */
int ColorSensor_Init(void)
{
    if (I2C1_Write_Reg8(TCS34725_ADDR7, TCS34725_COMMAND_BIT | TCS34725_REG_ATIME, 0xF6U) != 0) {
        return -1;
    }
    I2C1_Write_Reg8(TCS34725_ADDR7, TCS34725_COMMAND_BIT | TCS34725_REG_CONTROL, 0x01U);

    /* Per TCS34725 datasheet: assert PON (0x01), wait >= 2.4 ms, then enable RGBC ADC (0x03) */
    I2C1_Write_Reg8(TCS34725_ADDR7, TCS34725_COMMAND_BIT | TCS34725_REG_ENABLE, 0x01U);
    Robot_Delay_Ms(5U);
    I2C1_Write_Reg8(TCS34725_ADDR7, TCS34725_COMMAND_BIT | TCS34725_REG_ENABLE, 0x03U);
    Robot_Delay_Ms(30U);
    return 0;
}

/**
 * @brief  Burst-read 16-bit Clear, Red, Green, and Blue channels and classify station colour.
 */
int ColorSensor_Read_RGBC(TCS34725_Data_t *c)
{
    uint8_t buf[8] = {0};
    if (I2C1_Read_Multi8(TCS34725_ADDR7, TCS34725_COMMAND_BIT | TCS34725_REG_CDATAL, buf, 8U) != 0) {
        return -1;
    }

    /* TCS34725 outputs 16-bit channels in little-endian order (Low byte first, High byte second) */
    c->clear    = ((uint16_t)buf[1] << 8U) | buf[0];
    c->red      = ((uint16_t)buf[3] << 8U) | buf[2];
    c->green    = ((uint16_t)buf[5] << 8U) | buf[4];
    c->blue     = ((uint16_t)buf[7] << 8U) | buf[6];
    c->detected = ColorSensor_Classify_Tile(c);
    return 0;
}

/**
 * @brief  Ratio-based RGB classifier immune to ambient brightness changes.
 *         Requires dominant channel to exceed both other channels by >= 25% (ratio > 5/4)
 *         so white track board (where R ~ G ~ B) safely returns COLOR_NONE.
 */
StationColor_t ColorSensor_Classify_Tile(TCS34725_Data_t *c)
{
    /* Reject very dark surfaces or shadows */
    if (c->clear < 120U) {
        return COLOR_NONE;
    }

    if ((uint32_t)c->red * 4U > (uint32_t)c->green * 5U &&
        (uint32_t)c->red * 4U > (uint32_t)c->blue  * 5U) {
        return COLOR_RED;   /* Ball 1 */
    }
    if ((uint32_t)c->blue * 4U > (uint32_t)c->red   * 5U &&
        (uint32_t)c->blue * 4U > (uint32_t)c->green * 5U) {
        return COLOR_BLUE;  /* Ball 2 */
    }
    if ((uint32_t)c->green * 4U > (uint32_t)c->red  * 5U &&
        (uint32_t)c->green * 4U > (uint32_t)c->blue * 5U) {
        return COLOR_GREEN; /* Ball 3 */
    }
    return COLOR_NONE;
}
