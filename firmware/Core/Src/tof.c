/**
 * @file    tof.c
 * @brief   Library 2: 3x VL53LXX-V2 Time-of-Flight Distance Sensors (Bare-Metal I2C1 + XSHUT)
 *
 * Key Features:
 *   - Uses GPIOG XSHUT pins (PG0, PG1, PG2) to bring up the three sensors one by one and move
 *     them off default address 0x29 to 0x30 (Front), 0x31 (Left), and 0x32 (Right).
 *   - Auto-detects both VL53L0X-V2 (Model ID 0xEE @ reg 0xC0) and VL53L1X-V2 (0xEA @ 0x010F).
 *   - Starts continuous back-to-back ranging so distance reads never stall the 100 Hz PID loop.
 *   - Safely skips any unplugged sensor during bench tests (returns 8190 mm = clear path).
 */

#include "tof.h"
#include "i2c_bus.h"

typedef enum {
    TOF_NONE    = 0, /* Sensor unplugged or unresponsive */
    TOF_VL53L0X = 1, /* Standard purple GY-530 VL53L0X-V2 (8-bit registers) */
    TOF_VL53L1X = 2  /* Extended-range VL53L1X-V2 (16-bit registers) */
} ToF_Chip_t;

/* Per-sensor detected chip type: [0]=Front (0x30), [1]=Left (0x31), [2]=Right (0x32) */
static ToF_Chip_t tof_chip[3] = {TOF_NONE, TOF_NONE, TOF_NONE};

static int tof_addr_to_idx(uint8_t dev_addr7)
{
    if (dev_addr7 == TOF_ADDR_FRONT) return 0;
    if (dev_addr7 == TOF_ADDR_LEFT)  return 1;
    if (dev_addr7 == TOF_ADDR_RIGHT) return 2;
    return -1;
}

/**
 * @brief  Configure PG0, PG1, PG2 as outputs and drive them LOW to hold all 3 ToFs in reset.
 *         Must be called BEFORE I2C1_Bus_Init() / ColorSensor_Init()!
 */
void ToF_Hold_All_In_Reset(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    (void)RCC->AHB1ENR;

    /* Set PG0, PG1, PG2 to General-Purpose Output (MODER = 0b01) */
    GPIOG->MODER &= ~((3UL << (0U * 2U)) | (3UL << (1U * 2U)) | (3UL << (2U * 2U)));
    GPIOG->MODER |=  ((1UL << (0U * 2U)) | (1UL << (1U * 2U)) | (1UL << (2U * 2U)));

    /* Drive PG0, PG1, PG2 LOW using atomic BSRR reset bits (bits 16..18) */
    GPIOG->BSRR = (1UL << (0U + 16U)) | (1UL << (1U + 16U)) | (1UL << (2U + 16U));
    Robot_Delay_Ms(10U);
}

/**
 * @brief  Perform ST's VL53L0X stop_variable handshake and enter continuous back-to-back ranging.
 */
static void vl53l0x_start_continuous(uint8_t addr7)
{
    uint8_t stop_var = 0U;

    /* Read internal stop_variable (register 0x91) required by ST VL53L0X state machine */
    I2C1_Write_Reg8(addr7, 0x88U, 0x00U);
    I2C1_Write_Reg8(addr7, 0x80U, 0x01U);
    I2C1_Write_Reg8(addr7, 0xFFU, 0x01U);
    I2C1_Write_Reg8(addr7, 0x00U, 0x00U);
    I2C1_Read_Reg8(addr7,  0x91U, &stop_var);
    I2C1_Write_Reg8(addr7, 0x00U, 0x01U);
    I2C1_Write_Reg8(addr7, 0xFFU, 0x00U);
    I2C1_Write_Reg8(addr7, 0x80U, 0x00U);

    /* Start continuous back-to-back ranging (SYSRANGE_START reg 0x00 = 0x02) */
    I2C1_Write_Reg8(addr7, 0x80U, 0x01U);
    I2C1_Write_Reg8(addr7, 0xFFU, 0x01U);
    I2C1_Write_Reg8(addr7, 0x00U, 0x00U);
    I2C1_Write_Reg8(addr7, 0x91U, stop_var);
    I2C1_Write_Reg8(addr7, 0x00U, 0x01U);
    I2C1_Write_Reg8(addr7, 0xFFU, 0x00U);
    I2C1_Write_Reg8(addr7, 0x80U, 0x00U);
    I2C1_Write_Reg8(addr7, 0x00U, 0x02U);
}

/**
 * @brief  Release one sensor's XSHUT pin, identify its silicon ID, and program its new I2C address.
 */
static void tof_assign_address(uint8_t idx, uint8_t xshut_pin, uint8_t new_addr7)
{
    tof_chip[idx] = TOF_NONE;

    /* Drive XSHUT pin HIGH to wake only this single VL53LXX-V2 board */
    GPIOG->BSRR = (1UL << xshut_pin);
    Robot_Delay_Ms(15U);

    /* 1. Try VL53L0X Model ID check (8-bit register 0xC0 returns 0xEE) */
    uint8_t id8 = 0U;
    if (I2C1_Read_Reg8(TOF_ADDR_DEFAULT, 0xC0U, &id8) == 0 && id8 == 0xEEU) {
        if (I2C1_Write_Reg8(TOF_ADDR_DEFAULT, 0x8AU, new_addr7 & 0x7FU) == 0) {
            I2C1_Write_Reg8(new_addr7, 0x89U, 0x01U); /* Enable 2.8V I/O standard mode */
            vl53l0x_start_continuous(new_addr7);
            tof_chip[idx] = TOF_VL53L0X;
        }
        Robot_Delay_Ms(5U);
        return;
    }

    /* 2. Try VL53L1X Model ID check (16-bit register 0x010F returns 0xEA) */
    uint8_t id16 = 0U;
    if (I2C1_Read_Reg16(TOF_ADDR_DEFAULT, 0x010FU, &id16) == 0 && id16 == 0xEAU) {
        if (I2C1_Write_Reg16(TOF_ADDR_DEFAULT, 0x0001U, new_addr7 & 0x7FU) == 0) {
            I2C1_Write_Reg16(new_addr7, 0x0087U, 0x40U); /* Start continuous ranging */
            tof_chip[idx] = TOF_VL53L1X;
        }
    }
    Robot_Delay_Ms(5U);
}

void ToF_Init_All(void)
{
    ToF_Hold_All_In_Reset();

    /* Wake and re-address each sensor one at a time */
    tof_assign_address(0U, 0U, TOF_ADDR_FRONT); /* PG0 -> 0x30 */
    tof_assign_address(1U, 1U, TOF_ADDR_LEFT);  /* PG1 -> 0x31 */
    tof_assign_address(2U, 2U, TOF_ADDR_RIGHT); /* PG2 -> 0x32 */
}

uint16_t ToF_Read_Single_Mm(uint8_t dev_addr7)
{
    int idx = tof_addr_to_idx(dev_addr7);
    if (idx < 0 || tof_chip[idx] == TOF_NONE) {
        return 8190U; /* Unplugged sensor safely reports no obstacle */
    }

    if (tof_chip[idx] == TOF_VL53L0X) {
        /* Read latest 16-bit range result (mm) from RESULT_RANGE_STATUS + 10 (reg 0x1E) */
        uint8_t buf[2] = {0};
        if (I2C1_Read_Multi8(dev_addr7, 0x1EU, buf, 2U) != 0) {
            return 8190U;
        }
        I2C1_Write_Reg8(dev_addr7, 0x0BU, 0x01U); /* Clear VL53L0X interrupt flag */
        uint16_t mm = ((uint16_t)buf[0] << 8U) | buf[1];
        return (mm > 25U && mm < 2200U) ? mm : 8190U;
    } else {
        /* Read latest 16-bit range result (mm) from VL53L1X register 0x0096 */
        uint16_t mm = 8190U;
        if (I2C1_Read_Reg16_Word(dev_addr7, 0x0096U, &mm) != 0) {
            return 8190U;
        }
        I2C1_Write_Reg16(dev_addr7, 0x0086U, 0x01U); /* Clear VL53L1X interrupt flag */
        return (mm > 25U && mm < 4000U) ? mm : 8190U;
    }
}

void ToF_Read_All(ToF_Distances_t *d)
{
    d->front_mm = ToF_Read_Single_Mm(TOF_ADDR_FRONT);
    d->left_mm  = ToF_Read_Single_Mm(TOF_ADDR_LEFT);
    d->right_mm = ToF_Read_Single_Mm(TOF_ADDR_RIGHT);
}
