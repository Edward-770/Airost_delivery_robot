/**
 * @file    motor_mpu.c
 * @brief   Library 1: TB6612FNG Dual Motor Driver + Quadrature Encoders + Cascaded Wheel PID
 *                     + Ball Gripper Servos + MPU-6050 6-Axis IMU
 *
 * Hardware Timers Configured Directly (Bare-Metal Registers):
 *   - TIM1 : 20 kHz Ultrasonic Motor PWM on PE9 (PWMA / D6) & PE11 (PWMB / D5) — silent operation!
 *   - TIM2 : 50 Hz Standard Servo PWM (1 us resolution) on PB10 (Lift) & PB11 (Claw).
 *   - TIM3 : Hardware Quadrature Encoder Interface Mode 3 on PA6 (D12) & PB5 (D11) (Left Wheel).
 *   - TIM4 : Hardware Quadrature Encoder Interface Mode 3 on PD12 & PD13 (Right Wheel).
 */

#include "motor_mpu.h"
#include "i2c_bus.h"

/* MPU-6050 I2C address (AD0 = GND) and internal register map */
#define MPU6050_ADDR7        0x68U
#define MPU6050_REG_SMPLRT   0x19U /* Sample Rate Divider */
#define MPU6050_REG_CONFIG   0x1AU /* Digital Low-Pass Filter (DLPF) */
#define MPU6050_REG_GYRO_CFG 0x1BU /* Gyroscope Full-Scale Range */
#define MPU6050_REG_ACC_CFG  0x1CU /* Accelerometer Full-Scale Range */
#define MPU6050_REG_ACCEL_X  0x3BU /* Start of 14-byte burst: Accel(6) + Temp(2) + Gyro(6) */
#define MPU6050_REG_PWR_MGMT 0x6BU /* Power Management 1 (Sleep & Clock Source) */

/**
 * @brief  Initialize TB6612FNG PWM/Dir/STBY, Servo PWM (TIM2), and Wheel Encoders (TIM3/TIM4).
 */
void MotorMPU_Init(uint32_t apb1_timer_clk_hz, uint32_t apb2_timer_clk_hz)
{
    /* 1. Enable peripheral clocks for GPIOA, GPIOB, GPIOD, GPIOE and TIM1..TIM4 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    (void)RCC->APB1ENR;

    /* 2. Configure PE9 (TIM1_CH1 = PWMA / D6) and PE11 (TIM1_CH2 = PWMB / D5) as AF1 */
    GPIOE->MODER  &= ~((3UL << (9U * 2U)) | (3UL << (11U * 2U)));
    GPIOE->MODER  |=  ((2UL << (9U * 2U)) | (2UL << (11U * 2U)));
    GPIOE->AFR[1] &= ~((0xFUL << ((9U - 8U) * 4U)) | (0xFUL << ((11U - 8U) * 4U)));
    GPIOE->AFR[1] |=  ((1UL   << ((9U - 8U) * 4U)) | (1UL   << ((11U - 8U) * 4U)));

    /* 3. Configure PE10 (STBY) and PE12..PE15 (AIN1, AIN2, BIN1, BIN2) as Push-Pull Outputs */
    GPIOE->MODER &= ~((3UL << (10U * 2U)) | (3UL << (12U * 2U)) | (3UL << (13U * 2U)) |
                      (3UL << (14U * 2U)) | (3UL << (15U * 2U)));
    GPIOE->MODER |=  ((1UL << (10U * 2U)) | (1UL << (12U * 2U)) | (1UL << (13U * 2U)) |
                      (1UL << (14U * 2U)) | (1UL << (15U * 2U)));

    /* Drive PE10 (TB6612FNG STBY pin) HIGH so the dual H-bridge outputs are enabled */
    GPIOE->BSRR = (1UL << 10U);

    /* Configure TIM1 for 20 kHz PWM (Duty cycle 0..1000 via CCR1 & CCR2) */
    uint32_t psc1 = apb2_timer_clk_hz / (20000UL * 1000UL);
    TIM1->PSC   = (psc1 > 0U) ? (psc1 - 1U) : 0U;
    TIM1->ARR   = 1000U - 1U;
    TIM1->CCR1  = 0U;
    TIM1->CCR2  = 0U;
    TIM1->CCMR1 = (6UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | /* CH1 PWM Mode 1 + Preload */
                  (6UL << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;  /* CH2 PWM Mode 1 + Preload */
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;                  /* Enable CH1 & CH2 outputs */
    TIM1->BDTR  = TIM_BDTR_MOE;                                   /* Master Output Enable (required on TIM1!) */
    TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;                     /* Start TIM1 */

    /* 4. Configure PB10 (TIM2_CH3 = Lift Servo) and PB11 (TIM2_CH4 = Claw Servo) as AF1 */
    GPIOB->MODER  &= ~((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    GPIOB->MODER  |=  ((2UL << (10U * 2U)) | (2UL << (11U * 2U)));
    GPIOB->AFR[1] &= ~((0xFUL << ((10U - 8U) * 4U)) | (0xFUL << ((11U - 8U) * 4U)));
    GPIOB->AFR[1] |=  ((1UL   << ((10U - 8U) * 4U)) | (1UL   << ((11U - 8U) * 4U)));

    /* Configure TIM2 for 50 Hz servo pulses (1 MHz tick -> 1 count = 1 us; 20000 us = 20 ms) */
    uint32_t psc2 = apb1_timer_clk_hz / 1000000UL;
    TIM2->PSC   = (psc2 > 0U) ? (psc2 - 1U) : 0U;
    TIM2->ARR   = 20000U - 1U;
    TIM2->CCR3  = 1500U; /* Neutral 1.5 ms pulse on Lift servo */
    TIM2->CCR4  = 1500U; /* Neutral 1.5 ms pulse on Claw servo */
    TIM2->CCMR2 = (6UL << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6UL << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM2->CCER  = TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM2->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* 5. Configure Hardware Quadrature Encoders (AF2 on TIM3 & TIM4):
     *    Left  Encoder -> PA6 (TIM3_CH1, D12) & PB5 (TIM3_CH2, D11) — zero Ethernet conflict!
     *    Right Encoder -> PD12 (TIM4_CH1, CN10 Pin 21) & PD13 (TIM4_CH2, CN10 Pin 19)
     */
    GPIOA->MODER  &= ~(3UL << (6U * 2U));
    GPIOA->MODER  |=  (2UL << (6U * 2U));
    GPIOA->PUPDR  &= ~(3UL << (6U * 2U));
    GPIOA->PUPDR  |=  (1UL << (6U * 2U)); /* Enable pull-up for open-drain Hall sensors */
    GPIOA->AFR[0] &= ~(0xFUL << (6U * 4U));
    GPIOA->AFR[0] |=  (2UL   << (6U * 4U));

    GPIOB->MODER  &= ~(3UL << (5U * 2U));
    GPIOB->MODER  |=  (2UL << (5U * 2U));
    GPIOB->PUPDR  &= ~(3UL << (5U * 2U));
    GPIOB->PUPDR  |=  (1UL << (5U * 2U));
    GPIOB->AFR[0] &= ~(0xFUL << (5U * 4U));
    GPIOB->AFR[0] |=  (2UL   << (5U * 4U));

    GPIOD->MODER  &= ~((3UL << (12U * 2U)) | (3UL << (13U * 2U)));
    GPIOD->MODER  |=  ((2UL << (12U * 2U)) | (2UL << (13U * 2U)));
    GPIOD->PUPDR  &= ~((3UL << (12U * 2U)) | (3UL << (13U * 2U)));
    GPIOD->PUPDR  |=  ((1UL << (12U * 2U)) | (1UL << (13U * 2U)));
    GPIOD->AFR[1] &= ~((0xFUL << ((12U - 8U) * 4U)) | (0xFUL << ((13U - 8U) * 4U)));
    GPIOD->AFR[1] |=  ((2UL   << ((12U - 8U) * 4U)) | (2UL   << ((13U - 8U) * 4U)));

    /* TIM3 & TIM4 Encoder Mode 3 (SMS = 0b011): counts all rising & falling edges on CH1 & CH2 (4x resolution) */
    TIM3->ARR   = 0xFFFFU;
    TIM3->SMCR  = (3UL << TIM_SMCR_SMS_Pos);
    TIM3->CCMR1 = (1UL << TIM_CCMR1_CC1S_Pos) | (1UL << TIM_CCMR1_CC2S_Pos);
    TIM3->CNT   = 0U;
    TIM3->CR1   = TIM_CR1_CEN;

    TIM4->ARR   = 0xFFFFU;
    TIM4->SMCR  = (3UL << TIM_SMCR_SMS_Pos);
    TIM4->CCMR1 = (1UL << TIM_CCMR1_CC1S_Pos) | (1UL << TIM_CCMR1_CC2S_Pos);
    TIM4->CNT   = 0U;
    TIM4->CR1   = TIM_CR1_CEN;
}

/* ============================================================================
 * Generic Discrete PID Controller & Cascaded Wheel Encoder Speed PID
 * ============================================================================ */

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float int_limit, float out_limit)
{
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->int_limit  = int_limit;
    pid->out_limit  = out_limit;
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

/**
 * @brief  Compute one discrete PID step with integral anti-windup and output saturation.
 */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement, float dt_sec)
{
    float error = setpoint - measurement;

    /* Proportional term */
    float p_out = pid->kp * error;

    /* Integral term with anti-windup clamping */
    pid->integral += error * dt_sec;
    if (pid->integral > pid->int_limit) {
        pid->integral = pid->int_limit;
    } else if (pid->integral < -pid->int_limit) {
        pid->integral = -pid->int_limit;
    }
    float i_out = pid->ki * pid->integral;

    /* Derivative term */
    float deriv = (dt_sec > 0.0001f) ? ((error - pid->prev_error) / dt_sec) : 0.0f;
    pid->prev_error = error;
    float d_out = pid->kd * deriv;

    /* Sum P + I + D and clamp output to safe limits */
    float out = p_out + i_out + d_out;
    if (out > pid->out_limit) {
        out = pid->out_limit;
    } else if (out < -pid->out_limit) {
        out = -pid->out_limit;
    }
    return out;
}

/**
 * @brief  Initialize default starting gains for Left/Right Wheel Speed PIDs and Outer Line PID.
 */
void DrivePID_Init(DrivePID_State_t *drive)
{
    /* Inner-Loop Wheel Speed PIDs:
     *   Input  : Encoder ticks measured per 10 ms (~10..35 ticks/10ms on 25 mm metal gearmotors)
     *   Output : TB6612FNG PWM adjustment (-1000..+1000)
     */
    PID_Init(&drive->left_speed_pid,  22.0f, 85.0f, 0.45f, 10.0f, 1000.0f);
    PID_Init(&drive->right_speed_pid, 22.0f, 85.0f, 0.45f, 10.0f, 1000.0f);

    /* Outer-Loop Line Steering PID:
     *   Input  : Line centroid error (-2000..+2000)
     *   Output : Differential speed command in ticks/10ms (-14..+14)
     */
    PID_Init(&drive->steer_pid, 0.0065f, 0.0005f, 0.0012f, 500.0f, 14.0f);

    drive->target_left_ticks    = 0.0f;
    drive->target_right_ticks   = 0.0f;
    drive->measured_left_ticks  = 0;
    drive->measured_right_ticks = 0;
    drive->total_left_ticks     = 0;
    drive->total_right_ticks    = 0;
    drive->left_pwm_out         = 0;
    drive->right_pwm_out        = 0;
}

void DrivePID_Reset(DrivePID_State_t *drive)
{
    PID_Reset(&drive->left_speed_pid);
    PID_Reset(&drive->right_speed_pid);
    PID_Reset(&drive->steer_pid);
    drive->target_left_ticks  = 0.0f;
    drive->target_right_ticks = 0.0f;
    drive->left_pwm_out       = 0;
    drive->right_pwm_out      = 0;
    Motor_Stop();
}

void DrivePID_Set_Target_Ticks(DrivePID_State_t *drive, float left_ticks_per_10ms, float right_ticks_per_10ms)
{
    drive->target_left_ticks  = left_ticks_per_10ms;
    drive->target_right_ticks = right_ticks_per_10ms;
}

/**
 * @brief  Inner-loop 100 Hz (10 ms) Closed-Loop Wheel Speed PID updater.
 *         Reads TIM3/TIM4 encoder counts, adds velocity feedforward + PID, and drives TB6612FNG.
 */
void DrivePID_Update_10ms(DrivePID_State_t *drive, float dt_sec)
{
    /* 1. Sample quadrature encoder ticks since last 10 ms loop and accumulate total odometry */
    drive->measured_left_ticks  = Encoder_Get_Left_Delta();
    drive->measured_right_ticks = Encoder_Get_Right_Delta();
    drive->total_left_ticks    += (int32_t)drive->measured_left_ticks;
    drive->total_right_ticks   += (int32_t)drive->measured_right_ticks;

    /* 2. If both targets are zero, stop cleanly and clear integral terms to prevent creep */
    if (drive->target_left_ticks > -0.1f && drive->target_left_ticks < 0.1f &&
        drive->target_right_ticks > -0.1f && drive->target_right_ticks < 0.1f) {
        PID_Reset(&drive->left_speed_pid);
        PID_Reset(&drive->right_speed_pid);
        drive->left_pwm_out  = 0;
        drive->right_pwm_out = 0;
        Motor_Stop();
        return;
    }

    /* 3. Velocity Feedforward (instant response) + Encoder Feedback PID (eliminates slip/load error) */
    float left_ff  = drive->target_left_ticks  * 18.0f;
    float right_ff = drive->target_right_ticks * 18.0f;

    float left_pid_out  = PID_Compute(&drive->left_speed_pid,
                                      drive->target_left_ticks,
                                      (float)drive->measured_left_ticks,
                                      dt_sec);
    float right_pid_out = PID_Compute(&drive->right_speed_pid,
                                      drive->target_right_ticks,
                                      (float)drive->measured_right_ticks,
                                      dt_sec);

    float l_total = left_ff  + left_pid_out;
    float r_total = right_ff + right_pid_out;

    if (l_total >  1000.0f) l_total =  1000.0f;
    if (l_total < -1000.0f) l_total = -1000.0f;
    if (r_total >  1000.0f) r_total =  1000.0f;
    if (r_total < -1000.0f) r_total = -1000.0f;

    drive->left_pwm_out  = (int16_t)l_total;
    drive->right_pwm_out = (int16_t)r_total;

    /* 4. Apply signed PWM commands to TB6612FNG */
    Motor_Set_PWM(drive->left_pwm_out, drive->right_pwm_out);
}

/* ============================================================================
 * Low-Level TB6612FNG PWM, Servo PWM, Encoder Readers & MPU-6050 IMU
 * ============================================================================ */

/**
 * @brief  Command Left and Right motor direction (GPIOE->BSRR) and 20 kHz PWM duty (TIM1->CCR1/2).
 */
void Motor_Set_PWM(int16_t left_pwm, int16_t right_pwm)
{
    /* Left motor: AIN1 = PE12, AIN2 = PE13, PWMA = TIM1->CCR1 */
    if (left_pwm >= 0) {
        GPIOE->BSRR = (1UL << 12U) | (1UL << (13U + 16U)); /* AIN1=1, AIN2=0 (Forward) */
        TIM1->CCR1  = (left_pwm > 1000) ? 1000U : (uint16_t)left_pwm;
    } else {
        int16_t mag = -left_pwm;
        GPIOE->BSRR = (1UL << 13U) | (1UL << (12U + 16U)); /* AIN1=0, AIN2=1 (Reverse) */
        TIM1->CCR1  = (mag > 1000) ? 1000U : (uint16_t)mag;
    }

    /* Right motor: BIN1 = PE14, BIN2 = PE15, PWMB = TIM1->CCR2 */
    if (right_pwm >= 0) {
        GPIOE->BSRR = (1UL << 14U) | (1UL << (15U + 16U)); /* BIN1=1, BIN2=0 (Forward) */
        TIM1->CCR2  = (right_pwm > 1000) ? 1000U : (uint16_t)right_pwm;
    } else {
        int16_t mag = -right_pwm;
        GPIOE->BSRR = (1UL << 15U) | (1UL << (14U + 16U)); /* BIN1=0, BIN2=1 (Reverse) */
        TIM1->CCR2  = (mag > 1000) ? 1000U : (uint16_t)mag;
    }
}

void Motor_Stop(void)
{
    TIM1->CCR1  = 0U;
    TIM1->CCR2  = 0U;
    /* Clear AIN1, AIN2, BIN1, BIN2 to put TB6612FNG into coast/stop */
    GPIOE->BSRR = (1UL << (12U + 16U)) | (1UL << (13U + 16U)) |
                  (1UL << (14U + 16U)) | (1UL << (15U + 16U));
}

void Servo_Set_Pulse_Us(uint16_t lift_us, uint16_t claw_us)
{
    if (lift_us < 600U)  lift_us = 600U;
    if (lift_us > 2400U) lift_us = 2400U;
    if (claw_us < 600U)  claw_us = 600U;
    if (claw_us > 2400U) claw_us = 2400U;
    TIM2->CCR3 = lift_us; /* PB10 = Lift Servo */
    TIM2->CCR4 = claw_us; /* PB11 = Claw Servo */
}

int16_t Encoder_Get_Left_Delta(void)
{
    int16_t d = (int16_t)TIM3->CNT; /* Signed 16-bit cast automatically handles reverse wraparound! */
    TIM3->CNT = 0U;
    return d;
}

int16_t Encoder_Get_Right_Delta(void)
{
    int16_t d = (int16_t)TIM4->CNT;
    TIM4->CNT = 0U;
    return d;
}

/**
 * @brief  Wake MPU-6050, configure +-500 deg/s gyro scale, and average 100 stationary bias samples.
 *         Keep the robot still for ~0.3 seconds immediately after power-on!
 */
int MPU6050_Init_And_Calibrate(MPU6050_Data_t *imu)
{
    imu->yaw_deg     = 0.0f;
    imu->gyro_z_bias = 0.0f;

    /* Wake MPU-6050 and select X-gyro PLL clock reference (PWR_MGMT_1 = 0x01) */
    if (I2C1_Write_Reg8(MPU6050_ADDR7, MPU6050_REG_PWR_MGMT, 0x01U) != 0) {
        return -1;
    }
    Robot_Delay_Ms(20U);

    /* Configure 125 Hz sample rate, 42 Hz DLPF, +-500 deg/s gyro (65.5 LSB/(deg/s)), +-2g accel */
    I2C1_Write_Reg8(MPU6050_ADDR7, MPU6050_REG_SMPLRT,   0x07U);
    I2C1_Write_Reg8(MPU6050_ADDR7, MPU6050_REG_CONFIG,   0x03U);
    I2C1_Write_Reg8(MPU6050_ADDR7, MPU6050_REG_GYRO_CFG, 0x08U);
    I2C1_Write_Reg8(MPU6050_ADDR7, MPU6050_REG_ACC_CFG,  0x00U);

    /* Measure stationary zero-rate bias over 100 samples to prevent heading drift */
    int32_t sum_gz = 0;
    uint8_t buf[14];
    for (uint32_t i = 0U; i < 100U; i++) {
        if (I2C1_Read_Multi8(MPU6050_ADDR7, MPU6050_REG_ACCEL_X, buf, 14U) == 0) {
            int16_t gz = (int16_t)(((uint16_t)buf[12] << 8U) | buf[13]);
            sum_gz += gz;
        }
        Robot_Delay_Ms(3U);
    }
    imu->gyro_z_bias = (float)sum_gz / 100.0f;
    return 0;
}

/**
 * @brief  Burst-read 14 bytes from MPU-6050 and integrate bias-corrected Z-axis yaw angle.
 */
int MPU6050_Update_Yaw(MPU6050_Data_t *imu, float dt_sec)
{
    uint8_t buf[14];
    if (I2C1_Read_Multi8(MPU6050_ADDR7, MPU6050_REG_ACCEL_X, buf, 14U) != 0) {
        return -1;
    }
    imu->accel_x = (int16_t)(((uint16_t)buf[0]  << 8U) | buf[1]);
    imu->accel_y = (int16_t)(((uint16_t)buf[2]  << 8U) | buf[3]);
    imu->accel_z = (int16_t)(((uint16_t)buf[4]  << 8U) | buf[5]);
    imu->gyro_x  = (int16_t)(((uint16_t)buf[8]  << 8U) | buf[9]);
    imu->gyro_y  = (int16_t)(((uint16_t)buf[10] << 8U) | buf[11]);
    imu->gyro_z  = (int16_t)(((uint16_t)buf[12] << 8U) | buf[13]);

    /* Convert raw LSB to deg/s (+-500 deg/s => 65.5 LSB per deg/s) with deadband noise filter */
    float rate_dps = ((float)imu->gyro_z - imu->gyro_z_bias) / 65.5f;
    if (rate_dps > 0.35f || rate_dps < -0.35f) {
        imu->yaw_deg += rate_dps * dt_sec;
    }
    return 0;
}

void MPU6050_Reset_Yaw(MPU6050_Data_t *imu)
{
    imu->yaw_deg = 0.0f;
}

/**
 * @brief  Execute an exact closed-loop in-place turn (e.g. +90 deg Left or -90 deg Right)
 *         by feeding the MPU-6050 gyro angle error into the Left & Right Wheel Encoder Speed PIDs!
 */
void MotorMPU_Turn_By_Angle_PID(MPU6050_Data_t *imu, DrivePID_State_t *drive, float target_deg, float max_turn_ticks)
{
    MPU6050_Reset_Yaw(imu);
    PID_Reset(&drive->left_speed_pid);
    PID_Reset(&drive->right_speed_pid);

    /* Timeout cap = 600 * 10 ms = 6 seconds max so robot can never spin forever if IMU disconnects */
    for (uint32_t step = 0U; step < 600U; step++) {
        MPU6050_Update_Yaw(imu, 0.010f);
        float angle_err = target_deg - imu->yaw_deg;

        /* Stop once within +-1.8 degrees of target heading */
        if (angle_err > -1.8f && angle_err < 1.8f) {
            break;
        }

        /* Proportional turn speed that slows down smoothly as we approach 90 degrees */
        float turn_ticks = 0.25f * angle_err;
        if (turn_ticks >  max_turn_ticks) turn_ticks =  max_turn_ticks;
        if (turn_ticks < -max_turn_ticks) turn_ticks = -max_turn_ticks;
        if (turn_ticks >  0.0f && turn_ticks <  4.0f) turn_ticks =  4.0f;
        if (turn_ticks <  0.0f && turn_ticks > -4.0f) turn_ticks = -4.0f;

        /* Opposite wheel targets spin the robot cleanly around its center axle */
        DrivePID_Set_Target_Ticks(drive, -turn_ticks, turn_ticks);
        DrivePID_Update_10ms(drive, 0.010f);
        Robot_Delay_Ms(10U);
    }

    DrivePID_Reset(drive);
}
