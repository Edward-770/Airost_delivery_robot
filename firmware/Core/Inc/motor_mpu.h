/**
 * @file    motor_mpu.h
 * @brief   Library 1: TB6612FNG Dual Motor Driver + Metal Cylinder Quadrature Encoders
 *                     + Cascaded Closed-Loop Wheel Speed PID + Gripper Servos + MPU-6050 IMU
 *
 * Hardware Connections (STM32 NUCLEO-F756ZG — 100% free of onboard Ethernet RMII conflicts!):
 *   - TB6612FNG Motor Driver (Logic VCC = 3.3V, Motor VM = Battery +):
 *       PWMA (Left Motor PWM)   -> PE9  (TIM1_CH1, CN10 Pin 4,  labeled D6, 20 kHz)
 *       PWMB (Right Motor PWM)  -> PE11 (TIM1_CH2, CN10 Pin 6,  labeled D5, 20 kHz)
 *       STBY (Driver Enable)    -> PE10 (GPIOE Out, CN10 Pin 24, 4th hole below D0, driven HIGH)
 *       AIN1 (Left Dir 1)       -> PE12 (GPIOE Out, CN10 Pin 26, 5th hole below D0)
 *       AIN2 (Left Dir 2)       -> PE13 (GPIOE Out, CN10 Pin 10, labeled D3)
 *       BIN1 (Right Dir 1)      -> PE14 (GPIOE Out, CN10 Pin 28, 6th hole below D0)
 *       BIN2 (Right Dir 2)      -> PE15 (GPIOE Out, CN10 Pin 30, 7th hole below D0)
 *   - 25 mm Metal Cylinder Motor Hall Encoders (Power Encoder VCC from 3.3V!):
 *       Left  Encoder C1 / C2   -> PA6  (TIM3_CH1, CN7 Pin 12, D12) & PB5  (TIM3_CH2, CN7 Pin 14, D11)
 *       Right Encoder C1 / C2   -> PD12 (TIM4_CH1, CN10 Pin 21)     & PD13 (TIM4_CH2, CN10 Pin 19)
 *   - Ball Gripper Micro Servos (50 Hz PWM, Power Servo V+ from 5V Buck Regulator):
 *       Lift Servo Signal       -> PB10 (TIM2_CH3, CN10 Pin 32, 8th hole below D0)
 *       Claw Servo Signal       -> PB11 (TIM2_CH4, CN10 Pin 34, 9th hole below D0)
 *   - MPU-6050 6-Axis IMU (Shared I2C1 Bus, Address 0x68):
 *       SCL -> PB8 (D15), SDA -> PB9 (D14)
 */

#ifndef MOTOR_MPU_H
#define MOTOR_MPU_H

#include "stm32f7xx.h"
#include <stdint.h>

/* Generic discrete PID controller state (reused for Left Wheel, Right Wheel, and Steering) */
typedef struct {
    float kp;         /* Proportional gain */
    float ki;         /* Integral gain */
    float kd;         /* Derivative gain */
    float integral;   /* Accumulated integral state */
    float prev_error; /* Previous loop error for derivative calculation */
    float int_limit;  /* Anti-windup clamp for integral term */
    float out_limit;  /* Output saturation limit */
} PID_Controller_t;

/* Complete Closed-Loop Drivetrain Telemetry (add `drive_pid` to STM32CubeIDE Live Expressions!) */
typedef struct {
    PID_Controller_t left_speed_pid;   /* Inner PID regulating Left Wheel encoder ticks / 10 ms */
    PID_Controller_t right_speed_pid;  /* Inner PID regulating Right Wheel encoder ticks / 10 ms */
    PID_Controller_t steer_pid;        /* Outer PID converting IR line error -> differential ticks */

    float   target_left_ticks;         /* Desired Left wheel speed (encoder ticks per 10 ms) */
    float   target_right_ticks;        /* Desired Right wheel speed (encoder ticks per 10 ms) */
    int16_t measured_left_ticks;       /* Actual Left encoder counts read from TIM3 in last 10 ms */
    int16_t measured_right_ticks;      /* Actual Right encoder counts read from TIM4 in last 10 ms */
    int32_t total_left_ticks;          /* Cumulative odometry ticks on Left wheel */
    int32_t total_right_ticks;         /* Cumulative odometry ticks on Right wheel */
    int16_t left_pwm_out;              /* Final PWM duty applied to TB6612FNG PWMA (-1000..+1000) */
    int16_t right_pwm_out;             /* Final PWM duty applied to TB6612FNG PWMB (-1000..+1000) */
} DrivePID_State_t;

/* MPU-6050 Sensor Data & Integrated Yaw Angle */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float   yaw_deg;      /* Integrated Z-axis heading angle in degrees (+CCW / -CW) */
    float   gyro_z_bias;  /* Stationary Z-gyro zero offset calibrated at startup */
} MPU6050_Data_t;

/* Peripheral & PID Initialization */
void    MotorMPU_Init(uint32_t apb1_timer_clk_hz, uint32_t apb2_timer_clk_hz);
void    PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float int_limit, float out_limit);
void    PID_Reset(PID_Controller_t *pid);
float   PID_Compute(PID_Controller_t *pid, float setpoint, float measurement, float dt_sec);

/* Closed-Loop Encoder Speed PID Interface */
void    DrivePID_Init(DrivePID_State_t *drive);
void    DrivePID_Reset(DrivePID_State_t *drive);
void    DrivePID_Set_Target_Ticks(DrivePID_State_t *drive, float left_ticks_per_10ms, float right_ticks_per_10ms);
void    DrivePID_Update_10ms(DrivePID_State_t *drive, float dt_sec);

/* Direct Low-Level Hardware Actuators & Encoders */
void    Motor_Set_PWM(int16_t left_pwm, int16_t right_pwm); /* Duty range: -1000 to +1000 */
void    Motor_Stop(void);
void    Servo_Set_Pulse_Us(uint16_t lift_us, uint16_t claw_us); /* Pulse width: 600..2400 us */
int16_t Encoder_Get_Left_Delta(void);
int16_t Encoder_Get_Right_Delta(void);

/* MPU-6050 IMU & Closed-Loop Gyro + Encoder 90-Degree Turn */
int     MPU6050_Init_And_Calibrate(MPU6050_Data_t *imu);
int     MPU6050_Update_Yaw(MPU6050_Data_t *imu, float dt_sec);
void    MPU6050_Reset_Yaw(MPU6050_Data_t *imu);
void    MotorMPU_Turn_By_Angle_PID(MPU6050_Data_t *imu, DrivePID_State_t *drive, float target_deg, float max_turn_ticks);

#endif /* MOTOR_MPU_H */
