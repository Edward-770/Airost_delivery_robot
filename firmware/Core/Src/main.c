/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AIROST Dual-Mode Autonomous Delivery Robot — Main Combination Logic
  *                   (STM32 NUCLEO-F756ZG Bare-Metal Modular Architecture)
  *
  * Modular Libraries Integrated Here:
  *   1. motor_mpu.h    : TB6612FNG PWM + Quadrature Encoders + Cascaded PID + Servos + MPU-6050
  *   2. tof.h          : 3x VL53LXX-V2 ToF Distance Sensors (XSHUT PG0-PG2 -> 0x30/0x31/0x32)
  *   3. color_sensor.h : TCS34725 RGB Colour Sensor (0x29: Red=Ball 1, Blue=Ball 2, Green=Ball 3)
  *   4. bluetooth.h    : ZS-040 Bluetooth Manual Control & Telemetry (USART2 PD5/PD6 @ 9600)
  *   5. line_sensor.h  : 5-Channel IR Reflectance Array (ADC1 + DMA2_Stream0 circular scan)
  *   6. i2c_bus.h      : Shared 100 kHz I2C1 Bus (PB8/PB9) + Exact TIM6 1 ms Hardware Delay
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "i2c_bus.h"
#include "motor_mpu.h"
#include "tof.h"
#include "color_sensor.h"
#include "bluetooth.h"
#include "line_sensor.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* SystemClock_Config() runs on 16 MHz HSI (AHB=16 MHz, APB1=16 MHz, APB2=16 MHz) */
#define APB1_CLK_HZ             16000000UL
#define APB1_TIMER_CLK_HZ       16000000UL
#define APB2_TIMER_CLK_HZ       16000000UL

/* Fixed 100 Hz (10.00 ms) PID control loop period */
#define LOOP_DT_SEC             0.010f

/* Target base wheel speeds in quadrature encoder ticks per 10 ms (tune 10..30 for your gearbox) */
#define BASE_SPEED_TICKS_10MS   16.0f
#define MAZE_SPEED_TICKS_10MS   13.0f
#define MANUAL_SPEED_TICKS_10MS 18.0f
#define TURN_MAX_TICKS_10MS     14.0f

/* Micro-servo pulse width presets in microseconds (600..2400 us) */
#define SERVO_LIFT_UP_US        1800U /* Arm raised in carry pose */
#define SERVO_LIFT_DOWN_US      1150U /* Arm lowered for 40 mm ping-pong ball pickup */
#define SERVO_CLAW_OPEN_US      1100U /* Gripper jaws open */
#define SERVO_CLAW_CLOSE_US     1750U /* Gripper jaws closed around ball */
/* USER CODE END PD */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* High-level finite state machine modes */
typedef enum {
    STATE_AUTO_LINE_FOLLOW = 0, /* Autonomous line + MPU-6050 dashed-gap lock + colour detection */
    STATE_AUTO_MAZE_AVOID  = 1, /* Autonomous obstacle / maze navigation via 3x ToF + MPU-6050 */
    STATE_MANUAL_BLUETOOTH = 2  /* Wireless manual control via ZS-040 Bluetooth for ball pickup */
} RobotState_t;
/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* Kept for clean linkage with CubeMX-generated stm32f7xx_it.c (bare-metal drivers own ADC1/DMA2!) */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
/* Global telemetry structs — add these to STM32CubeIDE "Live Expressions" while debugging! */
volatile uint32_t       alive_counter  = 0U;         /* Increments 100x/sec to prove main loop is running */
volatile int32_t        live_line_err  = 0;          /* Live line error (-2000..+2000) for Live Expressions */
volatile RobotState_t   robot_state    = STATE_AUTO_LINE_FOLLOW;
volatile StationColor_t target_station = COLOR_NONE; /* Latched: 1=Red/Ball1, 2=Blue/Ball2, 3=Green/Ball3 */
volatile uint8_t        branch_counter = 0U;         /* Counts perpendicular ball branches passed */
volatile uint8_t        cross_latch    = 0U;         /* Edge-detect latch so each junction counts once */

DrivePID_State_t drive_pid;
MPU6050_Data_t   imu_data;
ToF_Distances_t  tof_data = {8190U, 8190U, 8190U};   /* Init to clear path so boot never false-triggers */
TCS34725_Data_t  color_data;

static uint16_t current_lift_us = SERVO_LIFT_UP_US;
static uint16_t current_claw_us = SERVO_CLAW_OPEN_US;
static int32_t  last_line_err   = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);

/* USER CODE BEGIN PFP */
static void Handle_Bluetooth_Command(char cmd);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
 * @brief  Decode single-byte commands received from the ZS-040 Bluetooth app.
 *         Even in Manual Mode, drive commands run through the Encoder Wheel Speed PID!
 */
static void Handle_Bluetooth_Command(char cmd)
{
    /* Mode switch & universal emergency stop work from ANY state */
    if (cmd == 'M' || cmd == 'm') {
        robot_state = STATE_MANUAL_BLUETOOTH;
        DrivePID_Reset(&drive_pid);
        Bluetooth_Send_String("MODE:MANUAL_BT_PID\r\n");
        return;
    }
    if (cmd == 'T' || cmd == 't') {
        robot_state = STATE_AUTO_LINE_FOLLOW;
        DrivePID_Reset(&drive_pid);
        MPU6050_Reset_Yaw(&imu_data);
        Bluetooth_Send_String("MODE:AUTO_LINE_PID\r\n");
        return;
    }
    if (cmd == '0' || cmd == 'X' || cmd == 'x') {
        robot_state = STATE_MANUAL_BLUETOOTH;
        DrivePID_Reset(&drive_pid);
        Bluetooth_Send_String("STOP:MANUAL_BT\r\n");
        return;
    }

    /* Manual wheel driving & servo ball-gripper actions (case-insensitive) */
    if (robot_state == STATE_MANUAL_BLUETOOTH) {
        switch (cmd) {
            case 'F': case 'f': case 'W': case 'w': /* Drive Forward */
                DrivePID_Set_Target_Ticks(&drive_pid, MANUAL_SPEED_TICKS_10MS, MANUAL_SPEED_TICKS_10MS);
                Bluetooth_Send_String("CMD:F\r\n");
                break;
            case 'B': case 'b': case 'S': case 's': /* Drive Backward */
                DrivePID_Set_Target_Ticks(&drive_pid, -MANUAL_SPEED_TICKS_10MS, -MANUAL_SPEED_TICKS_10MS);
                Bluetooth_Send_String("CMD:B\r\n");
                break;
            case 'L': case 'l': case 'A': case 'a': /* Spin Left */
                DrivePID_Set_Target_Ticks(&drive_pid, -TURN_MAX_TICKS_10MS, TURN_MAX_TICKS_10MS);
                Bluetooth_Send_String("CMD:L\r\n");
                break;
            case 'R': case 'r': case 'D': case 'd': /* Spin Right */
                DrivePID_Set_Target_Ticks(&drive_pid, TURN_MAX_TICKS_10MS, -TURN_MAX_TICKS_10MS);
                Bluetooth_Send_String("CMD:R\r\n");
                break;

            case 'G': case 'g': /* Close gripper claw to grab 40 mm ping-pong ball */
                current_claw_us = SERVO_CLAW_CLOSE_US;
                Servo_Set_Pulse_Us(current_lift_us, current_claw_us);
                break;
            case 'O': case 'o': /* Open gripper claw to release ball in unload zone */
                current_claw_us = SERVO_CLAW_OPEN_US;
                Servo_Set_Pulse_Us(current_lift_us, current_claw_us);
                break;
            case 'U': case 'u': /* Raise pickup arm to carry pose */
                current_lift_us = SERVO_LIFT_UP_US;
                Servo_Set_Pulse_Us(current_lift_us, current_claw_us);
                break;
            case 'J': case 'j': /* Lower pickup arm to ball level */
                current_lift_us = SERVO_LIFT_DOWN_US;
                Servo_Set_Pulse_Us(current_lift_us, current_claw_us);
                break;
            default:
                break;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  Application entry point — initializes all 5 bare-metal libraries & runs 100 Hz loop.
  */
int main(void)
{
    /* 1. Configure Cortex-M7 MPU, basic HAL tick, and 16 MHz HSI System Clock */
    MPU_Config();
    HAL_Init();
    SystemClock_Config();

    /* USER CODE BEGIN 2 */
    /* IMPORTANT Cortex-M7 safety: only disable D-Cache if it was actually enabled!
     * Calling SCB_DisableDCache() when D-Cache is already OFF cleans uninitialized cache tags
     * and causes a HardFault / RAM corruption on STM32F756ZG. */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_DisableDCache();
    }

    /* Enable Cortex-M7 Hardware Floating-Point Unit (FPU) with pipeline sync barriers */
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    /* Configure onboard Green User LED LD1 (PB0 on NUCLEO-F756ZG) as heartbeat indicator */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(3UL << (0U * 2U));
    GPIOB->MODER |=  (1UL << (0U * 2U));

    /* ------------------------------------------------------------------------
     * 2. Bare-Metal Initialization Sequence (Start Line ADC+DMA First!)
     * ------------------------------------------------------------------------ */
    /* Library 5: Start 5-channel IR array continuous circular scan via ADC1 + DMA2_Stream0 FIRST */
    LineSensor_Init();

    ToF_Hold_All_In_Reset(); /* Hold PG0, PG1, PG2 LOW first so 0x29 bus is clean */
    I2C1_Bus_Init();         /* Init shared 100 kHz I2C1 bus on PB8 (SCL) & PB9 (SDA) */

    /* Library 2: Wake 3x VL53LXX-V2 one by one -> reassign to 0x30, 0x31, 0x32 */
    ToF_Init_All();

    /* Library 3: Init TCS34725 RGB Colour Sensor at default address 0x29 */
    ColorSensor_Init();

    /* Library 1: Init TB6612FNG PWM/Dir/STBY, Servos, Encoders, Cascaded PID, and MPU-6050 */
    MotorMPU_Init(APB1_TIMER_CLK_HZ, APB2_TIMER_CLK_HZ);
    DrivePID_Init(&drive_pid);
    Servo_Set_Pulse_Us(current_lift_us, current_claw_us);
    MPU6050_Init_And_Calibrate(&imu_data); /* Keep robot still ~0.3 s for gyro zero calibration */

    /* Library 4: Init ZS-040 Bluetooth serial link on USART2 (PD5=TX, PD6=RX @ 9600 baud) */
    Bluetooth_Init(APB1_CLK_HZ, 9600UL);
    Bluetooth_Send_String("AIROST CASCADED PID READY\r\n");

    uint32_t loop_tick = 0U;
    /* USER CODE END 2 */

    /* ------------------------------------------------------------------------
     * 3. Deterministic 100 Hz (10 ms) Main Combination Control Loop
     * ------------------------------------------------------------------------ */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* A. Always check for incoming wireless commands from ZS-040 Bluetooth */
        char bt_cmd = 0;
        while (Bluetooth_Read_Char_NonBlocking(&bt_cmd)) {
            Handle_Bluetooth_Command(bt_cmd);
        }

        /* B. Integrate MPU-6050 Z-axis gyro yaw angle every 10 ms */
        MPU6050_Update_Yaw(&imu_data, LOOP_DT_SEC);

        /* C. Poll 3x ToF distance sensors and TCS34725 colour sensor every 50 ms (20 Hz) */
        if ((loop_tick % 5U) == 0U) {
            ToF_Read_All(&tof_data);
            ColorSensor_Read_RGBC(&color_data);

            /* Latch target ball station the first time a Red/Blue/Green tile is detected */
            if (target_station == COLOR_NONE && color_data.detected != COLOR_NONE) {
                target_station = color_data.detected;
                MPU6050_Reset_Yaw(&imu_data);
            }
        }

        /* D. Outer-Loop State Machine -> Computes Target Wheel Speeds (ticks/10 ms) */
        if (robot_state == STATE_AUTO_LINE_FOLLOW)
        {
            /* 1. Front obstacle safety check (< 130 mm) -> enter Maze / Obstacle Avoidance */
            if (tof_data.front_mm < 130U) {
                DrivePID_Reset(&drive_pid);
                robot_state = STATE_AUTO_MAZE_AVOID;
            }
            /* 2. Perpendicular ball-branch junction detection (>= 4 black IR channels) */
            else if (LineSensor_Is_Cross_Marker()) {
                if (!cross_latch) {
                    cross_latch = 1U;
                    branch_counter++;

                    /* Stop & turn 90 deg when branch count matches the colour tile read earlier! */
                    if (target_station != COLOR_NONE && branch_counter == (uint8_t)target_station) {
                        DrivePID_Reset(&drive_pid);
                        /* Exact closed-loop 90-deg turn using MPU-6050 Gyro + Encoder Speed PID */
                        MotorMPU_Turn_By_Angle_PID(&imu_data, &drive_pid, -90.0f, TURN_MAX_TICKS_10MS);
                        /* Hand over to ZS-040 Bluetooth manual mode to pick up the ping-pong ball */
                        robot_state = STATE_MANUAL_BLUETOOTH;
                        Bluetooth_Send_String("STATION_REACHED:MANUAL_PICKUP\r\n");
                    }
                }
                /* If we haven't reached our target branch yet, drive straight across the junction */
                if (robot_state == STATE_AUTO_LINE_FOLLOW) {
                    DrivePID_Set_Target_Ticks(&drive_pid, BASE_SPEED_TICKS_10MS, BASE_SPEED_TICKS_10MS);
                }
            }
            /* 3. Cascaded Line-Following PID + MPU-6050 Gyro Lock across 40 cm Dashed Line Gaps */
            else {
                cross_latch = 0U;
                float steer_ticks = 0.0f;

                if (LineSensor_Is_Line_Visible()) {
                    /* Black tape is under the 5-ch array -> run Outer-Loop Line Steering PID */
                    last_line_err = LineSensor_Get_Error();
                    steer_ticks   = PID_Compute(&drive_pid.steer_pid, 0.0f, -(float)last_line_err, LOOP_DT_SEC);
                    MPU6050_Reset_Yaw(&imu_data); /* Re-zero gyro reference while aligned on tape */
                } else {
                    /* Inside a 10 mm white gap of the 40 cm dashed line -> hold 0-deg heading with gyro! */
                    PID_Reset(&drive_pid.steer_pid);
                    steer_ticks = -0.35f * imu_data.yaw_deg;
                    if (steer_ticks >  10.0f) steer_ticks =  10.0f;
                    if (steer_ticks < -10.0f) steer_ticks = -10.0f;
                }

                /* Command differential speed targets to the Inner Left & Right Encoder PIDs */
                DrivePID_Set_Target_Ticks(&drive_pid,
                                          BASE_SPEED_TICKS_10MS + steer_ticks,
                                          BASE_SPEED_TICKS_10MS - steer_ticks);
            }
        }
        else if (robot_state == STATE_AUTO_MAZE_AVOID)
        {
            if (tof_data.front_mm < 150U) {
                DrivePID_Reset(&drive_pid);
                /* Turn 90 degrees toward whichever side (Left vs Right ToF) has more open space */
                if (tof_data.left_mm > tof_data.right_mm) {
                    MotorMPU_Turn_By_Angle_PID(&imu_data, &drive_pid, 90.0f, TURN_MAX_TICKS_10MS);
                } else {
                    MotorMPU_Turn_By_Angle_PID(&imu_data, &drive_pid, -90.0f, TURN_MAX_TICKS_10MS);
                }
            } else {
                /* Center the robot between Left and Right maze walls using side ToF sensors */
                float wall_steer_ticks = 0.0f;
                if (tof_data.left_mm < 250U && tof_data.right_mm < 250U) {
                    wall_steer_ticks = 0.04f * ((float)tof_data.right_mm - (float)tof_data.left_mm);
                    if (wall_steer_ticks >  8.0f) wall_steer_ticks =  8.0f;
                    if (wall_steer_ticks < -8.0f) wall_steer_ticks = -8.0f;
                }
                DrivePID_Set_Target_Ticks(&drive_pid,
                                          MAZE_SPEED_TICKS_10MS + wall_steer_ticks,
                                          MAZE_SPEED_TICKS_10MS - wall_steer_ticks);

                /* Resume autonomous line following as soon as black track tape is re-acquired */
                if (LineSensor_Is_Line_Visible()) {
                    PID_Reset(&drive_pid.steer_pid);
                    robot_state = STATE_AUTO_LINE_FOLLOW;
                }
            }
        }

        /* E. Run Inner-Loop Left & Right Wheel Encoder Speed PID every 10 ms (reads TIM3/TIM4) */
        DrivePID_Update_10ms(&drive_pid, LOOP_DT_SEC);

        /* F. Blink onboard Green LED (LD1 / PB0) & send Bluetooth telemetry every 250 ms (4 Hz) */
        live_line_err = LineSensor_Get_Error();
        alive_counter++;
        if ((loop_tick % 25U) == 0U) {
            GPIOB->ODR ^= (1UL << 0U); /* Visual heartbeat: blinking Green LD1 proves firmware is alive! */
            Bluetooth_Send_Telemetry(last_line_err,
                                     (uint8_t)target_station,
                                     tof_data.front_mm,
                                     (int16_t)imu_data.yaw_deg);
        }

        loop_tick++;
        Robot_Delay_Ms(10U); /* Exact 10.00 ms hardware TIM6 period */
    }
    /* USER CODE END WHILE */
}

/**
  * @brief  System Clock Configuration — 16 MHz HSI (AHB=16 MHz, APB1=16 MHz, APB2=16 MHz)
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  Cortex-M7 MPU Configuration (CubeMX default non-cacheable guard region)
  */
static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x0;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  Error Handler called on fatal clock configuration error.
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
