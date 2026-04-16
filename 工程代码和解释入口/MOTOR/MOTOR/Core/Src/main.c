/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	#define SHARED_EN_PORT  GPIOB
	#define SHARED_EN_PIN   GPIO_PIN_3

	/* Motor Instance Descriptor */
	typedef struct {
			GPIO_TypeDef* port;
			uint16_t pin_pul;
			uint16_t pin_dir;
			volatile bool is_busy;
	} MotorInst_t;

	/* Hardware Mapping Table */
	static MotorInst_t motors[4] = {
			{GPIOB, GPIO_PIN_4, GPIO_PIN_5, false}, /* MOTOR 1 */
			{GPIOB, GPIO_PIN_6, GPIO_PIN_7, false}, /* MOTOR 2 */
			{GPIOD, GPIO_PIN_0, GPIO_PIN_1, false}, /* MOTOR 3 */
			{GPIOD, GPIO_PIN_2, GPIO_PIN_3, false}  /* MOTOR 4 */
	};

	/* 160MHz Delay Calibration: 1us ˜ 32 loop iterations */
	#define DELAY_US_COEFF  32U

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : HBT42C 4-Axis Closed-Loop Stepper Motor Control
  * @wiring_info    :
  *                   MOTOR1: PU -> PB4,  DR -> PB5
  *                   MOTOR2: PU -> PB6,  DR -> PB7
  *                   MOTOR3: PU -> PD0,  DR -> PD1
  *                   MOTOR4: PU -> PD2,  DR -> PD3
  *                   SHARED ENA -> PB3   (Controls all 4 motors simultaneously)
  *                   All PU-/DR-/EN- pins -> MCU GND (Common-Cathode wiring)
  *                   DIP Switch SW1~SW4: ON/ON/ON/ON (400 Pulse/Rev)
  ******************************************************************************
  */



/**
 * @brief  Microsecond delay (calibrated for 160MHz system clock)
 * @param  nus: Delay duration in microseconds
 */
__STATIC_INLINE void delay_us(uint32_t nus)
{
    __IO uint32_t cycles = nus * DELAY_US_COEFF;
    while(cycles--);
}

/**
 * @brief  Global enable control for all 4 motors (Active-High per manual)
 */
__STATIC_INLINE void motor_global_enable(bool en)
{
    HAL_GPIO_WritePin(SHARED_EN_PORT, SHARED_EN_PIN, 
                      en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Set rotation direction for a specific motor
 */
__STATIC_INLINE void set_motor_dir(MotorInst_t* m, bool cw)
{
    HAL_GPIO_WritePin(m->port, m->pin_dir, 
                      cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Output a single pulse with timing-compliant width
 */
__STATIC_INLINE void pulse_out(MotorInst_t* m, uint32_t half_us)
{
    /* Enforce t3/t4 >= 1.5us requirement from HBT42C manual */
    if(half_us < 2U) half_us = 2U;
    
    HAL_GPIO_WritePin(m->port, m->pin_pul, GPIO_PIN_SET);
    delay_us(half_us);
    HAL_GPIO_WritePin(m->port, m->pin_pul, GPIO_PIN_RESET);
    delay_us(half_us);
}

/**
 * @brief  Run specified motor for exactly 1 revolution (400 pulses)
 * @param  id: Motor index (0~3)
 * @param  cw: Direction (true=CW, false=CCW)
 * @note   Fixed parameters: 400 steps/rev, 60 RPM target, trapezoidal profile
 */
void motor_run(uint8_t id, bool cw)
{
    if(id >= 4U) return;
    MotorInst_t* m = &motors[id];
    m->is_busy = true;

    /* Fixed operational parameters */
    const uint32_t total_steps  = 400U;      /* 1 Revolution */
    const uint32_t start_freq   = 500U;      /* Safe startup frequency */
    const uint32_t target_freq  = 4000U;     /* 60 RPM @ 400 steps/rev */
    const uint32_t accel_steps  = 150U;      /* Accelerate first 150 steps */
    const uint32_t dec_start    = 250U;      /* Decelerate last 150 steps */
    
    /* Step 1: Enable drivers (t1: ENA high >=5us before DIR change) */
    motor_global_enable(true);
    delay_us(10U);

    /* Step 2: Set direction (t2: DIR stable >=1us before PUL) */
    set_motor_dir(m, cw);
    delay_us(5U);

    /* Step 3: Pulse generation with trapezoidal speed profile */
    uint32_t current_freq = start_freq;
    for(uint32_t step = 0; step < total_steps; step++)
    {
        /* Calculate half-period in microseconds */
        uint32_t half_period = 500000U / current_freq;
        pulse_out(m, half_period);

        /* Acceleration phase */
        if(step < accel_steps && current_freq < target_freq)
        {
            current_freq += (target_freq - start_freq) / accel_steps;
            if(current_freq > target_freq) current_freq = target_freq;
        }
        /* Deceleration phase */
        else if(step >= dec_start && current_freq > start_freq)
        {
            current_freq -= (target_freq - start_freq) / (total_steps - dec_start);
            if(current_freq < start_freq) current_freq = start_freq;
        }
    }

    m->is_busy = false;
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
	motor_global_enable(true);
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		motor_run(0, true);  HAL_Delay(500);
    motor_run(1, false); HAL_Delay(500);
    motor_run(2, true);  HAL_Delay(500);
    motor_run(3, false); HAL_Delay(500);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
