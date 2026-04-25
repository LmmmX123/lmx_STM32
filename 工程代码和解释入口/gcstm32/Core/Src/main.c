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
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "chassis_ctrl.h" 
#include "openmv_ctrl.h"
#include "barcode_scanner.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

extern UART_HandleTypeDef huart1;

int fputc(int ch, FILE *f) {
    uint8_t c = ch;
    HAL_UART_Transmit(&huart2, &c, 1, 0xFFFF);
    return ch;
}


// ============================================================
// ?? HARDWARE WIRING GUIDE (Common-Cathode Configuration)
// ============================================================
// 1. Alarm Wires (AL+ / AL-): Leave FLOATING / UNCONNECTED.
// 2. Enable Wires (EN+): Twist all 4 motor EN+ wires together -> Connect to STM32 PB3.
//    Enable Wires (EN-): Twist all 4 motor EN- wires together -> Connect to STM32 GND.
// 3. Pulse Wires (PU+):
//    Motor 1 -> PB4 | Motor 2 -> PB6 | Motor 3 -> PD0 | Motor 4 -> PD2
//    Pulse Wires (PU-): Twist all 4 motor PU- wires together -> Connect to STM32 GND.
// 4. Direction Wires (DR+):
//    Motor 1 -> PB5 | Motor 2 -> PB7 | Motor 3 -> PD1 | Motor 4 -> PD3
//    Direction Wires (DR-): Twist all 4 motor DR- wires together -> Connect to STM32 GND.
// 5. Power: 24VDC to +VDC/GND. WARNING: Verify polarity before powering on!
// 6. DIP Switches: SW1~SW4 MUST be set to ON/ON/ON/ON (400 Pulse/Rev).
// ============================================================


/********************************************************************
 * Function Call Guide (in main.c)
 ********************************************************************
 * 1. chassis_move(float dist_x, float dist_y, float speed)
 *    - Omnidirectional movement (forward, backward, left, right, diagonal)
 *    - dist_x:  left(negative) / right(positive), unit: meter
 *    - dist_y:  backward(negative) / forward(positive), unit: meter
 *    - speed:   movement speed, recommended: 0.3 ~ 1.2
 *
 *    Examples:
 *    chassis_move(0,    0.5f, 0.8f);   // Move forward 0.5m
 *    chassis_move(0,   -0.3f, 0.8f);   // Move backward 0.3m
 *    chassis_move(0.4f, 0,    0.8f);   // Move right 0.4m
 *    chassis_move(-0.4f,0,    0.8f);   // Move left 0.4m
 *    chassis_move(0.3f, 0.5f, 0.8f);   // Move forward-right diagonally
 *
 * 2. chassis_rotate(int8_t dir, uint16_t angle, float speed)
 *    - In-place rotation (only 90/180/360 degrees supported)
 *    - dir:     1 = clockwise,  -1 = counterclockwise, 0 = clockwise
 *    - angle:   only 90 / 180 / 360
 *    - speed:   rotation speed, recommended: 0.3 ~ 1.0
 *
 *    Examples:
 *    chassis_rotate(1,   90, 0.8f);    // Rotate 90� clockwise
 *    chassis_rotate(-1,  90, 0.8f);    // Rotate 90� counterclockwise
 *    chassis_rotate(1,  180, 0.8f);    // Rotate 180� clockwise
 *    chassis_rotate(1,  360, 0.8f);    // Rotate 360� clockwise
 ********************************************************************/

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
	//chassis_init();
	//OpenMV_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	chassis_init();        // ????

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*
		uint8_t result = OpenMV_Run_Align(1, 3000);

		if(result == 1)
		{
			// ??:OpenMV ?? DONE
			HAL_UART_Transmit(&huart1, (uint8_t*)"OpenMV OK\r\n", 10, HAL_MAX_DELAY);
		}
		else
		{
			// ??:??
			HAL_UART_Transmit(&huart1, (uint8_t*)"OpenMV TIMEOUT\r\n", 15, HAL_MAX_DELAY);
		}*/
		// Rotate 90 degrees CLOCKWISE

		//HAL_Delay(5000);
		//chassis_rotate(-1, 90, 0.3f);		
		
		/*HAL_Delay(5000);
		chassis_move(1.0f, 0.0f, 0.3f);
		HAL_Delay(2000);
		chassis_move(-1.0f, 0.0f, 0.3f);
		HAL_Delay(2000);
		chassis_move(0.0f, 1.0f, 0.3f);
		HAL_Delay(2000);
		chassis_move(0.0f, -1.0f, 0.3f);
		HAL_Delay(2000);
		chassis_rotate(1, 90, 0.1f);
		HAL_Delay(2000);
		chassis_rotate(-1, 90, 0.1f);*/
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);
		HAL_Delay(500); // 等待舵机到位

		// 转到15°
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 667);
		HAL_Delay(500); // 等待舵机到位
		
		
		
		/*
		char *barcode = GM65_ScanOnce();
        
        // 3. ?????? g_barcode_result(????)
        if (barcode != NULL)
        {
				if (strlen(g_barcode_result) > 10)
            {
                HAL_UART_Transmit(&huart1, (uint8_t*)"??????10\r\n", 16, 100);
            }
        }
        else
        {
            // ??????,????????,????
            HAL_UART_Transmit(&huart1, (uint8_t*)"??????\r\n", 12, 100);
        }

        HAL_Delay(1000);   // ??1?,??????
    }
}
		*/
		
		
		
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
