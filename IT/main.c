/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "uart.h"
#include "string.h"
#include "stdio.h"
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
I2C_Handle hi2c1;
UART_Handle huart1;
UART_Handle huart2;
TaskHandle_t IMU_TaskHandle;
TaskHandle_t GPSR_TaskHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void IMU_Task(void const * argument);
void GPSR_Task(void const * argument);

/* USER CODE BEGIN PFP */
static void JS_I2C1_Init(void);
static void JS_UART1_Init(void);
static void JS_UART2_Init(void);
void debugPrint(char *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE BEGIN 2 */
  JS_I2C1_Init();
  JS_UART1_Init();
  JS_UART2_Init();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutex
   * es, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  xTaskCreate((TaskFunction_t) IMU_Task,
  		(const portCHAR *)"IMU_Task", 512, NULL, tskIDLE_PRIORITY + 3,
  		&IMU_TaskHandle);

  xTaskCreate((TaskFunction_t) GPSR_Task,
          (const portCHAR *)"GPSR_Task", 128, NULL, tskIDLE_PRIORITY + 3,
          &GPSR_TaskHandle);

  vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void JS_I2C1_Init(void) {
	hi2c1.instance = I2C1;
	hi2c1.clockSpeed = 400000;

	if (I2C_Init(&hi2c1, 1) != I2C_OK) {
		Error_Handler();
	}
}

static void JS_UART1_Init(void) {
   huart1.instance = USART1;
   huart1.baudRate = 9600;
   huart1.lock = UART_UNLOCKED;
   huart1.enableIT = 1;

   if (UART_Init(&huart1) != UART_OK) {
      Error_Handler();
   }
}

static void JS_UART2_Init(void) {
	huart2.instance = USART2;
	huart2.baudRate = 9600;
	huart2.lock = UART_UNLOCKED;
	huart2.enableIT = 1;

	if (UART_Init(&huart2) != UART_OK) {
		Error_Handler();
	}
}

void debugPrint(char *_out) {
	UART_Write_IT(&huart2, (uint8_t *) _out, strlen(_out));
}
/* USER CODE END 4 */

/* USER CODE BEGIN IMU_Task */
/**
  * @brief  Function implementing the IMU_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END IMU_Task */
void IMU_Task(void const *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t data[6] = {0};
	int16_t acc[3] = {0};
	const TickType_t xInterruptFrequency = pdMS_TO_TICKS(500UL);
	const TickType_t delay = pdMS_TO_TICKS(1000UL);
	char str[100];

	debugPrint("Start IMU tasks...\r\n");

	I2C_Read_IT(&hi2c1, 0x68, 0x75, data, 1);
	CHECK_IT(xInterruptFrequency);
	if (data[0] == 113) {
		data[0] = 0;
		I2C_Write_IT(&hi2c1, 0x68, 0x6b, data, 1);
		CHECK_IT(xInterruptFrequency);
		data[0] = 0x07;
		I2C_Write_IT(&hi2c1, 0x68, 0x19, data, 1);
		CHECK_IT(xInterruptFrequency);
		data[0] = 0;
		I2C_Write_IT(&hi2c1, 0x68, 0x1c, data, 1);
		CHECK_IT(xInterruptFrequency);
		data[0] = 0;
		I2C_Write_IT(&hi2c1, 0x68, 0x1b, data, 1);
		CHECK_IT(xInterruptFrequency);
	} else {
		sprintf(str,"Wrong who_am_i number: %d\r\n",data[0]);
		debugPrint(str);
	}

	debugPrint("Start IMU loop...\r\n");
  /* Infinite loop */
	for(;;)
	{
//		I2C_Read_IT(&hi2c1, 0x68, 0x3b, data, 6);
//		CHECK_IT(xInterruptFrequency);
//		for(int i=0;i<3;i++)
//			acc[i] = (int16_t)((data[2*i]<<8) | data[2*i+1]);
//		sprintf(str,"Ax: %d Ay: %d Az: %d\r\n",acc[0],acc[1],acc[2]);
//		debugPrint(str);

		vTaskDelay(delay ? delay : 1);
	}
  /* USER CODE END 5 */
}

void GPSR_Task(void const *argument) {
	const TickType_t delay = pdMS_TO_TICKS(10UL);
	const TickType_t waitDelay = pdMS_TO_TICKS(1000UL);
	uint8_t data[1] = {0};

	debugPrint("Start GPSR tasks...\r\n");
	debugPrint("Start GPSR loop...\r\n");
	for (;;) {
		if (!UART_Read_IT(&huart1, data, waitDelay))
			debugPrint(data);

		vTaskDelay(delay ? delay : 1);
	}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
