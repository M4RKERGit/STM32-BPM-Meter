/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_LEN  1
UART_HandleTypeDef huart1;
uint8_t RX_BUFFER[BUFFER_LEN] = {0};
#define SELECTION_SIZE 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int calcBPM(uint32_t *lastBeats, int size);
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
int calcBPM(uint32_t *lastBeats, int size)
{
	float freq;
	uint32_t averageTime;
	int32_t difference[7] = {0, 0, 0, 0, 0, 0, 0};
	uint32_t sum = 0;
	for (int i = 0; i < (size - 1); i++)
	{
		difference[i] = (lastBeats[i+1] - lastBeats[i]);
		if (difference[i] < 0) difference[i] = difference[i]*(-1);
		sum += difference[i];
	}
	averageTime = sum / (size - 1);
	freq = 1/averageTime;
	int toRet = freq * 60;
	return toRet;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastBeats[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  int arrayCounter = 0;
  int BPM = 0;
  char buf[32];
  /*while(1)																	//кусок для теста и отладки
  {
	  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2))
	  	  {
	  		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  		  uint32_t jojo = HAL_GetTick();
	  		  HAL_UART_Transmit(&huart1, (uint8_t*)jojo, sizeof(32), 1000);
	  		  HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 2, 1000);
	  		  HAL_Delay(250);
	  	  }
  }*/
  while (1)
  {
	  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2))
	  {
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  lastBeats[arrayCounter] = HAL_GetTick();

		  arrayCounter ++;
		  if (arrayCounter == 8)
		  {
			  arrayCounter = 0;
		  }


		  float freq;
		  float averageTime;
		  uint32_t difference[7] = {0, 0, 0, 0, 0, 0, 0};
		  long sum = 0;
		  for (int i = 0; i < 7; i++)
		  {
		  	difference[i] = lastBeats[i+1] - lastBeats[i];
		  	sum += difference[i];
		  }
		  averageTime = sum / (7);
		  freq = 1000/averageTime;
		  BPM = freq * 60;




		  //BPM = calcBPM(lastBeats, SELECTION_SIZE);
		  sprintf(buf, "%d", BPM);	//вот тут вылетает
		  HAL_Delay(100);
	  }
	  if ((HAL_GetTick() % 1001) >= 1000)
	  {
		  HAL_UART_Transmit(&huart1, (uint8_t*)buf, sizeof(32), 1000);
		  HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 2, 1000);
	  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart1.Instance)
    {
	HAL_UART_Receive_IT(&huart1, RX_BUFFER, BUFFER_LEN);
    }
}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
