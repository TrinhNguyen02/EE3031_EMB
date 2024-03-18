/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum{LED_OFF, OFF_WAIT, LED_ON, ON_RELEASE, ON_WAIT, OFF_RELEASE} state_t;
typedef enum{BTN_RELEASE, BTN_PRESS} btn_t;

static state_t Status_GreenLED, Status_RedLED;
static btn_t Btn_1, Btn_2;
static btn_t StatusBtn(uint8_t sw);

int timerCount = 0;


btn_t StatusBtn(uint8_t sw){
	switch(sw){
	case 1:
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
			HAL_Delay(10);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) return BTN_RELEASE;
		}
		else return BTN_PRESS;
		break;

	case 2:
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)) {
			HAL_Delay(10);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)) return BTN_RELEASE;
		}
		else return BTN_PRESS;
		break;
	}
}

void checkBtn(){
	Btn_1 = StatusBtn(1);
	Btn_2 = StatusBtn(2);
}

void initState(){
	Status_RedLED = LED_OFF;
	Status_GreenLED = LED_OFF;
}

void stateMachine_SW1(void){
	switch(Status_RedLED)
	{
	case LED_OFF:
		if(Btn_1 == BTN_PRESS) Status_RedLED = ON_WAIT;
		break;

	case ON_WAIT:
		if(Btn_1 == BTN_RELEASE) Status_RedLED = LED_ON;
		break;

	case LED_ON:
		if(Btn_1 == BTN_PRESS) Status_RedLED = OFF_WAIT;
		break;

	case OFF_WAIT:
		if(Btn_1 == BTN_RELEASE) Status_RedLED = LED_OFF;
		break;

	default:
		Status_RedLED = LED_OFF;
		break;

	}

    // control Red LED
    controlLED(GPIOA, GPIO_PIN_3, Status_RedLED);

}

void stateMachine_SW2(void){
    switch (Status_GreenLED)
    {
    case LED_OFF:
        if(Btn_2==BTN_PRESS) {
            Status_GreenLED = OFF_WAIT;
            timerCount = 3000;
        }
        break;
    case OFF_WAIT:
        if(Btn_2==BTN_PRESS){
            if(timerCount==0){
                Status_GreenLED = LED_ON;
            }
        }
        else if(Btn_2==BTN_RELEASE){
            Status_GreenLED = LED_OFF;
        }
        break;
    case LED_ON:
        if(Btn_2==BTN_RELEASE){
            Status_GreenLED = ON_RELEASE;
        }
        break;
    case ON_RELEASE:
        if(Btn_2==BTN_PRESS){
            Status_GreenLED = ON_WAIT;
            timerCount = 6000;
        }
        break;
    case ON_WAIT:
        if(Btn_2==BTN_PRESS){
            if(timerCount == 0){
                Status_GreenLED = OFF_RELEASE;
            }
        }
        else if(Btn_2==BTN_RELEASE){
            Status_GreenLED = ON_RELEASE;
        }
        break;
    case OFF_RELEASE:
        if(Btn_2==BTN_RELEASE){
        	Status_GreenLED = LED_OFF;
        }
        break;
    default:
        Status_GreenLED = LED_OFF;
        break;
    }

    // control Green LED
    controlLED(GPIOA, GPIO_PIN_4, Status_GreenLED);

}

void controlLED(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, state_t status_LED){
	switch(status_LED){
	case LED_OFF:
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, RESET);
		break;

	case OFF_WAIT:
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, RESET);
		break;

	case LED_ON:
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, SET);
		break;

	case ON_RELEASE:
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, SET);
		break;

	case ON_WAIT:
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, SET);
		break;

	case OFF_RELEASE:
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, RESET);
		break;

	default:
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, RESET);
		break;
	}
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
  initState();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  checkBtn();
	  stateMachine_SW1();
	  stateMachine_SW2();
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
