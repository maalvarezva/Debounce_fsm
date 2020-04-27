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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

led_t led2;
fsm_t fsm_button;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* each 1 mSeg */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	led2.counter++; /* */

	if (fsm_button.start_countdown == TRUE) {
		fsm_button.counter++;
	}

	if (led2.counter >= led2.period && led2.start == 1) { /* */
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); /* */
		led2.counter = 0; /* */
	}
	/* 10 mSeg elapsed*/
	if (fsm_button.counter >= TICK_PERIOD) {
		fsm_button.counter = 0;
		fsm_button.start_countdown = FALSE;
		fsm_button.new_event = TRUE;
		fsm_button.event = TICK_TIMEOUT;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if ( fsm_button.state == WAITING ) {
		fsm_button.event = BUTTON_ON;
		fsm_button.new_event = TRUE;
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
	init_led_struct(&led2);
	init_fsm(&fsm_button);
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
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Transmit(&huart2, (uint8_t *)"Initializing fsm...\n", sizeof("Initializing fsm...\n")-1, 100);
	print_current_state(&fsm_button);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		run_fsm(&fsm_button);
//
//		if (fsm_button.new_event == TRUE) {
//			fsm_button.new_event = FALSE;
//			switch (fsm_button.state) {
//			case WAITING:
//				print_current_state(&fsm_button);
//				if ( fsm_button.event == BUTTON_ON ) {
//					fsm_button.start_countdown = TRUE; /* init countdown of timer ISR */
//					fsm_button.state = DETECTED; /* next state of fsm */
//				}
//				break;
//			case DETECTED:
//				print_current_state(&fsm_button);
//				if ( fsm_button.event == TICK_TIMEOUT ) {
//					if (button_pressed(B1_GPIO_Port, B1_Pin)) {
//						fsm_button.state = WAIT_RELEASE; /* next state */
//						fsm_button.start_countdown = TRUE;
//					} else {
//						fsm_button.state = WAITING;
//					}
//				}
//				break;
//			case WAIT_RELEASE:
//				print_current_state(&fsm_button);
//				if ( fsm_button.event == TICK_TIMEOUT ) {
//					if (!button_pressed(B1_GPIO_Port, B1_Pin)) {
//						fsm_button.state = UPDATE;
//						fsm_button.start_countdown = TRUE;
//					} else {
//						fsm_button.state = WAIT_RELEASE;
//						fsm_button.start_countdown = TRUE;
//					}
//				}
//				break;
//			case UPDATE:
//				print_current_state(&fsm_button);
//				if ( fsm_button.event == TICK_TIMEOUT ) {
//					fsm_button.state = WAITING;
//					fsm_button.event = NON_EVENT;
//					fsm_button.new_event = FALSE;
//					print_current_state(&fsm_button);
//					HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//				}
//				break;
//			default:
//				print_current_state(&fsm_button);
//				HAL_UART_Transmit(&huart2, (uint8_t *)"Unknown State", sizeof("Unknown State"), 100);
//				while (1);
//				break;
//			}
//		}

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 16;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void init_led_struct ( led_t *led ) {
	led->counter = 0;
	led->period = PERIOD;
	led->start = 0;
}

void init_fsm ( fsm_t *sm ) {
	sm->state = WAITING;
	sm->event = NON_EVENT;
	sm->new_event = FALSE;
	sm->counter = 0;
	sm->start_countdown = 0;
}

void run_fsm ( fsm_t *sm ) {
	if (sm->new_event == TRUE) {
		sm->new_event = FALSE;
		switch (sm->state) {
		case WAITING:
			print_current_state(sm);
			if ( sm->event == BUTTON_ON ) {
				sm->start_countdown = TRUE; /* init countdown of timer ISR */
				sm->state = DETECTED; /* next state of fsm */
			}
			break;
		case DETECTED:
			print_current_state(sm);
			if ( sm->event == TICK_TIMEOUT ) {
				if (button_pressed(B1_GPIO_Port, B1_Pin)) {
					sm->state = WAIT_RELEASE; /* next state */
					sm->start_countdown = TRUE;
				} else {
					sm->state = WAITING;
				}
			}
			break;
		case WAIT_RELEASE:
			print_current_state(sm);
			if ( sm->event == TICK_TIMEOUT ) {
				if (!button_pressed(B1_GPIO_Port, B1_Pin)) {
					sm->state = UPDATE;
					sm->start_countdown = TRUE;
				} else {
					sm->state = WAIT_RELEASE;
					sm->start_countdown = TRUE;
				}
			}
			break;
		case UPDATE:
			print_current_state(sm);
			if ( sm->event == TICK_TIMEOUT ) {
				sm->state = WAITING;
				sm->event = NON_EVENT;
				sm->new_event = FALSE;
				print_current_state(sm);
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}
			break;
		default:
			print_current_state(sm);
			HAL_UART_Transmit(&huart2, (uint8_t *)"Unknown State", sizeof("Unknown State"), 100);
			while (1);
			break;
		}
	}
}
GPIO_PinState button_pressed ( GPIO_TypeDef *port, uint16_t pin ) {
	return !HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
}

void print_current_state ( fsm_t *fsm ) {
	switch (fsm->state) {
	case WAITING:
		HAL_UART_Transmit(&huart2, (uint8_t*)"WAITING\n", sizeof("WAITING\n")-1, 100);
		break;
	case DETECTED:
		HAL_UART_Transmit(&huart2, (uint8_t*)"DETECTED\n", sizeof("DETECTED\n")-1, 100);
		break;
	case WAIT_RELEASE:
		HAL_UART_Transmit(&huart2, (uint8_t*)"WAIT FOR RELEASE\n", sizeof("WAIT FOR RELEASE\n")-1, 100);
		break;
	case UPDATE:
		HAL_UART_Transmit(&huart2, (uint8_t*)"UPDATE\n", sizeof("UPDATE\n")-1, 100);
		break;
	default:
		HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR\n", sizeof("ERROR\n")-1, 100);
		break;
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
