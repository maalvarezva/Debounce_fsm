/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum _states_ {
	WAITING,
	DETECTED,
	WAIT_RELEASE,
	UPDATE,
	TOTAL_STATES
}states_t;

typedef enum _events_ {
	NON_EVENT,
	BUTTON_ON,
	TICK_TIMEOUT,
	TOTAL_EVENTS
}events_t;

typedef enum _bool_ {
	FALSE,
	TRUE
}bool_t;

typedef struct _fsm_ {
	states_t state;
	events_t event;
	uint8_t counter; /* each 10mSeg */
	bool_t new_event:1;
	bool_t start_countdown:1;
}fsm_t;

typedef struct _led_ {
	uint16_t period;
	volatile uint16_t counter; /* into the timer */
	uint8_t start;
}led_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
GPIO_PinState button_pressed ( GPIO_TypeDef *port, uint16_t pin );
void init_led_struct ( led_t *led );
void init_fsm ( fsm_t *sm );
void print_current_state ( fsm_t *fsm );
void run_fsm ( fsm_t *sm );
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define PERIOD	1000
#define TICK_PERIOD	10
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
