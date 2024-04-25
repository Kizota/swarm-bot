/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes */
#include "main.h"
#include <string.h>
#include <stdio.h>
/* Private includes */

#include <stdio.h>
#include <stdlib.h>

#include "Timer.h"
#include "Ultrasonic.h"
#include "360_servo.h"
#include "Robot.h"
#include "PID_control.h"


#define SETPOINT_DISTANCE 10 //cm
#define ROBOT_TIME_INTERVAL 2000 //ms
#define PID_POS_INTERVAL 20 //ms
/* Private typedef*/

/* Private define */

/* Private macro */

/* Private variables */
UART_HandleTypeDef huart2;

//ultrasonic
Echo_record record;
Distance distance;

//Servo
Servo l_wheel;
Servo r_wheel;

//robot
Robot robot;
Movement movement;

//PID control
PID pos_control;

//timer
Timer robo_timer;
Timer pos_timer;

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

//Timer configuration
void Timer_3_init();
void Timer_4_init();

/* Private user code*/

/**
 * @brief  The application entry point.
 * @retval int
 */
Timer test_timer;
int main(void) {

	/* MCU Configuration*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();
	printf("debug start!\n");
	fflush(stdout);

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable clock timer 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable clock timer 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable clock timer 4
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // Enable clock timer 15

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable clock for portA
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable clock for portB

	/* Initialize all configured peripherals */
	SysTick_init();
	Timer_2_init();
	Timer_3_init();
	Timer_4_init();
	/* debug purpose */
	printf("debug start!\n");
	fflush(stdout);

	//eyes
	create_ultrasonic_datas(&record, &distance, &TIM4->CCR3);

	//wheels
	create_servo(&l_wheel, &TIM2->CCR2);  //left wheel
	create_servo(&r_wheel, &TIM2->CCR1);  //right wheel

	//robot
	robot_init(&robot, &movement, &l_wheel, &r_wheel, &distance,
			SETPOINT_DISTANCE);

	//timer
	create_timer(&robo_timer, ROBOT_TIME_INTERVAL);
	create_timer(&pos_timer,PID_POS_INTERVAL);

	//PID init
	PID_init(&pos_control);

	//tunning
	tuning(&pos_control,1,0.3,0.7);

	//debugging
	//set_speed(&robot, -120);
    uint32_t preVal = 0;
	while (1) {

		//robot behaviour reaction
		drifting_robot(&robot, &movement,&pos_control,&robo_timer,&pos_timer);
		if(preVal != robot.eye->value)
		{
       printf("eye value: %lu",robot.eye->value);fflush(stdout);
       preVal = robot.eye->value;
		}		//update robot speed
		check_update_speed(&l_wheel);
		check_update_speed(&r_wheel);
	}
}

void SysTick_Handler(void) {
	timing(&robo_timer);
	timing(&pos_timer);

}
void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) {
		TIM2->SR &= ~TIM_SR_UIF;
	}

	if (TIM2->SR & TIM_SR_CC1IF) {
		TIM2->SR &= ~TIM_SR_CC1IF;
	}

	if (TIM2->SR & TIM_SR_CC2IF) {
		TIM2->SR &= ~TIM_SR_CC2IF;
	}
}

void TIM3_IRQHandler(void) {
	if (TIM3->SR & TIM_SR_UIF) {
		TIM3->SR &= ~TIM_SR_UIF;
	}

	if (TIM3->SR & TIM_SR_CC2IF) {
		TIM3->SR &= ~TIM_SR_CC2IF;
	}
}
void TIM4_IRQHandler(void) {
	if (TIM4->SR & TIM_SR_UIF) {
		fflush(stdout);
		TIM4->SR &= ~TIM_SR_UIF;
	}

	if (TIM4->SR & TIM_SR_CC3IF) {
		check_ultra_sonic_info(&record, &distance);
		TIM4->SR &= ~TIM_SR_CC3IF;
	}
}

void TIM15_IRQHandler(void) {
	if (TIM15->SR & TIM_SR_UIF) {
		TIM15->SR &= ~TIM_SR_UIF;
	}

	if (TIM15->SR & TIM_SR_CC1IF) {
		TIM15->SR &= ~TIM_SR_CC1IF;
	}

	if (TIM15->SR & TIM_SR_CC2IF) {

		TIM15->SR &= ~TIM_SR_CC2IF;
	}

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

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

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
