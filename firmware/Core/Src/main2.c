//#include "main.h"
//#include <string.h>
//#include <stdbool.h>
//#include <stdlib.h>
//
//TIM_HandleTypeDef htim2;
//DMA_HandleTypeDef hdma_tim2_ch1;
//UART_HandleTypeDef huart2;
//
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_DMA_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_USART2_UART_Init(void);
//
//#define MAX_LED 256
//volatile uint8_t datasentflag = 0;
//
//#define RESET_TIME 1
//#define TOTAL_SIZE (24*MAX_LED)+RESET_TIME
//uint8_t pwm_data[TOTAL_SIZE] = { 0 };
//
//void clear_all_leds(void) {
//	for (int i = 0; i < TOTAL_SIZE - RESET_TIME; ++i) {
//		pwm_data[i] = 14;
//	}
//}
//
//void enable_led(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue) {
//	uint32_t color = blue << 16 | red << 8 | green;
//	uint8_t *pwm_data_ptr = &pwm_data[led_index * 24];
//	for (int i = 23; i >= 0; i--) {
//		if (color & (1 << i)) {
//			pwm_data_ptr[i] = 26;  // 2/3 of 90
//		}
//
//		else
//			pwm_data_ptr[i] = 14;  // 1/3 of 90
//	}
//}
//
//bool send_ws2812() {
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) pwm_data,
//	TOTAL_SIZE);
//	while (!datasentflag) {
//	};
//
//	datasentflag = 0;
//	return true;
//}
//
///*!
// * Enable single led, disable all others led
// */
//bool enable_single_led(uint8_t led_index, uint8_t red, uint8_t green,
//		uint8_t blue) {
//
//	clear_all_leds();
//	enable_led(led_index, red, green, blue);
//	return send_ws2812();
//
//}
//
//#define MAX_X 16
//#define MAX_Y 16
//
//uint8_t get_led_index_from_coordinate(uint8_t x, uint8_t y) {
//	uint8_t led_index = 0;
//
//	if (x >= MAX_X || y >= MAX_Y) {
//		return false;
//	}
//	if (y % 2 == 0) {
//		led_index = x + (y * MAX_X);
//	} else {
//		led_index = (y * MAX_X + (MAX_X - 1)) - x;
//	}
//
//	return led_index;
//}
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
//	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
//	datasentflag = 1;
//}
//
//static uint8_t random_color() {
//	return rand() % 255 + 10;
//}
//
//static uint8_t random_ampl() {
//	return rand() % 17;
//}
//
//static void change_color(uint8_t *red, uint8_t *green, uint8_t *blue) {
//	*red = random_color();
//	*green = random_color();
//	*blue = random_color();
//}
//
//bool dvd_animation(void) {
//	static uint8_t x = 5;
//	static uint8_t y = 0;
//	static uint8_t dir_y = 1;
//	static uint8_t dir_x = 1;
//
//	static uint8_t red = 255;
//	static uint8_t green = 0;
//	static uint8_t blue = 0;
//
//	uint8_t led_index = get_led_index_from_coordinate(x, y);
//	enable_single_led(led_index, red, green, blue);
//
//	if (x >= (MAX_X - 2)) {
//		change_color(&red, &green, &blue);
//		dir_x = -1;
//		if (dir_y == 1) {
//			dir_y = 1;
//		} else {
//			dir_y = -1;
//		}
//	}
//	if (y == (MAX_Y - 1)) {
//		change_color(&red, &green, &blue);
//		dir_y = -1;
//		if (dir_x == 1) {
//			dir_x = 1;
//		} else {
//			dir_x = -1;
//		}
//	}
//
//	if (x == 1) {
//		change_color(&red, &green, &blue);
//		dir_x = 1;
//		if (dir_y == 1) {
//			dir_y = 1;
//		} else {
//			dir_y = -1;
//		}
//
//	}
//
//	if (y == 0) {
//		change_color(&red, &green, &blue);
//		dir_y = 1;
//		if (dir_x == 1) {
//			dir_x = 1;
//		} else {
//			dir_x = -1;
//		}
//
//	}
//
//	x += dir_x;
//	y += dir_y;
//
//	return true;
//}
//
//#define NUM_FRQ 7
//uint8_t frequence_table[NUM_FRQ];
//
//bool audio_animation(uint8_t frequence_table[NUM_FRQ]) {
//
//	for (int frq_index = 0; frq_index < NUM_FRQ; ++frq_index) {
//		int8_t amplitude = frequence_table[frq_index];
//
//		for (int amp = 0; amp < amplitude; ++amp) {
//			int8_t led_index = get_led_index_from_coordinate(
//					(frq_index * 2) + 2, amp);
//			if (amp > 12) {
//				enable_led(led_index, 0x60, 0, 0); //red
//			} else if (amp > 7) {
//				enable_led(led_index, 0x60, 0x60, 0); //orange
//			} else {
//				enable_led(led_index, 0, 0x60, 0); // green
//			}
//		}
//	}
//	send_ws2812();
//	return true;
//
//}
//int main(void) {
//
//	HAL_Init();
//	srand(0);
//	SystemClock_Config();
//
//	MX_GPIO_Init();
//	MX_DMA_Init();
//	MX_TIM2_Init();
//	MX_USART2_UART_Init();
//
//	while (1) {
////		int8_t led_index = get_led_index_from_coordinate(0, 0);
////		enable_led(led_index, 0, 0x60, 0); // green
////		int8_t led_index = get_led_index_from_coordinate(0, 0);
////		enable_led(led_index, 0, 0x60, 0); // green
////		int8_t led_index = get_led_index_from_coordinate(0, 0);
////		enable_led(led_index, 0, 0x60, 0); // green
////		int8_t led_index = get_led_index_from_coordinate(0, 0);
////		enable_led(led_index, 0, 0x60, 0); // green
////		int8_t led_index = get_led_index_from_coordinate(0, 0);
////		enable_led(led_index, 0, 0x60, 0); // green
////		int8_t led_index = get_led_index_from_coordinate(0, 0);
////		enable_led(led_index, 0, 0x60, 0); // green
//#if 0
//		dvd_animation();
//		HAL_Delay(1);
//#else
//		for (int var = 0; var < NUM_FRQ; ++var) {
//			frequence_table[var] = random_ampl();
//		}
////		frequence_table[0] = 5;
//		clear_all_leds();
//		audio_animation(frequence_table);
//		HAL_Delay(100);
//#endif
//
//	}
//}
//
///**
// * @brief System Clock Configuration
// * @retval None
// */
//void SystemClock_Config(void) {
//	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
//	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };
//
//	/** Configure the main internal regulator output voltage
//	 */
//	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//	/** Initializes the RCC Oscillators according to the specified parameters
//	 * in the RCC_OscInitTypeDef structure.
//	 */
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
//	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
//	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//		Error_Handler();
//	}
//	/** Initializes the CPU, AHB and APB buses clocks
//	 */
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
//			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
//		Error_Handler();
//	}
//	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
//	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
//	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
//		Error_Handler();
//	}
//}
//
///**
// * @brief TIM2 Initialization Function
// * @param None
// * @retval None
// */
//static void MX_TIM2_Init(void) {
//
//	/* USER CODE BEGIN TIM2_Init 0 */
//
//	/* USER CODE END TIM2_Init 0 */
//
//	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
//	TIM_OC_InitTypeDef sConfigOC = { 0 };
//
//	/* USER CODE BEGIN TIM2_Init 1 */
//
//	/* USER CODE END TIM2_Init 1 */
//	htim2.Instance = TIM2;
//	htim2.Init.Prescaler = 0;
//	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//	htim2.Init.Period = 40 - 1;
//	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
//		Error_Handler();
//	}
//	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
//			!= HAL_OK) {
//		Error_Handler();
//	}
//	sConfigOC.OCMode = TIM_OCMODE_PWM1;
//	sConfigOC.Pulse = 0;
//	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
//			!= HAL_OK) {
//		Error_Handler();
//	}
//	/* USER CODE BEGIN TIM2_Init 2 */
//
//	/* USER CODE END TIM2_Init 2 */
//	HAL_TIM_MspPostInit(&htim2);
//
//}
//
///**
// * @brief USART2 Initialization Function
// * @param None
// * @retval None
// */
//static void MX_USART2_UART_Init(void) {
//
//	/* USER CODE BEGIN USART2_Init 0 */
//
//	/* USER CODE END USART2_Init 0 */
//
//	/* USER CODE BEGIN USART2_Init 1 */
//
//	/* USER CODE END USART2_Init 1 */
//	huart2.Instance = USART2;
//	huart2.Init.BaudRate = 115200;
//	huart2.Init.WordLength = UART_WORDLENGTH_8B;
//	huart2.Init.StopBits = UART_STOPBITS_1;
//	huart2.Init.Parity = UART_PARITY_NONE;
//	huart2.Init.Mode = UART_MODE_TX_RX;
//	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//	if (HAL_UART_Init(&huart2) != HAL_OK) {
//		Error_Handler();
//	}
//	/* USER CODE BEGIN USART2_Init 2 */
//
//	/* USER CODE END USART2_Init 2 */
//
//}
//
///**
// * Enable DMA controller clock
// */
//static void MX_DMA_Init(void) {
//
//	/* DMA controller clock enable */
//	__HAL_RCC_DMA1_CLK_ENABLE();
//
//	/* DMA interrupt init */
//	/* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
//	HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
//
//}
//
///**
// * @brief GPIO Initialization Function
// * @param None
// * @retval None
// */
//static void MX_GPIO_Init(void) {
//	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
//
//	/* GPIO Ports Clock Enable */
//	__HAL_RCC_GPIOC_CLK_ENABLE();
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//
//	/*Configure GPIO pin Output Level */
//	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
//
//	/*Configure GPIO pin : LD3_Pin */
//	GPIO_InitStruct.Pin = LD3_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);
//
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
// * @brief  This function is executed in case of error occurrence.
// * @retval None
// */
//void Error_Handler(void) {
//	/* USER CODE BEGIN Error_Handler_Debug */
//	/* User can add his own implementation to report the HAL error return state */
//	__disable_irq();
//	while (1) {
//	}
//	/* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
//
