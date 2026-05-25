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
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	//Timer are connected to Bus Clock, (eg, Tim1 to APB2)
	//timer counts up till it ARR then reset to 0

	//	Timer_Clock = APB_TIMER_CLOCK / PRESCALER_SETTING
	//	Clock Freq = PWM Freq = Timer_Clock	/ Auto_Reload_Register(ARR)
	//	Duty% = (Capture_Compare_Register / ARR)*100

	//eg: Timer_Clock = 180 MHz / 180-1	= 1MHz, 1 count = 1 microsecond
	//	  PWM Freq = 1Mhz / 1000-1 = 1kHz, ARR = 1000 counts
	//	  Duty = 35% for CCRx = 350
	//PWM signal has T=1ms and is high only for the first 350 counts of the Timer_Clock

	//we can use DMA to move data to CRRx efficiently in runtime
	//Data direction in this case is Memory->Peripheral (ram to CCRx)
	//half byte because we use data bigger than 8bits

uint16_t dutyCycles[10];
int count = 0;
int DMAcount=0;

// Pre-computed 32-point sine wave scaled for ARR = 1000
const uint16_t SINE_WAVE_LUT_32[32] = {
    500, 597, 691, 777, 853, 915, 961, 990,
    1000, 990, 961, 915, 853, 777, 691, 597,
    500, 402, 308, 222, 146, 84,  38,  9,
    0,   9,   38,  84,  146, 222, 308, 402
};

// Pre-computed 512-point sine wave scaled for ARR = 1000
const uint16_t SINE_WAVE_LUT_512[512] = {
     500,  506,  512,  518,  525,  531,  537,  543,  549,  555,  561,  567,  573,  579,  585,  592,
     598,  604,  610,  616,  621,  627,  633,  639,  645,  651,  657,  663,  668,  674,  680,  686,
     691,  697,  703,  708,  714,  719,  725,  730,  736,  741,  746,  752,  757,  762,  767,  773,
     778,  783,  788,  793,  798,  803,  808,  812,  817,  822,  827,  831,  836,  840,  845,  849,
     854,  858,  862,  866,  870,  875,  879,  883,  887,  890,  894,  898,  902,  905,  909,  912,
     916,  919,  922,  926,  929,  932,  935,  938,  941,  944,  947,  949,  952,  955,  957,  960,
     962,  964,  966,  969,  971,  973,  975,  977,  978,  980,  982,  983,  985,  986,  988,  989,
     990,  992,  993,  994,  995,  995,  996,  997,  998,  998,  999,  999,  999, 1000, 1000, 1000,
    1000, 1000, 1000, 1000,  999,  999,  999,  998,  998,  997,  996,  995,  995,  994,  993,  992,
     990,  989,  988,  986,  985,  983,  982,  980,  978,  977,  975,  973,  971,  969,  966,  964,
     962,  960,  957,  955,  952,  949,  947,  944,  941,  938,  935,  932,  929,  926,  922,  919,
     916,  912,  909,  905,  902,  898,  894,  890,  887,  883,  879,  875,  870,  866,  862,  858,
     854,  849,  845,  840,  836,  831,  827,  822,  817,  812,  808,  803,  798,  793,  788,  783,
     778,  773,  767,  762,  757,  752,  746,  741,  736,  730,  725,  719,  714,  708,  703,  697,
     691,  686,  680,  674,  668,  663,  657,  651,  645,  639,  633,  627,  621,  616,  610,  604,
     598,  592,  585,  579,  573,  567,  561,  555,  549,  543,  537,  531,  525,  518,  512,  506,
     500,  494,  488,  482,  475,  469,  463,  457,  451,  445,  439,  433,  427,  421,  415,  408,
     402,  396,  390,  384,  379,  373,  367,  361,  355,  349,  343,  337,  332,  326,  320,  314,
     309,  303,  297,  292,  286,  281,  275,  270,  264,  259,  254,  248,  243,  238,  233,  227,
     222,  217,  212,  207,  202,  197,  192,  188,  183,  178,  173,  169,  164,  160,  155,  151,
     146,  142,  138,  134,  130,  125,  121,  117,  113,  110,  106,  102,   98,   95,   91,   88,
      84,   81,   78,   74,   71,   68,   65,   62,   59,   56,   53,   51,   48,   45,   43,   40,
      38,   36,   34,   31,   29,   27,   25,   23,   22,   20,   18,   17,   15,   14,   12,   11,
      10,    8,    7,    6,    5,    5,    4,    3,    2,    2,    1,    1,    1,    0,    0,    0,
       0,    0,    0,    0,    1,    1,    1,    2,    2,    3,    4,    5,    5,    6,    7,    8,
      10,   11,   12,   14,   15,   17,   18,   20,   22,   23,   25,   27,   29,   31,   34,   36,
      38,   40,   43,   45,   48,   51,   53,   56,   59,   62,   65,   68,   71,   74,   78,   81,
      84,   88,   91,   95,   98,  102,  106,  110,  113,  117,  121,  125,  130,  134,  138,  142,
     146,  151,  155,  160,  164,  169,  173,  178,  183,  188,  192,  197,  202,  207,  212,  217,
     222,  227,  233,  238,  243,  248,  254,  259,  264,  270,  275,  281,  286,  292,  297,  303,
     309,  314,  320,  326,  332,  337,  343,  349,  355,  361,  367,  373,  379,  384,  390,  396,
     402,  408,  415,  421,  427,  433,  439,  445,  451,  457,  463,  469,  475,  482,  488,  494
};

//DMA calls this callback after finishing transferring data to the timer CCRx
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	//	if (DMAcount >= 1000)
	//	{
	//		HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);	//stop the DMA in circular mode
	//	}
	DMAcount++;
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
  MX_DMA_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */



//  //Simple PWM Output
//  	 TIM1->CCR1 = 25;	//load CCR register of channel 1 of timer 1
//  	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	//start normal PWM Output to channel 1



//  //PWM Output using DMA
//
//  	 //set data for DMA to move
//  	 dutyCycles[0]=10;dutyCycles[1]=20;dutyCycles[2]=30;dutyCycles[3]=40;dutyCycles[4]=50;
//  	 dutyCycles[5]=60;dutyCycles[6]=70;dutyCycles[7]=80;dutyCycles[8]=90;dutyCycles[9]=100;
//
//  	 //DMA will send 10 elements from dutyCycles array directly into CCR1 of TIM1
//  	 HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)dutyCycles, 10);


  	  	 HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)SINE_WAVE_LUT_512, 512);
 	  	 //LED fade in and out smoothly like a sine wave
  	  	 //eg: Timer_Clock = 180 MHz / 180-1	= 1 MHz
  		 //	   PWM Freq = 1 Mhz / 1000-1 = 1 kHz (flicker starts to be unnoticeable)

  	  	 //frequency_sine_wave = frequency_PWM / Sine_Wave_LUT_Points
  	  	 //	f_32 = 1kHz/32 = 31.25Hz, f_512 = 1kHz/512 = 1.92Hz

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  //Update duty cycle dynamically using set CCR function
//	  //pwm must be started first for the set CCR function to work
//	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//
//	  for(int i = 0; i < 100; i += 5)
//	  {
//		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, i);  // TIM1->CCR1 = i;
//	   	  HAL_Delay(500);  // Wait 500ms before changing duty cycle
//
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  count++;
//	  }


	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  count++;
	  HAL_Delay(500);


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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
