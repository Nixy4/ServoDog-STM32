/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include "quadruped.h"
#include "elog.h"
#include "stdio.h"

#define TAG "Main"

void elog_init_default()
{
  elog_init();
  elog_set_fmt(ELOG_LVL_ASSERT,   ELOG_FMT_ALL);
  elog_set_fmt(ELOG_LVL_ERROR,    ELOG_FMT_FUNC|ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_WARN,     ELOG_FMT_FUNC|ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_INFO,     ELOG_FMT_FUNC|ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_DEBUG,    ELOG_FMT_FUNC|ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_VERBOSE,  ELOG_FMT_FUNC|ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_filter_lvl(ELOG_LVL_ASSERT);
  elog_start();
}

quad_acb_cfg tacb_cfg = 
{
  .calc = _easing_calc_Linear,
  .frame_cfg = {
    .interval = 0,
  },
};

quad_acb_cfg sacb_cfg = 
{
  .calc = _easing_calc_Linear,
  .frame_cfg = {
    .interval = 0,
  },
};

quad_ccb_cfg ccb_cfg = 
{
  .calc_x = _easing_calc_Linear,
  .calc_z = _easing_calc_Linear,
  .frame_cfg = {
    .interval = 0,
  },
};

quad_leg_cfg leg_cfg_rf = 
{
  .type = LEG_RF,
  .thigh_servo_cfg = {
    .channel = 1,
    .offset = CONFIG_ANGLE_OFFSET_RF_THIGH+CONFIG_ANGLE_OFFSET_GLOBAL_THIGH,
  },
  .shank_servo_cfg = {
    .channel = 0,
    .offset = CONFIG_ANGLE_OFFSET_RF_SHANK+CONFIG_ANGLE_OFFSET_GLOBAL_SHANK,
  },
  .tacb_cfg = &tacb_cfg,
  .sacb_cfg = &sacb_cfg,
  .ccb_cfg  = &ccb_cfg,
};

quad_leg_cfg leg_cfg_lf = 
{
  .type = LEG_LF,
  .thigh_servo_cfg = {
    .channel = 3,
    .offset = CONFIG_ANGLE_OFFSET_LF_THIGH+CONFIG_ANGLE_OFFSET_GLOBAL_THIGH,
  },
  .shank_servo_cfg = {
    .channel = 2,
    .offset = CONFIG_ANGLE_OFFSET_LF_SHANK+CONFIG_ANGLE_OFFSET_GLOBAL_SHANK,
  },
  .tacb_cfg = &tacb_cfg,
  .sacb_cfg = &sacb_cfg,
  .ccb_cfg  = &ccb_cfg,
};

quad_leg_cfg leg_cfg_rb = 
{
  .type = LEG_RB,
  .thigh_servo_cfg = {
    .channel = 5,
    .offset = CONFIG_ANGLE_OFFSET_RB_THIGH+CONFIG_ANGLE_OFFSET_GLOBAL_THIGH,
  },
  .shank_servo_cfg = {
    .channel = 4,
    .offset = CONFIG_ANGLE_OFFSET_RB_SHANK+CONFIG_ANGLE_OFFSET_GLOBAL_SHANK,
  },
  .tacb_cfg = &tacb_cfg,
  .sacb_cfg = &sacb_cfg,
  .ccb_cfg  = &ccb_cfg,
};

quad_leg_cfg leg_cfg_lb = 
{
  .type = LEG_LB,
  .thigh_servo_cfg = {
    .channel = 7,
    .offset = CONFIG_ANGLE_OFFSET_LB_THIGH+CONFIG_ANGLE_OFFSET_GLOBAL_THIGH,
  },
  .shank_servo_cfg = {
    .channel = 6,
    .offset = CONFIG_ANGLE_OFFSET_LB_SHANK+CONFIG_ANGLE_OFFSET_GLOBAL_SHANK,
  },
  .tacb_cfg = &tacb_cfg,
  .sacb_cfg = &sacb_cfg,
  .ccb_cfg  = &ccb_cfg,
};

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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("MX Init Success\r\n");
  elog_init_default();
  // elog_set_filter_lvl(ELOG_LVL_INFO);
  // elog_set_filter_lvl(ELOG_LVL_DEBUG);
  // elog_set_filter_lvl(ELOG_LVL_VERBOSE);
  
//TODO:MAIN
  servo_set_freq(50);

  leg_init(&leg_rf, leg_cfg_rf);
  leg_init(&leg_lf, leg_cfg_lf);
  leg_init(&leg_rb, leg_cfg_rb);
  leg_init(&leg_lb, leg_cfg_lb);

  walk_init(35.f, 35.f, 0.5f);
  sil_init(40.f, 0.5f);

  fixed_set_angle_zero();
  // HAL_Delay(1000);
  // fixed_set_angle_mid();
  // HAL_Delay(1000);
  // fixed_stand_by_coord0();
  HAL_Delay(100);
  // fixed_stand_by_acb(500);
  // fixed_stand_by_ccb(500);
  // walk(150, 10);
  sil(150, 10);
  // sync_sil(150, 10);
  // fixed_stand_by_acb(500);

  // quad_coord p1 = {0, 90};
  // quad_coord p2 = {0, 115};

  // for(int i=0;i<5;++i)
  // {
  //   leg_ccb_target(&leg_rf, &leg_rf.ccb, p1, 500);
  //   leg_ccb_target(&leg_lf, &leg_lf.ccb, p1, 500);
  //   leg_ccb_target(&leg_rb, &leg_rb.ccb, p1, 500);
  //   leg_ccb_target(&leg_lb, &leg_lb.ccb, p1, 500);
  //   leg_ccb_update_all_block();
  //   HAL_Delay(1000);
  //   leg_ccb_target(&leg_rf, &leg_rf.ccb, p2, 500);
  //   leg_ccb_target(&leg_lf, &leg_lf.ccb, p2, 500);
  //   leg_ccb_target(&leg_rb, &leg_rb.ccb, p2, 500);
  //   leg_ccb_target(&leg_lb, &leg_lb.ccb, p2, 500);
  //   leg_ccb_update_all_block();
  // }


  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLN = 168;
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

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16800;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF5 PF6 PF7 PF8
                           PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
