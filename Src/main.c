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

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "elog.h"

static const char* TAG = "MAIN";

void elog_init_default()
{
  elog_init();
  elog_set_fmt(ELOG_LVL_ASSERT,   ELOG_FMT_ALL);// 参数是否为空
  elog_set_fmt(ELOG_LVL_ERROR,    ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_WARN,     ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_INFO,     ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_DEBUG,    ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_VERBOSE,  ELOG_FMT_FUNC|ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_filter_lvl(ELOG_LVL_DEBUG);
  elog_start();
}

// const leg_config_t rf_cfg = {
//   .id = LEG_ID_RF,
//   .thighServoId = 0,
//   .thighAngle = 0,
//   .thighOffset = 0,
//   .shankServoId = 5,
//   .shankAngle = 0,
//   .shankOffset = 0
// };

// const leg_config_t rb_cfg = {
//   .id = LEG_ID_RB,
//   .thighServoId = 1,
//   .thighAngle = 0,
//   .thighOffset = 0,
//   .shankServoId = 4,
//   .shankAngle = 0,
//   .shankOffset = 0
// };

// const leg_config_t lf_cfg = {
//   .id = LEG_ID_LF,
//   .thighServoId = 3,
//   .thighAngle = 0,
//   .thighOffset = -10,
//   .shankServoId = 7,
//   .shankAngle = 0,
//   .shankOffset = 0
// };

// const leg_config_t lb_cfg = {
//   .id = LEG_ID_LB,
//   .thighServoId = 2,
//   .thighAngle = 0,
//   .thighOffset = -10,
//   .shankServoId = 6,
//   .shankAngle = 0,
//   .shankOffset = 0
// };

// const leg_config_t* leg_cfg[4] = {
//   &rf_cfg,
//   &rb_cfg,
//   &lf_cfg,
//   &lb_cfg
// };

// void motion_init()
// {
//   legs_init();
//   elog_i(TAG, "Dog Init Success");
// }

// void motion_stand0(double ms)
// {
//   legs_move_block_x0_z_max(ms);
//   elog_i(TAG, "Dog Stand !");
// }

// void motion_stand1(double ms)
// {
//   double X = -10;
//   double Z = 110;
//   leg_move_target(&rf, X, Z, ms);
//   leg_move_target(&rb, X, Z, ms);
//   leg_move_target(&lf, X, Z, ms);
//   leg_move_target(&lb, X, Z, ms);
//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);
// }

// void motion_fall0(double ms)
// {
//   legs_move_block_start(ms);
//   elog_i(TAG, "Dog Fall !");
// }

// void motion_fall1(double ms)
// {
//   double AS1 = 50;
//   double AS2 = 0;
//   leg_turn_target(&rf, AS1, AS2, ms);
//   leg_turn_target(&rb, AS1+5, AS2, ms);
//   leg_turn_target(&lf, AS1, AS2, ms);
//   leg_turn_target(&lb, AS1+5, AS2, ms);
//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);
//   HAL_Delay(ms);
// }

// void motion_squat0(double ms)
// {
//   leg_move_target(&rf, x0_z_max.X, x0_z_max.Z, ms);
//   leg_move_target(&rb, start.X, start.Z, ms);
//   leg_move_target(&lf, x0_z_max.X, x0_z_max.Z, ms);
//   leg_move_target(&lb, start.X, start.Z, ms);
//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);
//   HAL_Delay(ms);  
//   elog_i(TAG, "Dog Squat !");
// }

// void motion_squat1(double ms)
// {
//   double FAS1 = 30;
//   double FAS2 = 115;
//   double BAS1 = 40;
//   double BAS2 = 0;
//   leg_turn_target(&rf, FAS1, FAS2, ms);
//   leg_turn_target(&lf, FAS1, FAS2, ms);
//   leg_turn_target(&rb, BAS1, BAS2, ms);
//   leg_turn_target(&lb, BAS1, BAS2, ms);
//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);
//   HAL_Delay(ms);
// }

// void motion_bow0(double ms)
// {
//   //1.先恢复站立
//   motion_stand0(ms);
//   //2.前蹲
//   double FAS1= 30;
//   double FAS2= 30;
//   leg_turn_target(&rf, FAS1, FAS2, ms);
//   leg_turn_target(&lf, FAS1, FAS2, ms);
//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&lf);
//   } while(flag != 0);
//   HAL_Delay(ms);
//   //3.恢复站立
//   motion_stand0(ms);
//   //4.前蹲
//   leg_turn_target(&rf, FAS1, FAS2, ms);
//   leg_turn_target(&lf, FAS1, FAS2, ms);
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&lf);
//   } while(flag != 0);
//   HAL_Delay(ms);
//   //5.恢复站立
//   motion_stand0(ms);
// }

// #define WALK_X_BASE       30.f
// #define WALK_Z_BASE       115.f
// #define WALK_PERIOD       200.f
// #define WALK_SWING_DUTY   0.5f
// #define WALK_SWING_TIME   (WALK_PERIOD*WALK_SWING_DUTY)
// #define WALK_SWING_WIDTH  30.f
// #define WALK_SWING_HEIGHT 30.f

// gait_t walk = 
// {
//   .periedTick = WALK_PERIOD,
//   .swingDuty = WALK_SWING_DUTY,
//   .swingTime = WALK_SWING_TIME,
//   .swingWidth = WALK_SWING_WIDTH,
//   .swingHeight = WALK_SWING_HEIGHT,
//   .xBase = WALK_X_BASE,
//   .zBase = WALK_Z_BASE,
//   .deltaTick = 1,
//   .rfPtr = &rf,
//   .rbPtr = &rb,
//   .lfPtr = &lf,
//   .lbPtr = &lb,
//   .t = 0,
// };

// void walk0(double ms)
// {
//   leg_move_target(&rf, walk.xBase, walk.zBase, ms);
//   leg_move_target(&rb, walk.xBase, walk.zBase, ms);
//   leg_move_target(&lf, walk.xBase, walk.zBase, ms);
//   leg_move_target(&lb, walk.xBase, walk.zBase, ms);
//   legs_update_block();
// }

// void walk1(double ms)
// {
//   // leg_move_target(&rf, walk.xBase, walk.zBase, ms);
//   leg_move_target(&rb, walk.xBase, walk.zBase, ms);
//   // leg_move_target(&lf, walk.xBase, walk.zBase, ms);
//   leg_move_target(&lb, walk.xBase, walk.zBase, ms);
//   legs_update_block();
// }

// void walk2(int steps)
// {
//   for(int i = 0; i < steps; i++)
//   {
//     while(gait_update(&walk) != 0);
//   }
// }

#include "pca9685.h"
#include "quadruped.h"

LegConfig rf_cfg = 
{
  .pcaChannel1 = 0,
  .pcaChannel2 = 5,
  .offset1 = 0,
  .offset2 = 0,
  .angle1 = 0,
  .angle2 = 0,
  .ea1_config = EASING_ANGLE_CONFIG_DEFAULT(),
  .ea2_config = EASING_ANGLE_CONFIG_DEFAULT(),
  .ec_config = EASING_COORD_CONFIG_DEFAULT(),
};

LegConfig rb_cfg = 
{
  .pcaChannel1 = 1,
  .pcaChannel2 = 4,
  .offset1 = 0,
  .offset2 = 0,
  .angle1 = 0,
  .angle2 = 0,
  .ea1_config = EASING_ANGLE_CONFIG_DEFAULT(),
  .ea2_config = EASING_ANGLE_CONFIG_DEFAULT(),
  .ec_config = EASING_COORD_CONFIG_DEFAULT(),
};

LegConfig lf_cfg = 
{
  .pcaChannel1 = 3,
  .pcaChannel2 = 7,
  .offset1 = 0,
  .offset2 = 0,
  .angle1 = 0,
  .angle2 = 0,
  .ea1_config = EASING_ANGLE_CONFIG_DEFAULT(),
  .ea2_config = EASING_ANGLE_CONFIG_DEFAULT(),
  .ec_config = EASING_COORD_CONFIG_DEFAULT(),
};

LegConfig lb_cfg = 
{
  .pcaChannel1 = 2,
  .pcaChannel2 = 6,
  .offset1 = 0,
  .offset2 = 0,
  .angle1 = 0,
  .angle2 = 0,
  .ea1_config = EASING_ANGLE_CONFIG_DEFAULT(),
  .ea2_config = EASING_ANGLE_CONFIG_DEFAULT(),
  .ec_config = EASING_COORD_CONFIG_DEFAULT(),
};

void setup()
{
  pca9685_set_freq(50);
  leg_init(LEG_ID_RF, &rf_cfg);
  leg_init(LEG_ID_RB, &rb_cfg);
  leg_init(LEG_ID_LF, &lf_cfg);
  leg_init(LEG_ID_LB, &lb_cfg);
}

void loop()
{

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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  printf("MX Init Success\r\n");

  elog_init_default();

  setup();
  loop();

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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
