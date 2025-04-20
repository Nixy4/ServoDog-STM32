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
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>
#include "math.h"
#include "pca9685.h"
#include "easing.h"
#include "elog.h"
#include "hal_hw_uart_printf.h"
#include "aeabi.h"

static const char* TAG = "MAIN";

void pca9685_set_angle_all(double angle)
{
  uint32_t off = 0;
  off = __aeabi_d2ulz(angle * 2.276 + 0.5)+102;
  for (uint8_t i = 0; i < 16; i++) {
    pca9685_set_pwm(i, 0, off);
  }
}

#define SERVO_EASING_FRAME_CNT 50
#define SERVO_EASING_INTERVAL  2 // 1.67ms
#define SERVO_ACTION_MS        (SERVO_EASING_FRAME_CNT*SERVO_EASING_INTERVAL)
#define SERVO_DEFUALT_CALC     _easing_calc_InOutBounce

/**
 * F = Front
 * B = Back
 * 
 * R = Right
 * L = Left
 * 
 * T = Thighs
 * C = Calves
 */
// enum
// {
//   SERVO_ID_FRT = 0,
//   SERVO_ID_FRC,
//   SERVO_ID_FLT,
//   SERVO_ID_FLC,
//   SERVO_ID_BRC,
//   SERVO_ID_BRT,
//   SERVO_ID_BLC,
//   SERVO_ID_BLT,
//   SERVO_ID_MAX
// };

#define SERVO_LEDX_RFT 0
#define SERVO_LEDX_RFC 5

#define SERVO_LEDX_FLT 3
#define SERVO_LEDX_FLC 7

#define SERVO_LEDX_BRT 1
#define SERVO_LEDX_BRC 4

#define SERVO_LEDX_BLT 2
#define SERVO_LEDX_BLC 6

#define SERVO_ID_FRT SERVO_LEDX_RFT
#define SERVO_ID_FRC SERVO_LEDX_RFC
#define SERVO_ID_FLT SERVO_LEDX_FLT
#define SERVO_ID_FLC SERVO_LEDX_FLC
#define SERVO_ID_BRC SERVO_LEDX_BRC
#define SERVO_ID_BRT SERVO_LEDX_BRT
#define SERVO_ID_BLC SERVO_LEDX_BLC
#define SERVO_ID_BLT SERVO_LEDX_BLT
#define SERVO_ID_MAX 8

static const char* get_id_string(uint8_t id)
{
  switch (id)
  {
  case SERVO_LEDX_RFT: return "前右大腿";
  case SERVO_LEDX_RFC: return "前右小腿";
  case SERVO_LEDX_FLT: return "前左大腿";
  case SERVO_LEDX_FLC: return "前左小腿";
  case SERVO_LEDX_BRT: return "后右大腿";
  case SERVO_LEDX_BRC: return "后右小腿";
  case SERVO_LEDX_BLC: return "后左大腿";
  case SERVO_LEDX_BLT: return "后左小腿";
       default       : return "未知";
  }
}

typedef struct servo
{
  easing angle;
  float last_angle;
}servo_t;

static servo_t sl[SERVO_ID_MAX];

static void servo_init_pre(uint8_t id, easing_mode_t mode, easing_calc_fn calc, float off)
{
  easing_init(&sl[id].angle, mode, calc, SERVO_EASING_FRAME_CNT, off, 0);
  sl[id].last_angle = off;
  pca9685_set_angle(id, easing_get_float(&sl[id].angle));
  elog_d(TAG, "servo %s init , off %f°", get_id_string(id), off);
}


static void servo_init()
{
  pca9685_set_freq(50);

  servo_init_pre(SERVO_LEDX_RFT, EASING_MODE_DEFAULT, SERVO_DEFUALT_CALC, 0);
  servo_init_pre(SERVO_LEDX_RFC, EASING_MODE_DEFAULT, SERVO_DEFUALT_CALC, 0);
  servo_init_pre(SERVO_LEDX_FLT, EASING_MODE_DEFAULT, SERVO_DEFUALT_CALC, 0);
  servo_init_pre(SERVO_LEDX_FLC, EASING_MODE_DEFAULT, SERVO_DEFUALT_CALC, 0);
  servo_init_pre(SERVO_LEDX_BRC, EASING_MODE_DEFAULT, SERVO_DEFUALT_CALC, 0);
  servo_init_pre(SERVO_LEDX_BRT, EASING_MODE_DEFAULT, SERVO_DEFUALT_CALC, 0);
  servo_init_pre(SERVO_LEDX_BLC, EASING_MODE_DEFAULT, SERVO_DEFUALT_CALC, 0);
  servo_init_pre(SERVO_LEDX_BLT, EASING_MODE_DEFAULT, SERVO_DEFUALT_CALC, 0);

  HAL_Delay(1000);
}

void servo_set_angle(uint8_t id, float angle)
{
  float delta = __fabs(angle - sl[id].last_angle);

  servo_init_pre(SERVO_LEDX_RFT, EASING_MODE_DEFAULT, SERVO_DEFUALT_CALC, 0);
  sl[id].angle.fCurr = angle;
  sl[id].last_angle = angle;

  elog_d(TAG, "servo %s set angle %f° delta %f°", get_id_string(id), angle, delta);

  pca9685_set_angle(id, angle);
  HAL_Delay( (uint32_t)(delta*5+0.5f) );
}

static void servo_easing_update()
{
  float curr_angle = 0;
  for (uint8_t i = 0; i < SERVO_ID_MAX; i++) {
    easing_update(&sl[i].angle);
    curr_angle = easing_get_float(&sl[i].angle);
    elog_v(TAG, "servo %s angle %f° last angle %f°", 
      get_id_string(i), curr_angle, sl[i].last_angle);
    if (sl[i].last_angle != curr_angle ) {
      pca9685_set_angle(i, curr_angle);
      sl[i].last_angle = curr_angle;
    }
  }
}

static void servo_easing_start_target(uint8_t id, float target, float step)
{
  float delta = __fabs(target - sl[id].last_angle);
  easing_set_frame_cnt(&sl[id].angle, delta/step);
  easing_start_target(&sl[id].angle, target);
  elog_i(TAG, "servo %s turn to %f°", get_id_string(id), target);
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
  /* USER CODE BEGIN 2 */

  hal_printf_init(&huart1, 115200);

  elog_init();
  /* set EasyLogger log format */
  elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
  elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_FUNC|ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
  /* start EasyLogger */
  elog_start();
  servo_init();

  servo_set_angle(SERVO_LEDX_RFT, 0);
  servo_set_angle(SERVO_LEDX_RFT, 90);
  servo_set_angle(SERVO_LEDX_RFT, 180);
  servo_set_angle(SERVO_LEDX_RFT, 90);
  servo_set_angle(SERVO_LEDX_RFT, 0);
  servo_set_angle(SERVO_LEDX_RFT, 90);
  servo_set_angle(SERVO_LEDX_RFT, 180);
  servo_set_angle(SERVO_LEDX_RFT, 90);
  servo_set_angle(SERVO_LEDX_RFT, 0);

  servo_easing_start_target(SERVO_LEDX_RFT, 90, 6);
  for(int i = 0;i<easing_get_frame_cnt(&sl[SERVO_LEDX_RFT].angle);i++){
    servo_easing_update();
  }

  servo_easing_start_target(SERVO_LEDX_RFT, 180, 6);
  for(int i = 0;i<easing_get_frame_cnt(&sl[SERVO_LEDX_RFT].angle);i++){
    servo_easing_update();
  }

  servo_easing_start_target(SERVO_LEDX_RFT, 0, 6);
  for(int i = 0;i<easing_get_frame_cnt(&sl[SERVO_LEDX_RFT].angle);i++){
    servo_easing_update();
  }
  

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
