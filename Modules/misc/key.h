#pragma once
#include "stm32f4xx_hal.h"

#define KEY_IRQ_MODE              1

#define KEY_X_PROT                GPIOF
#define KEY_RCC_IS_CLK_DISABLED() __HAL_RCC_GPIOF_IS_CLK_DISABLED()
#define KEY_RCC_CLK_ENABLE()      __HAL_RCC_GPIOF_CLK_ENABLE()

#define KEY_2_PIN                 GPIO_PIN_5
#define KEY_3_PIN                 GPIO_PIN_6
#define KEY_4_PIN                 GPIO_PIN_7
#define KEY_5_PIN                 GPIO_PIN_8
#define KEY_6_PIN                 GPIO_PIN_9

#define KEY_UP_PIN                KEY_6_PIN
#define KEY_DOWN_PIN              KEY_4_PIN
#define KEY_LEFT_PIN              KEY_2_PIN
#define KEY_RIGHT_PIN             KEY_5_PIN
#define KEY_CENTER_PIN            KEY_3_PIN

typedef enum
{
  KEY_UNKNOWN = -1,
  KEY_UP = 0,
  KEY_DOWN,
  KEY_LEFT,
  KEY_RIGHT,
  KEY_CENTER,
  KEY_MAX
} key_value_t;

void key_init(void);
key_value_t key_read(void);
