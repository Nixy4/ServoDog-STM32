#include "key.h"

void key_init()
{
#if KEY_IRQ_MODE
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if( KEY_RCC_IS_CLK_DISABLED() ) {
    KEY_RCC_CLK_ENABLE();
  }
  GPIO_InitStruct.Pin = KEY_2_PIN|KEY_3_PIN|KEY_4_PIN|KEY_5_PIN|KEY_6_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_X_PROT, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#else
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if( KEY_RCC_IS_CLK_DISABLED() ) {
    KEY_RCC_CLK_ENABLE();
  }
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
#endif
}

#if KEY_IRQ_MODE
static volatile key_value_t key_value = KEY_UNKNOWN;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_UP_PIN) {
    key_value = KEY_UP;
  } else if(GPIO_Pin == KEY_DOWN_PIN) {
    key_value = KEY_DOWN;
  } else if(GPIO_Pin == KEY_LEFT_PIN) {
    key_value = KEY_LEFT;
  } else if(GPIO_Pin == KEY_RIGHT_PIN) {
    key_value = KEY_RIGHT;
  } else if(GPIO_Pin == KEY_CENTER_PIN) {
    key_value = KEY_CENTER;
  } else {
    key_value = KEY_UNKNOWN;
  }
}
#endif

key_value_t key_read()
{
#if KEY_IRQ_MODE
  return key_value;
#else
  if(HAL_GPIO_ReadPin(KEY_X_PROT, KEY_UP_PIN) == GPIO_PIN_RESET) {
    return KEY_UP;
  } else if(HAL_GPIO_ReadPin(KEY_X_PROT, KEY_DOWN_PIN) == GPIO_PIN_RESET) {
    return KEY_DOWN;
  } else if(HAL_GPIO_ReadPin(KEY_X_PROT, KEY_LEFT_PIN) == GPIO_PIN_RESET) {
    return KEY_LEFT;
  } else if(HAL_GPIO_ReadPin(KEY_X_PROT, KEY_RIGHT_PIN) == GPIO_PIN_RESET) {
    return KEY_RIGHT;
  } else if(HAL_GPIO_ReadPin(KEY_X_PROT, KEY_CENTER_PIN) == GPIO_PIN_RESET) {
    return KEY_CENTER;
  } else {
    return KEY_UNKNOWN;
  }
#endif
}
