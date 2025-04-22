#pragma once

#include "stdint.h"

#define PCA9685_LED0  0U
#define PCA9685_LED1  1U
#define PCA9685_LED2  2U
#define PCA9685_LED3  3U
#define PCA9685_LED4  4U
#define PCA9685_LED5  5U
#define PCA9685_LED6  6U
#define PCA9685_LED7  7U
#define PCA9685_LED8  8U
#define PCA9685_LED9  9U
#define PCA9685_LED10 10U
#define PCA9685_LED11 11U
#define PCA9685_LED12 12U
#define PCA9685_LED13 13U
#define PCA9685_LED14 14U
#define PCA9685_LED15 15U

#define PCA9685_I2C_ADDR         0x80
#define HAL_I2C_TRANSFER_TIMEOUT 100

void pca9685_set_freq(double freq);
void pca9685_set_pwm(uint8_t ledx, uint16_t on, uint16_t off);
void pca9685_set_angle(uint8_t ledx, double angle);