#include "aeabi.h"
#include "stm32f4xx_hal.h"
#include "elog.h"
#include "pca9685.h"

#define I2C_ALLCALL_ADDR      (0xE0 >> 1)

#define REG_MODE1             0x00
#define REG_MODE1_ALLCALL_BIT 0x01 //使能PCA的广播地址响应
#define REG_MODE1_SUB3_BIT    0x02 //使能PCA的子地址3响应
#define REG_MODE1_SUB2_BIT    0x04 //使能PCA的子地址2响应
#define REG_MODE1_SUB1_BIT    0x08 //使能PCA的子地址1响应
#define REG_MODE1_SLEEP_BIT   0x10 //睡眠模式
#define REG_MODE1_AUTOINC_BIT 0x20 //自动增加地址
#define REG_MODE1_EXTCLK_BIT  0x40 //外部时钟
#define REG_MODE1_RESTART_BIT 0x80 //重启功能

#define REG_MODE2             0x01
#define REG_MODE2_INVRT_BIT   0x10 //反转输出
#define REG_MODE2_OCH_BIT     0x08 //输出驱动
#define REG_MODE2_OUTDRV_BIT  0x04 //输出驱动
#define REG_MODE2_OUTNE1_BIT  0x02 //输出驱动
#define REG_MODE2_OUTNE0_BIT  0x01 //输出驱动

#define REG_LED_BASE          0x06                        // LED0~LED15 0x06~0x45
#define REG_LEDX_BASE(x)      ( REG_LED_BASE + 4U * x )
#define REG_LEDX_ON_L(x)      ( REG_LEDX_BASE(x) + 1U )
#define REG_LEDX_ON_H(x)      ( REG_LEDX_BASE(x) + 2U )
#define REG_LEDX_OFF_L(x)     ( REG_LEDX_BASE(x) + 3U )
#define REG_LEDX_OFF_H(x)     ( REG_LEDX_BASE(x) + 4U )

#define REG_PSC               0xFE

#define IS_LEDX(x)            ( (x) < 16U ) // >> bool

#define ONL(x)                ( ((uint16_t)(x) & 0xFFU) ) // 0x00FF
#define ONH(x)                ( ((uint16_t)(x) >> 8) )
#define OFFL(x)               ( ((uint16_t)(x) & 0xFFU) )
#define OFFH(x)               ( ((uint16_t)(x) >> 8) )

#define ICLK                  25000000U // PCA内部 25MHz
#define CNT_MAX               4096U

extern I2C_HandleTypeDef hi2c2;

static const char* TAG = "PCA9685";

static void writereg(uint8_t reg, uint8_t val)
{
  HAL_StatusTypeDef status;
  uint8_t buf[2] = {reg, val};
  status = HAL_I2C_Master_Transmit(&hi2c2, PCA9685_I2C_ADDR, buf, 2, HAL_I2C_TRANSFER_TIMEOUT);
  if (status != HAL_OK) {
    elog_e(TAG, "PCA9685 write reg failed, reg: 0x%02X, val: 0x%02X", reg, val);
  }
  elog_v(TAG, "write reg [%02X] = %02X", reg, val);
}

static uint8_t readreg(uint8_t reg)
{
  HAL_StatusTypeDef status;
  uint8_t val = 0;
  status = HAL_I2C_Master_Transmit(&hi2c2, PCA9685_I2C_ADDR, &reg, 1, HAL_I2C_TRANSFER_TIMEOUT);
  if (status != HAL_OK) {
    elog_e(TAG, "PCA9685 read reg failed, reg: 0x%02X", reg);
  }
  status = HAL_I2C_Master_Receive(&hi2c2, PCA9685_I2C_ADDR, &val, 1, HAL_I2C_TRANSFER_TIMEOUT);
  if (status != HAL_OK) {
    elog_e(TAG, "PCA9685 read reg failed, reg: 0x%02X", reg);
  }
  elog_v(TAG, "read reg [%02X] = %02X", reg, val);
  return val;
}

void pca9685_set_psc(uint8_t psc)
{
  uint8_t old, new;

  //使能PCA的广播地址响应, 用于同时控制多个PCA9685
  new = 0x00;
  SET_BIT(new, REG_MODE1_ALLCALL_BIT);
  writereg(REG_MODE1, new);

  //读取旧的模式值
  old = readreg(REG_MODE1); 
  new = old; 

  //设置新的模式值: 1.禁用重启  2.启用睡眠模式
  CLEAR_BIT(new, REG_MODE1_RESTART_BIT); //禁用重启功能
  SET_BIT(new, REG_MODE1_SLEEP_BIT);     //启用睡眠模式
  writereg(REG_MODE1, new);

  //设置预分频寄存器
  writereg(REG_PSC, psc);//设置预分频寄存器
  //恢复旧的模式值
  writereg(REG_MODE1, old);
  HAL_Delay(5);

  //设置新的模式值: 1.重启  2.启用自动增加地址  3.启用广播地址响应
  SET_BIT(old, REG_MODE1_RESTART_BIT|REG_MODE1_AUTOINC_BIT|REG_MODE1_ALLCALL_BIT);
  writereg(REG_MODE1, old);
}

void pca9685_set_freq(QUAD_TYPE freq)
{
  freq *= 0.98f; // 频率补偿, 使得实际频率接近设定频率
  // 50Hz * 0.98 = 49Hz ; 25000000Hz / 49Hz ≈ 510204 tick/s; 510204 tick/s / 4096 tick ≈ 124.5 tick/s
  // 50Hz * 0.92 = 46Hz ; 25000000Hz / 46Hz ≈ 543478 tick/s; 543478 tick/s / 4096 tick ≈ 132.8 tick/s
  // 周期1 = 1 / (124.5-1) ≈ 8.09ms;
  // 周期2 = 1 / (132.8-1) ≈ 7.59ms;
  uint8_t psc = ( (uint8_t)( (QUAD_TYPE)(ICLK) / ( (QUAD_TYPE)CNT_MAX * freq ) ) ) - 1 ;
  pca9685_set_psc(psc);
}

void pca9685_set_pwm(uint8_t ledx, uint16_t on, uint16_t off)
{
  if (!IS_LEDX(ledx)) {
    elog_e(TAG, "PCA9685 set pwm failed, ledx: %d", ledx);
    return;
  }
  uint8_t buf[5] = {REG_LEDX_BASE(ledx),ONL(on),ONH(on),OFFL(off),OFFH(off)};
  HAL_I2C_Master_Transmit(&hi2c2, PCA9685_I2C_ADDR, buf, 5, HAL_I2C_TRANSFER_TIMEOUT);
}

void pca9685_set_angle(uint8_t ledx, QUAD_TYPE angle)
{
  if (!IS_LEDX(ledx)) {
    elog_e(TAG, "PCA9685 set angle failed, ledx: %d", ledx);
    return;
  }
  uint32_t off = 0;
  off = __aeabi_d2ulz(angle * 2.276 + 0.5) + 102;
  pca9685_set_pwm(ledx, 0, off);
}