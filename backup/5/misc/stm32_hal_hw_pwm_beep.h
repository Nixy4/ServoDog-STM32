#ifndef __STM32_BEEP__
#define __STM32_BEEP__

#include "stdbool.h"
#include "stm32f4xx_hal.h"

typedef struct
{
  uint16_t note;
  uint16_t tone;
  uint16_t ms;
} beep_note_t;

/**
 * @brief  初始化蜂鸣器
 * @param  htim: HAL库定时器句柄
 * @param  channel: 定时器通道
 * @param  sysClock: 系统时钟频率
 */
void beep_init(TIM_HandleTypeDef* htim,uint32_t channel,float sysClock);

/**
 * @brief  设置音量
 * @param  vol: 音量 0-100
*/
void beep_set_volume(float vol);

/**
 * @brief  启动蜂鸣器
*/
void beep_start(void);

/**
 * @brief  停止蜂鸣器
*/
void beep_stop(void);

/**
 * @brief  播放音符
 * @param  note: 音符 1-7
 * @param  tone: 音调 1-6
 * @param  ms: 持续时间 ms
 * @note   音符频率表
 *        1     2     3     4     5     6
 *        低音  低半   中音  中半  高音   高半
 * 1  DO  262   277   523   554   1046  1109
 * 2	RE  294   311   578   622   1175  1245	
 * 3	MI  330   NULL  659   NULL  1318  NULL
 * 4	FA  349   NULL  698   740   1397  1480
 * 5	SO  392   415   784   831   1568  1661
 * 6	LA  440   466   880   932   1760  1865
 * 7	SI  494   NULL  988   NULL  1976  NULL
 */
void beep_play_note(uint16_t note, uint16_t tone, uint16_t ms);

/**
 * @brief 设置乐谱
 * @param _musicScore: 乐谱
 * @param _len: 乐谱长度
*/
void beep_set_musicScore(beep_note_t* _musicScore, uint16_t _len);

/**
 * @brief  设置乐谱
 * @retval 0: 播放完成
 * @retval 1: 还有需要播放的音符
 * @note 需要在循环中调用,内部自动计算索引和间隔时间,播放完成后就算调用也不再播放,需要重新设置乐谱
*/
int  beep_paly_musicScore(void);

/**
 * @brief  清空乐谱
 * @note   用于主动停止播放乐谱
*/
void beep_clear_musicScore(void);

#endif//! __STM32_BEEP__