#include "stm32_hal_hw_pwm_beep.h"

static TIM_HandleTypeDef* htimx = NULL;
static uint32_t channelx        = 0;
static float volume             = 50;

#define delay_ms(ms) HAL_Delay(ms)

/**
 * @brief  float转int四舍五入
 * @note   直接(int)强制转换会直接舍去小数部分
 */
static int ftoi(float f)
{
	return (int)(f+0.5f);
}

//! 先去网上搜一下对应音符的频率，然后再去计算对应的ARR值
//! 音符频率表
static uint16_t note_freq_lsit[7][6]={
	/*		低音,低半,中音,中半,高音,高半		*/
	/*DO*/{262,277,523,554,1046,1109},
	/*RE*/{294,311,578,622,1175,1245},			
	/*MI*/{330,NULL,659,NULL,1318,NULL},		
	/*FA*/{349,NULL,698,740,1397,1480},			
	/*SO*/{392,415,784,831,1568,1661},			
	/*LA*/{440,466,880,932,1760,1865},			
	/*SI*/{494,NULL,988,NULL,1976,NULL}	
};

void beep_init(TIM_HandleTypeDef* htim,uint32_t channel,float sysClock)
{
	htimx = htim;
	channelx = channel;
  __HAL_TIM_SET_PRESCALER(htimx, ftoi((sysClock/10000.f))-1);
  __HAL_TIM_SET_AUTORELOAD(htimx, 10000-1);
}

void beep_set_volume(float v)
{
	volume = v;
}

void beep_set_note(uint16_t note, uint16_t tone)
{
	float farr = 10000.f/note_freq_lsit[note-1][tone-1];
	float fccr = farr*(volume/100.0f);
	__HAL_TIM_SET_AUTORELOAD(htimx,ftoi(farr));//调整PWM周期
	__HAL_TIM_SET_COMPARE(htimx,channelx,ftoi(fccr));//调整PWM占空比(音量)
}

void beep_start()
{
	HAL_TIM_PWM_Start(htimx,channelx);//开启PWM
}

void beep_stop()
{
	HAL_TIM_PWM_Stop(htimx,channelx);//关闭PWM
	htimx->Instance->CNT=0;//复位PWM周期
}

void beep_play_note(uint16_t note, uint16_t tone, uint16_t ms)
{
	if(note!=0 && tone!=0 && ms!=0)
	{
		beep_set_note(note,tone);
		beep_start();
	}
	delay_ms(ms);
	beep_stop();
}

static bool musicLoop          = false;
static uint16_t musicIndex     = 0;
static beep_note_t* musicScore = NULL;
static uint16_t musicLen       = 0;
static uint32_t musicTick			 = 0;

void beep_set_musicScore(beep_note_t* music, uint16_t len)
{
	musicLoop  = true;
	musicIndex = 0;
	musicScore = music;
	musicLen   = len;	
}

int beep_paly_musicScore()
{
	if(musicLoop)
	{
		if( musicIndex==0 )
		{
			beep_start();
			beep_set_note(musicScore[musicIndex].note,musicScore[musicIndex].tone);
			musicIndex++;
			musicTick=HAL_GetTick();
			return 1;
		}
		else if(musicIndex==musicLen)
		{
			if((HAL_GetTick()-musicTick)>musicScore[musicIndex-1].ms)
			{
				beep_stop();
				musicLoop = false;
			}
			return 0;
		}
		else
		{
			if((HAL_GetTick()-musicTick)>musicScore[musicIndex-1].ms)
			{
				beep_set_note(musicScore[musicIndex].note,musicScore[musicIndex].tone);
				musicIndex++;
				musicTick=HAL_GetTick();
			}
			return 1;
		}
	}
	else
	{
		return 0;
	}
}

void beep_clear_musicScore()
{
	musicLoop = false;
}