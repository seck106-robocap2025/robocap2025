#ifndef __MONTO_H__
#define __MONTO_H__

#include "gpio.h"
#include "tim.h"

#define ENCODER_TIM 					htim5 
#define ENCODER_TIM_CHANNEL1 	TIM_CHANNEL_1
#define ENCODER_TIM_CHANNEL2 	TIM_CHANNEL_2

#define PWM_TIM     			htim1
#define PWM_TIM_CHANNEL1 	TIM_CHANNEL_3
#define PWM_TIM_CHANNEL2 	TIM_CHANNEL_4

#define GAP_TIM     htim3

#define MOTOR_SPEED_RERATIO 45u  //电机减速比
#define PULSE_PRE_ROUND 11 //一圈多少个脉冲
#define RADIUS_OF_TYRE 82 //轮胎半径，单位毫米
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14
#define RELOADVALUE __HAL_TIM_GetAutoreload(&ENCODER_TIM)
#define COUNTERNUM __HAL_TIM_GetCounter(&ENCODER_TIM)

typedef struct _Motor
{
    int32_t lastCount;		//​记录上一次编码器的计数值，用于计算两次定时器中断之间的计数差，从而得到速度
    int32_t totalCount;		//当前总计数值由当前编码器计数值(COUNTERNUM)加上溢出计数(overflowNum * RELOADVALUE)计算得到
    int16_t overflowNum;	//​编码器溢出计数当编码器计数超过上限时会触发溢出，这个变量记录溢出次数
    float speed;					//​计算得到的电机速度（mm）
    uint8_t direct;				//​转动方向（0表示正向，1表示反向）
}Motor;

void MOTOR_Init(void);
Motor* MOTOR_GetStatus(void);
void MOTOR_SetSpeed(int present);



#endif




