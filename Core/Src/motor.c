#include "motor.h"

Motor motor1;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调函数，用于计算速度
{
    if(htim == &ENCODER_TIM)//编码器输入定时器溢出中断，用于防溢出                   
    {      
        if(COUNTERNUM < 100000) motor1.overflowNum++;       //如果是向上溢出
        else if(COUNTERNUM >= 10000) motor1.overflowNum--; //如果是向下溢出
    	 __HAL_TIM_SetCounter(&ENCODER_TIM, 100000);             //重新设定初始值
				
				
    }
    else if(htim == &GAP_TIM)//间隔定时器中断，是时候计算速度了
    {
        motor1.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER_TIM); //如果向上计数（正转），返回值为0，否则返回值为1
        motor1.totalCount = COUNTERNUM + motor1.overflowNum * RELOADVALUE; //一个周期内的总计数值等于目前计数值加上溢出的计数值
        // motor1.speed = (float)(motor1.totalCount - motor1.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 10;//算得每秒多少转
        motor1.speed = -(float)(motor1.totalCount - motor1.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 10 * LINE_SPEED_C;//算得车轮线速度每秒多少毫米
        motor1.lastCount = motor1.totalCount; //记录这一次的计数值
			
    }
}

void MOTOR_Init(void){
	HAL_TIM_Encoder_Start(&ENCODER_TIM, ENCODER_TIM_CHANNEL1);      //开启编码器定时器
	HAL_TIM_Encoder_Start(&ENCODER_TIM, ENCODER_TIM_CHANNEL2);      //开启编码器定时器
	__HAL_TIM_ENABLE_IT(&ENCODER_TIM,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_SET_COUNTER(&ENCODER_TIM, 10000);                //编码器定时器初始值设定为10000
	
	HAL_TIM_PWM_Start(&PWM_TIM, PWM_TIM_CHANNEL1);
	HAL_TIM_PWM_Start(&PWM_TIM, PWM_TIM_CHANNEL2);
	
	HAL_TIM_Base_Start_IT(&GAP_TIM);                       //开启100ms定时器中断

	motor1.lastCount = 0;                                   //结构体内容初始化
	motor1.totalCount = 0;
	motor1.overflowNum = 0;                                  
	motor1.speed = 0;
	motor1.direct = 0;
}

Motor* MOTOR_GetStatus(void){
	return &motor1;
}


void MOTOR_SetSpeed(int present){
	present = -present;		//电机接反了，复数向后，正数向前，0电机停转。
	if(present >= 0){
		if(present > 100){
			present = 100;
		}
		__HAL_TIM_SET_COMPARE(&PWM_TIM,PWM_TIM_CHANNEL2,200*present); // 前进
		__HAL_TIM_SET_COMPARE(&PWM_TIM,PWM_TIM_CHANNEL1,0); // 后退
	}else{
		if(present < -100){
			present = -100;
		}
		__HAL_TIM_SET_COMPARE(&PWM_TIM,PWM_TIM_CHANNEL2,0); // 前进
		__HAL_TIM_SET_COMPARE(&PWM_TIM,PWM_TIM_CHANNEL1,-200*present); // 后退
	}
}







