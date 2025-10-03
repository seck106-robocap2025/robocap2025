#ifndef __PID_H__
#define __PID_H__

typedef struct
{
	float target_val;   //目标值（如目标速度、目标角度），是小车希望达到的设定值。
	float Error;          /*第 k 次偏差 */	//​当前偏差​​（目标值 - 实际值），反映当前状态与目标的差距。
	float LastError;     /* Error[-1],第 k-1 次偏差 */	//上一次的偏差​​（用于计算微分项），记录前一次控制的误差。
	float PrevError;    /* Error[-2],第 k-2 次偏差 */		//​​上上次的偏差​​（部分PID变体使用），用于更复杂的微分或积分计算。
	float Kp,Ki,Kd;     //比例、积分、微分系数		//​PID系数​​：比例、积分、微分项的权重，决定控制器对偏差的敏感程度。
	float integral;     //积分值		//积分累积值​​，累计历史偏差，消除静态误差（如小车长时间偏离赛道中心）。
	float output_val;   //输出值		//​控制器输出值​​，直接作用于执行器（如电机PWM占空比、舵机角度）。
}pid_handle,*pid_handle_t;


void pid_init(pid_handle_t ph);
float pi_realize(pid_handle_t ph, float actual_val);
float pid_realize(pid_handle_t ph, float actual_val);

#endif
