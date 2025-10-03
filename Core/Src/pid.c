#include "pid.h"

void pid_init(pid_handle_t ph)
{		
	ph->output_val=0.0;
	ph->Error=0.0;
	ph->LastError=0.0;
	ph->integral=0.0;
}

float pi_realize(pid_handle_t ph, float actual_val)
{
	/*计算目标值与实际值的误差*/
	ph->Error = ph->target_val - actual_val;
	/*积分项*/
	// ph->integral += ph->Error;
	ph->integral = ph->Error;
	/*PID算法实现*/
	ph->output_val = ph->Kp * ph->Error +
	                  ph->Ki * ph->integral;
	/*误差传递*/
	ph-> LastError = ph->Error;
	/*返回当前实际值*/
	return ph->output_val;
}


float pid_realize(pid_handle_t ph, float actual_val)
{
	/*计算目标值与实际值的误差*/
	ph->Error = ph->target_val - actual_val;
	/*积分项*/
	ph->integral =  ph->Error;

	/*PID算法实现*/
	ph->output_val = ph->Kp * ph->Error +
	                  ph->Ki * ph->integral +
	                  ph->Kd *(ph->Error -ph->LastError);
	/*误差传递*/
	ph-> LastError = ph->Error;
	/*返回当前实际值*/
	return ph->output_val;
}
