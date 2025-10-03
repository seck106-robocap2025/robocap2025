#include "command.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "usart.h"
#include "motor.h"
#include "sg90.h"

uint8_t command_recv_buffer[COMMAND_RECV_BUFFER_SIZE] = {0};
int command_recv_buffer_length = 0;
uint8_t command_recv_byte = 0;

void COMMAND_Init(void){
	HAL_UART_Receive_IT(&huart2,&command_recv_byte,1);
}

void COMMAND_Response(uint8_t code,uint8_t cmd){
	uint8_t buffer[16];
	buffer[0] = code;
	buffer[1] = cmd;
	buffer[2] = 0xCC;
	buffer[3] = 0xCC;
	HAL_UART_Transmit_IT(&huart2,buffer,4);
}

void COMMAND_ReceiveProcess(){
	uint8_t offset = 0;
	uint8_t first_find_flag = 0;
	command_recv_buffer[command_recv_buffer_length++] = command_recv_byte;
	
	if(command_recv_buffer_length >= 3 && command_recv_buffer[command_recv_buffer_length-1] == 0xCC && command_recv_buffer[command_recv_buffer_length-2] == 0xCC){
		switch(command_recv_buffer[COMMAND_OFFSET_CMD]){
			case COMMAND_CMD_STOP:
			{
				MOTOR_SetSpeed(0);
				COMMAND_Response(0,COMMAND_CMD_STOP);
			}
			break;
			case COMMAND_CMD_SET_SPEED:
			{
				int speed = command_recv_buffer[COMMAND_OFFSET_SPEED];
				if(command_recv_buffer[COMMAND_OFFSET_DIRECTION] == COMMAND_DIRECTION_BACK){
					speed = -speed;
				}
				MOTOR_SetSpeed(speed);
				COMMAND_Response(0,COMMAND_CMD_SET_SPEED);
			}
			break;
			case COMMAND_CMD_SET_ANGLE:
			{
				int angle = command_recv_buffer[COMMAND_OFFSET_ANGLE];
				if(command_recv_buffer[COMMAND_OFFSET_DIRECTION] == COMMAND_DIRECTION_LEFT){
					angle = -angle;
				}
				SG90_SetAngle(angle);
				COMMAND_Response(0,COMMAND_CMD_SET_ANGLE);
			}
			break;
		}
		
		memset(command_recv_buffer,0,COMMAND_RECV_BUFFER_SIZE);
		command_recv_buffer_length = 0;
	}
	HAL_UART_Receive_IT(&huart2,&command_recv_byte,1);

}



