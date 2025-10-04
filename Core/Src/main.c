/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"

 
 
#include "tern.h"
#include "linearRegression.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sg90.h"
#include "ms200.h"
#include "motor.h"
#include "oled.h"
#include "spcp.h"
#include "pid.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * 变量和宏定义区
 * 下面是一些用于控制小车行为的参数和常量。
 * 每个宏后面都加了注释，方便理解其作用。
 */
#define NEAR_NUM 10 // 近距离判断的点数
#define DEFAULT_WARRING_DISTANCE 200 // 如果测距小于200mm，倒车
#define DEFAULT_SPEED 60 // 连续拐弯时的速度
#define DEFAULT_LOW_SPEED 60 // 转弯时的速度
#define DEFAULT_HIGH_SPEED 80 // 直道时的速度
#define DEFAULT_HIGH_DISTANCE 2000 // 最大速度对应的前方距离
#define DEFAULT_BACK_ANGLE 0 // 默认倒车转角
#define DEFAULT_AFTER_BACK_ANGLE 30 // 默认倒车后偏离的角度
#define DEFAULT_BACK_SPEED 75 // 默认倒车速度
#define DEFAULT_BACK_HIGH_SPEED 100 // 倒车时的高速
#define DEFAULT_BACK_TIMER 1000 // 默认倒退时间
#define DEFAULT_BACK_DISTANCE 450 // 默认车后距离不小于DEFAULT_BACK_DISTANCE
#define DEFAULT_STOP_SPEED -100 // 紧急止动速度
#define DEFAULT_RR_COEFFICIENT 0.5 // 右转系数
#define FRONT_JUDGE_POINTS 20 // 前方转角距离判断个数
#define CAR_LONG 40 // 小车长度，单位mm
#define CAR_WIDE 25 // 小车宽度，单位mm
#define MS200_DISTANCE_FRONT 5 // 激光雷达到车前的距离
#define MS200_DISTANCE_REAR 35 // 激光雷达到车后的距离
#define MS200_DISTANCE_SIDE 12 // 激光雷达到车侧的距离



/* -------------- 停车判断可调宏 --------------- */

// #define STOP_THR 20// 停车阈值，单位 mm
// #define CACHE_FRM 6//取点间隔
// #define point_number 5//取点数


#define STOP_62 132.0f
#define STOP_160 353.0f
#define STOP_203 350.0f
#define STOP_300 151.0f
#define back_time 20   //倒车时间戳
#define stop_woring 14.0f
#define start_time 120 //启动的时间戳，单位为循环的次数，实际调试时候去改变
/* -------------------------------------- */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 定义处理雷达返回数据点的数量
#define SIZE 9 
// 定义一个结构体来存储数据点的坐标
/*
 * DataPoint结构体
 * 用于存储激光雷达返回的点的坐标（x, y）
 */
typedef struct {
    double xDistance; // x方向距离
    double yDistance; // y方向距离
} DataPoint;
//定义一个DataPoint数组来存雷达返回数据点据点的坐标

/*
 * LidarRightpoints/LidarLeftpoints
 * 存储右侧/左侧激光雷达点的坐标
 */
DataPoint LidarRightpoints[SIZE + 1];
DataPoint LidarLeftpoints[SIZE + 1];
DataPoint Last_LidarRightpoints[SIZE + 1];
DataPoint Last_LidarLeftpoints[SIZE + 1];
MS200_Point* points;
// 定义线性回归的参数（斜率a和截距b）
/*
 * 线性回归参数
 * a为斜率，b为截距
 */
 double Right_a=0.0f; 
 double Right_b=0.0f;

 double Left_a=0.0f; 
 double Left_b=0.0f;
 /*
 * 不同场景下的转角PID参数
 * angle_pid_1: 直道
 * angle_pid_2: 普通弯道
 * angle_pid_3: 急弯
 */
static float angle_pid_1[3] = { 0.2 , 0.1};
static float angle_pid_2[3] = { 0.4 , 0.2};
static float angle_pid_3[3] = { 0.6 , 0.2};
// 定义存储从20degree开始逐次增加5，至60degree的正余弦值
/*
 * 预先计算好的正弦和余弦值数组
 * 用于将极坐标转换为直角坐标
 */
static double sin_values[9] = {  
    0.3420201433, 0.4226182617, 0.5000000000,  
    0.5735764364, 0.6427876097, 0.7071067812, 
    0.7660444431, 0.8191520443, 0.8660254038
};
static double cos_values[9] = {  
    0.9396926208, 0.9063077870, 0.8660254038,  
    0.8191520443, 0.7660444431, 0.7071067812,
    0.6427876097, 0.5735764364, 0.5000000000
};
static float Kp, Kd;
/*
 * UART接收完成回调函数
 * 主要用于处理激光雷达数据
 * huart: 串口句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5)
    {
        MS200_ReceiveProcess();
    }
}
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/*
 * 激光雷达数据丢失点处理函数
 * i: 当前点的索引
 * points: 激光雷达点数组
 * currentData: 当前点的距离
 * lastData: 上一个有效点的距离
 */
void judge_data_abondan(int i , MS200_Point* points , float *currentData , float *lastData){
    *currentData = points[i].distance / 10;
    if(*currentData != 0){
        *lastData = *currentData;
    }
		else{
        if(*lastData != 0)
            *currentData = *lastData;
        else
            *currentData = 1;
    }
}

#define WINDOW_SIZE 7       // 窗口大小（建议奇数，对称窗口）
#define MAX_JUMP_DISTANCE 0.5f  // 使用float常量（f后缀）
//可以进一步优化，使用其他更为优化的算法
void robustSlidingWindowFilter(DataPoint* points, int size) {
    int i, j;
    for (i = WINDOW_SIZE/2; i < size-WINDOW_SIZE/2; i++) {
        float sum_x = 0.0f, sum_y = 0.0f;  // 改用float节省资源
        int valid_count = 0;

        // 检查窗口内每个点是否合理
        for (j = -WINDOW_SIZE/2; j <= WINDOW_SIZE/2; j++) {
            float dist_prev = hypotf(points[i+j-1].xDistance, points[i+j-1].yDistance); // 使用float版本
            float dist_curr = hypotf(points[i+j].xDistance, points[i+j].yDistance);

            // 跳变检测
            if (fabsf(dist_curr - dist_prev) < MAX_JUMP_DISTANCE) { // 使用float版本
                sum_x += points[i+j].xDistance;
                sum_y += points[i+j].yDistance;
                valid_count++;
            }
        }

        // 有效点足够时更新（至少需要窗口一半的有效点）
        if (valid_count > WINDOW_SIZE/2) {  // > 而非 >=，更严格
            points[i].xDistance = sum_x / valid_count;
            points[i].yDistance = sum_y / valid_count;
        }
    }
}

/*
 * 曲率计算函数
 * 用于判断赛道的弯曲程度
 * points: 点数组
 * size: 点的数量
 * a, b: 拟合直线的参数
 * 返回值: 曲率
 */
float calculateCurvature2D(DataPoint* points, int size, double a, double b) {
    float curvature = 0;
    for (int i = 0; i < size; ++i) {
        float y_pred = a * points[i].xDistance + b;
        curvature += pow(points[i].yDistance - y_pred, 2);
    }
    return curvature / size;
}


/*
 * 曲线拟合函数（最小二乘法）
 * 对一组点进行线性拟合，得到斜率a和截距b
 * points: 点数组
 * size: 点的数量
 * a: 输出斜率
 * b: 输出截距
 */
//
void curveFitting2D(DataPoint* points, int size, double* a, double* b) {
    double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_xy = 0;
    for (int i = 0; i < size; ++i) {
			if(points[i].xDistance == 0){
				if(i == 0){
					points[i].xDistance = points[i + 1].xDistance;
				}
				
				else if(i == size - 1){
					points[i].xDistance = points[i - 1].xDistance;
				}
				else{
					points[i].xDistance = (points[i + 1].xDistance + points[i - 1].xDistance) / 2;
				}	
			}
			if(points[i].yDistance == 0){
				if(i == 0){
					points[i].yDistance = points[i + 1].yDistance;
				}
				
				else if(i == size - 1){
					points[i].yDistance = points[i - 1].yDistance;
				}
				else{
					points[i].yDistance = (points[i + 1].yDistance + points[i - 1].yDistance) / 2;
				}
			}
			
			
        sum_x += points[i].xDistance;
        sum_y += points[i].yDistance;
        sum_x2 += points[i].xDistance * points[i].xDistance;
        sum_xy += points[i].xDistance * points[i].yDistance;
    }
    double denominator = size * sum_x2 - sum_x * sum_x;
		if (fabs(denominator) < 1e-6) {
        *a = 0;
        *b = (size > 0) ? sum_y / size : 0;
        return;
    }
    *a = (size * sum_xy - sum_x * sum_y) / denominator;
    *b = (sum_y * sum_x2 - sum_x * sum_xy) / denominator;
}

static float curvatureLeft;
static float curvatureRight;
static int speed;
static int mode=1;
//pid选择函数
//根据曲率选择pid参数
void pid_select(){
    for(int i=0;i<SIZE;i++){
				LidarRightpoints[i].xDistance = points[20 + i * 5].distance  * cos_values[i];
				LidarRightpoints[i].yDistance = points[20 + i * 5].distance  * sin_values[i];
				//如果非零，记录下来
				if(LidarRightpoints[i].xDistance != 0){
					Last_LidarRightpoints[i].xDistance = LidarRightpoints[i].xDistance;
					//如果为零，设为上次记录
				}else{
					LidarRightpoints[i].xDistance = Last_LidarRightpoints[i].xDistance;
				}
				
				if(LidarRightpoints[i].yDistance != 0){
					Last_LidarRightpoints[i].yDistance = LidarRightpoints[i].yDistance;
				}else{
					LidarRightpoints[i].yDistance = Last_LidarRightpoints[i].yDistance;
				}
				
				LidarLeftpoints[i].xDistance = points[340 - i * 5].distance  * cos_values[i];
				LidarLeftpoints[i].yDistance = points[340 - i * 5].distance  * sin_values[i];
				
				if(LidarLeftpoints[i].xDistance != 0){
					Last_LidarLeftpoints[i].xDistance = LidarLeftpoints[i].xDistance;
				}else{
					LidarLeftpoints[i].xDistance = Last_LidarLeftpoints[i].xDistance;
				}
				
				if(LidarLeftpoints[i].yDistance != 0){
					Last_LidarLeftpoints[i].yDistance = LidarLeftpoints[i].yDistance;
				}else{
					LidarLeftpoints[i].yDistance = Last_LidarLeftpoints[i].yDistance;
				}
				
			}
			
			// 数据预处理
			robustSlidingWindowFilter( LidarRightpoints, SIZE);
			robustSlidingWindowFilter( LidarLeftpoints, SIZE);

			// 赛道轮廓提取
	 
			curveFitting2D(LidarRightpoints, SIZE, &Right_a, &Right_b);
			curveFitting2D(LidarLeftpoints, SIZE, &Left_a, &Left_b);


			// 赛道形状判断
			curvatureRight = calculateCurvature2D(LidarRightpoints, SIZE, Right_a, Right_b);
			curvatureLeft = calculateCurvature2D(LidarLeftpoints, SIZE, Left_a, Left_b);
			//根据曲率判断pid的选择
			if(curvatureRight > 5000.0 || curvatureLeft > 5000.0){
				Kp = angle_pid_2[0];
				Kd = angle_pid_2[1];
                speed = 60;
                mode=2;
			}else{
				Kp = angle_pid_1[0];
				Kd = angle_pid_1[1];
                speed = 70;
                mode=1;
			}
}


int diss[360];
double error_last = 0;
double error_last_reverse = 0;
int angle_0 = 0;
void Speed_Control(float speed){
	MOTOR_SetSpeed((int)speed);

}

void Servo_Control(float angle)
{
	angle_0 = angle;
	if (angle_0>30) angle_0=30;
	if (angle_0<-30) angle_0=-30;
	SG90_SetAngle((int)angle_0);
}

int avg_dis(int dis[], int n){
	int sum = 0;
	int count = 0;
	for (int i=0; i<n; ++i){
		if (dis[i]!=0){
			if (dis[i]<2000) sum += dis[i];
			else sum += 2000;
			++count;
		}
	}
	return sum / count;
}

void avoid_obstacle(){
	// 避免左右有障碍物的时候还往障碍物上撞
	int dis_min = 250; //临界距离，可以调整
	int dis_l = avg_dis(diss+110, 20);
	int dis_r = avg_dis(diss+232, 20);
	if (dis_l < dis_min && angle_0<0) Servo_Control(0);
	if (dis_r < dis_min && angle_0>0) Servo_Control(0);
}

void run(double kp, double kd, int max_dis)
{
	// lyh的idea 目前的最优算法 update at 2023.10.12
	// int dis_min_front = 9999; // 记录前方张角30度内的最短距离
	// // 计算前方张角30度内的最短距离
	// // for (int i=150; i<=210; ++i){
	// // 	if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	// // }
	// dis_min_front = avg_dis(diss+170,21);
	// // 如果前方有障碍物，那么小车减速
	// if (dis_min_front < 800 && dis_min_front >= 500) Speed_Control(60);
	// else if (dis_min_front < 500) Speed_Control(59);
	// else Speed_Control(70);
    Speed_Control(speed);
	// 计算前方赛道中心点的方位，使用pid逼近那个点
    int count = 0;
    int index1 = 0, dis1 = 0; // 临时变量
    int index2 = 0, dis2 = 0;
	double idx_1 = 0, idx_2 = 0; // 记录的最远的两个点的索引
	double dis_1 = 0, dis_2 = 0;

    // int idx_max = 0;
    double d_max = 0;

    for (int i = 90; i < 270; i++)
    {
        int dis = diss[i];
        if (dis > 0 && dis < max_dis)
        {
            count++;
            index1 = index2;
            dis1 = dis2;
            index2 = i;
            dis2 = dis;

            if (count > 1)
            {
                    // 此处的距离计算函数错误，但效果很好（？）
            double d = (double)(index2-index1) * (double)(dis1+dis2) / 2.0 + (double)((dis2-dis1)>0?(dis2-dis1):(dis1-dis2));
                    // 纠正回来了，但是效果一坨，不如上面的
                    // double d = (double)(index2-index1)* PI / 180 * (double)(dis1+dis2) / 2.0 + (double)((dis2-dis1)>0?(dis2-dis1):(dis1-dis2));
            if (d > d_max)
            {
                        idx_1 = index1;
                        idx_2 = index2;
                        dis_1 = dis1;
                        dis_2 = dis2;
                        d_max = d;
                    }
                }
        }
    }
    double target_angle;
	/*采用中线算法*/
	double oppsite_side = sqrt(pow(dis_1, 2) + pow(dis_2, 2) - 2 * dis_1 * dis_2 * cos((idx_2 - idx_1) * PI / 180));
	double mid_line = sqrt((pow(dis_1, 2)+pow(dis_2, 2))/2-pow(oppsite_side, 2)/4);
	double offset_angle = acos((pow(dis_1, 2)+pow(mid_line, 2)-pow(oppsite_side/2, 2))/(2*dis_1*mid_line));
	target_angle = idx_1 + offset_angle * 180 / PI;
  double error_mid = target_angle - 180;
	// /*采用比例算法*/

	double lambda = 0.6;
	// if (diss[(int)idx_1]-diss[(int)idx_2] > 100) lambda = 0.6;
	// else lambda = 0.4;
	if (avg_dis(diss+175,10)>1100) lambda = 0.3333;
	else if (avg_dis(diss+175,10)>900) lambda = 0.4;
	else if (avg_dis(diss+175,10)>700) lambda = 0.5;
	else lambda = 0.6;
	// lambda = 0.5; // 先全部调成0.5 复刻第一轮
	double dot_ab = dis_1 * dis_2 * cos((idx_2 - idx_1) * PI / 180);
	double m = sqrt(pow(1-lambda,2)*pow(dis_1,2)+pow(lambda,2)*pow(dis_2,2)+2*(1-lambda)*lambda*dot_ab);
	double theta = acos(((1-lambda)*pow(dis_1,2)+lambda*dot_ab)/(dis_1*m))*180/PI;
	double target_angle_ratio = idx_1 + theta;
  double error_ratio = target_angle_ratio - 180;

	/*采用垂线算法*/ 
	//答辩
	// double dot_ab = dis_1 * dis_2 * cos((idx_2 - idx_1) * PI / 180);
	// double t = (pow(dis_2,2)-dot_ab)/(pow(dis_1,2)-2*dot_ab);
	// double theta = acos((t*pow(dis_1,2)+dot_ab)/(dis_1*sqrt(pow(t*dis_1,2)+pow(dis_2,2)+2*t*dot_ab)))*180/PI;
	// double target_angle = idx_1 + theta;

    double w_mid, w_ratio;
    
    double error_diff = fabs(error_mid - error_ratio);
    if (error_diff > 45) {
        // 算法差异很大，保守策略：主要依赖中线算法
        w_mid = 0.7;
        w_ratio = 0.3;
    }
    else if (error_diff > 25) {
        // 中等差异，均衡分配
        w_mid = 0.5;
        w_ratio = 0.5;
    }
    else {
        // 算法一致，充分利用所有信息
        w_mid = 0.4;
        w_ratio = 0.6;
    }

//-------------------- 最终误差融合--------------------------------
    double final_error = w_mid * error_mid + w_ratio * error_ratio;


  // double error = target_angle - 180;  // 逆时针转的时候error一般小于0，顺时针大于0
      Servo_Control(kp * final_error + kd * (final_error - error_last));
      avoid_obstacle();
    error_last = final_error;
  
}

/*---------------------------xiaohei---------------------*/
/*
*判断小车是否停止函数
*/
// int stop_points[point_number] = {0};   //通过时间戳，间隔取5次点

// int car_stop_judgment(MS200_Point* points,int car_time,int angle){

//     int floor = -1;
//     float reduction_number = 1000.0f;

//     /* 采样 */
//     if (points[angle].distance < 200.0f && car_time > start_time && (car_time - start_time) % CACHE_FRM == 0){

//         floor = ((car_time - start_time) % (CACHE_FRM * point_number)) / CACHE_FRM;  
//         stop_points[floor] = points[angle].distance;

//     }

//     /* 只在收满第 point_number-1 层时做停稳判决 */
//     if (floor == point_number - 1){
//         for(int i = 0;i < point_number - 1;++i){

//         //计算差值和
//             reduction_number += fabs((float)(stop_points[i] - stop_points[i + 1]));

//         }
//         reduction_number /= (float)(point_number - 1);

//     }

//     if(reduction_number <= STOP_THR){
//         return 1;
//     }

//     return 0;

// }

// #define STOP_62 142.0f
// #define STOP_160 363.0f
// #define STOP_203 360.0f
// #define STOP_300 161.0f

int car_stop_judgment(MS200_Point* points,int car_time){

    if(car_time < start_time) 
        return 3;
    if(points[62].distance < (STOP_62 + stop_woring))
        return 1;
    if(points[160].distance < (STOP_160 + stop_woring))
        return 2;
    if(points[203].distance < (STOP_203 + stop_woring))
        return -2;
    if(points[300].distance < (STOP_300 + stop_woring))
        return -1;
    
    return 0;

}


/*
* 判断倒车何时进行
*/
void run2(int stop,int car_time,int *back_run_time){

    
    if(stop == 0 || stop == 2 || stop == -2 ){
        run(Kp, Kd, 750);
    }
    
    if(stop == 1 && car_time > *back_run_time)
        *back_run_time = car_time + back_time;

    if(stop == -1 && car_time > *back_run_time)
        *back_run_time = car_time + back_time;

    if(car_time < *back_run_time){
        Speed_Control(-100);

        //右前轮撞墙
        if(stop == 1){
            Servo_Control(30);
        }

        //左前轮撞墙
        if(stop == -1){
            Servo_Control(-30);
        }
    }
    
}



/*--------------------------xiahei-----------------------*/
/*
 * 主函数
 * 程序入口，负责初始化硬件、主循环逻辑等
 * 返回值: 0
 */
int main(void){

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	SG90_Init();
	MS200_Init();
	MOTOR_Init();
	OLED_Init();

	SG90_SetAngle(0);
	
    int car_time = 0;
	while(1){
    OLED_Clear();
    points = MS200_GetPointsData();
	pid_select();
    static int remapped_diss[360]; 

    // 在一个循环内，直接从 points 读取，并存放到旋转后的正确位置
    for (int i = 0; i < 360; i++) {
    // 读取原始数据 points[i] (0度是前方)
    // 直接存入 remapped_diss 的目标位置 (180度是前方)
      remapped_diss[(i + 180) % 360] = (int)points[i].distance;
    }

    // 最后一步仍然需要 memcpy，将排好序的数据复制回全局的 diss 数组
    memcpy(diss, remapped_diss, sizeof(diss));
  //         //坐标变换修正 ---
  //   #define RADAR_OFFSET_L 130.0f // 假设雷达前置了80毫米，这个值你需要自己精确测量！
  //   // 将修正后的数组声明为 static，可以避免反复在栈上创建，更安全稳定
  //   static int corrected_diss[360]; 

  //   for (int i = 0; i < 360; i++) 
  //   {
  //   // 如果原始距离为0或无效，修正后也为0
  //     if (diss[i] <= 0) {
  //     corrected_diss[i] = 0;
  //     continue;
  //   }

  //   // 1. 将雷达数据转换到标准的数学坐标系
  //   //    小车前方为0度(X轴正方向)，逆时针为正
  //   //    雷达数据中180度是前方，所以需要减180度
  //   float angle_rad = (float)(i - 180) * PI / 180.0f;
  //   float d = (float)diss[i];

  //   // 2. 计算雷达探测点相对于小车旋转中心的直角坐标
  //   //    雷达在旋转中心前方 L 处，所以雷达的坐标是 (L, 0)
  //   //    探测点的坐标 = 雷达坐标 + d在各个方向上的分量
  //   float point_x = RADAR_OFFSET_L + d * cosf(angle_rad);
  //   float point_y = d * sinf(angle_rad);

  //   // 3. 计算该点相对于小车旋转中心的真实距离
  //   float new_dist = sqrtf(point_x * point_x + point_y * point_y);
                
  //   // 4. 【安全检查】检查计算结果是否是无效数字 (NaN)
  //   //    如果发生无效计算，就不要使用这个值，将其置为0
  //   if (isnan(new_dist)) {
  //     corrected_diss[i] = 0;
  //   } else {
  //     corrected_diss[i] = (int)new_dist;
  //   }
  // }

  // // 5. 使用修正后的数据覆盖原始数据，供 run() 函数使用
  // memcpy(diss, corrected_diss, sizeof(diss));
      
			
	car_time++;
    int stop = car_stop_judgment(points,car_time);//停车判断，1停车
    int back_run_time = 0;

    OLED_Clear();		
	OLED_ShowFloat(0,0,(double)back_run_time);
	OLED_ShowFloat(60,0,points[62].distance);
	OLED_ShowFloat(0,32,0.0f);
	OLED_ShowFloat(60,32,points[300].distance);
	OLED_ShowFloat(0,48, (double)stop);
	OLED_ShowFloat(60,48,(double)car_time);
	OLED_Display();

    run2(stop,car_time,&back_run_time);
			/* USER CODE END WHILE */
			
			/* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

