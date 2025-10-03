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

//------------------------------------------------------------------kxy---------------------------------------------------------------------
//-----DataPoint机构体，与曲率判断对应-----
//-----存储雷达返回的数据点的坐标----
//定义处理雷达返回数据点的数量

/*
右侧雷达扫描区域：20°, 25°, 30°, 35°, 40°, 45°, 50°, 55°, 60°
左侧雷达扫描区域：340°, 335°, 330°, 325°, 320°, 315°, 310°, 305°, 300°
*/

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

#define SIZE 9	
#define MOVING_AVG_SIZE 5
typedef struct {
    double xDistance;
    double yDistance;
} DataPoint;

DataPoint LidarRightpoints[SIZE], LidarLeftpoints[SIZE];
DataPoint Last_LidarRightpoints[SIZE], Last_LidarLeftpoints[SIZE];


//----------------添加数据有效性检查函数-----------------
// 数据有效性检查
int isValidPoint(DataPoint p) {
    return !(p.xDistance == 0 && p.yDistance == 0) &&
           !isnan(p.xDistance) && !isnan(p.yDistance) &&
           fabs(p.xDistance) < 10.0f && fabs(p.yDistance) < 10.0f;
}

// 数据插值
DataPoint interpolatePoint(DataPoint* points, int index, int size) {
    DataPoint result = {0, 0};
    int valid_neighbors = 0;
    
    // 向前找有效点
    for (int i = index+1; i < size && i <= index+2; i++) {
        if (isValidPoint(points[i])) {
            result.xDistance += points[i].xDistance;
            result.yDistance += points[i].yDistance;
            valid_neighbors++;
            break;
        }
    }
    
    // 向后找有效点
    for (int i = index-1; i >= 0 && i >= index-2; i--) {
        if (isValidPoint(points[i])) {
            result.xDistance += points[i].xDistance;
            result.yDistance += points[i].yDistance;
            valid_neighbors++;
            break;
        }
    }
    
    if (valid_neighbors > 0) {
        result.xDistance /= valid_neighbors;
        result.yDistance /= valid_neighbors;
    }
    
    return result;
}

//------------------------曲线拟合---------------------------------------------

 double Right_a=0.0f; 
 double Right_b=0.0f;
 double Left_a=0.0f; 
 double Left_b=0.0f;

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


 //曲率历史数据用于移动平均
float curvature_history_right[MOVING_AVG_SIZE] = {0};
float curvature_history_left[MOVING_AVG_SIZE] = {0};
int history_index = 0;

float leidaqv9zhi(){
//-----从雷达获取原始数据-----
    MS200_Point* points = MS200_GetPointsData();
    

    for(int i=0;i<SIZE;i++){
        // 右侧数据处理
        LidarRightpoints[i].xDistance = points[20 + i * 5].distance * cos_values[i];
        LidarRightpoints[i].yDistance = points[20 + i * 5].distance * sin_values[i];
        
        // ------进入无效点的修复------
        if (!isValidPoint(LidarRightpoints[i])) {
            if (isValidPoint(Last_LidarRightpoints[i])) {
                LidarRightpoints[i] = Last_LidarRightpoints[i];
            } else {
                LidarRightpoints[i] = interpolatePoint(LidarRightpoints, i, SIZE);
            }
        } else {
            Last_LidarRightpoints[i].xDistance = LidarRightpoints[i].xDistance;
            Last_LidarRightpoints[i].yDistance = LidarRightpoints[i].yDistance;
        }
        
        // 左侧数据处理
        LidarLeftpoints[i].xDistance = points[340 - i * 5].distance * cos_values[i];
        LidarLeftpoints[i].yDistance = points[340 - i * 5].distance * sin_values[i];
        
        if (!isValidPoint(LidarLeftpoints[i])) {
            if (isValidPoint(Last_LidarLeftpoints[i])) {
                LidarLeftpoints[i] = Last_LidarLeftpoints[i];
            } else {
                LidarLeftpoints[i] = interpolatePoint(LidarLeftpoints, i, SIZE);
            }
        } else {
            Last_LidarLeftpoints[i].xDistance = LidarLeftpoints[i].xDistance;
            Last_LidarLeftpoints[i].yDistance = LidarLeftpoints[i].yDistance;
        }
    }
    
    // 添加直线拟合调用
    curveFitting2D(LidarRightpoints, SIZE, &Right_a, &Right_b);
    curveFitting2D(LidarLeftpoints, SIZE, &Left_a, &Left_b);
}


/*
---------------根据不同情况选择pid\speed---------------
angle_pid_1  直道
angle_pid_2  90度
angle_pid_3  180度
angle_pid_4  270度
*/
static pid_handle angle_pid;
static float angle_pid_1[3] = { 1.5 , 0.1 , 0.20};
static float angle_pid_2[3] = { 2 , 0.15 , 0.10};
static float angle_pid_3[3] = { 2.5 , 0.15 , 0.30};
static float angle_pid_4[3] = { 2 , 0.05 , 0.20 };
float speed=0;
//-------------------跑道曲率的判断----------

// 三点基础曲率计算
float calculateBasicCurvature(DataPoint p1, DataPoint p2, DataPoint p3) {
    float area = fabs((p2.xDistance-p1.xDistance)*(p3.yDistance-p1.yDistance) - 
                     (p3.xDistance-p1.xDistance)*(p2.yDistance-p1.yDistance)) / 2.0f;
    
    float a = sqrtf((p2.xDistance-p1.xDistance)*(p2.xDistance-p1.xDistance) + (p2.yDistance-p1.yDistance)*(p2.yDistance-p1.yDistance));
    float b = sqrtf((p3.xDistance-p2.xDistance)*(p3.xDistance-p2.xDistance) + (p3.yDistance-p2.yDistance)*(p3.yDistance-p2.yDistance));
    float c = sqrtf((p3.xDistance-p1.xDistance)*(p3.xDistance-p1.xDistance) + (p3.yDistance-p1.yDistance)*(p3.yDistance-p1.yDistance));
    
    if (a*b*c < 1e-6) return 0.0f;
    return 4.0f * area / (a * b * c);
}

// 角度变化曲率
float calculateAngleCurvature(DataPoint* points, int size) {
    if (size < 3) return 0.0f;
    
    float total_angle = 0;
    int count = 0;
    
    for (int i = 1; i < size - 1; i++) {
        DataPoint prev = points[i-1];
        DataPoint curr = points[i];
        DataPoint next = points[i+1];
        
        float dx1 = curr.xDistance - prev.xDistance;
        float dy1 = curr.yDistance - prev.yDistance;
        float dx2 = next.xDistance - curr.xDistance;
        float dy2 = next.yDistance - curr.yDistance;
        
        float dot = dx1*dx2 + dy1*dy2;
        float cross = dx1*dy2 - dy1*dx2;
        float angle = atan2f(fabs(cross), (dot > 0.1f ? dot : 0.1f));
        
        if (angle < 1.0f) {
            total_angle += angle;
            count++;
        }
    }
    
    return (count > 0) ? total_angle / count : 0.0f;
}

// 最优曲率计算（多方法融合）
float calculateOptimalCurvature(DataPoint* points, int size) {
    if (size < 5) return 0.0f;
    
    // 方法1: 多点平均曲率
    float multi_point_curvature = 0;
    int multi_count = 0;
    for (int i = 0; i <= size-3; i++) {
        float curvature = calculateBasicCurvature(points[i], points[i+1], points[i+2]);
        if (curvature < 2.0f) {
            multi_point_curvature += curvature;
            multi_count++;
        }
    }
    if (multi_count > 0) multi_point_curvature /= multi_count;
    
    // 方法2: 角度曲率
    float angle_curvature = calculateAngleCurvature(points, size);
    
    // 加权融合
    return 0.6f * multi_point_curvature + 0.4f * angle_curvature;
}

// 移动平均滤波
float movingAverage(float new_value, float* history, int* index) {
    history[*index] = new_value;
    *index = (*index + 1) % MOVING_AVG_SIZE;
    
    float sum = 0;
    for (int i = 0; i < MOVING_AVG_SIZE; i++) {
        sum += history[i];
    }
    return sum / MOVING_AVG_SIZE;
}




float qvlvpanduan() {
    // 使用新的最优曲率计算方法
    float current_right = calculateOptimalCurvature(LidarRightpoints, SIZE);
    float current_left = calculateOptimalCurvature(LidarLeftpoints, SIZE);
    
    // 移动平均滤波
    float smoothed_right = movingAverage(current_right, curvature_history_right, &history_index);
    float smoothed_left = movingAverage(current_left, curvature_history_left, &history_index);
    
    // 计算曲率差
    float curvature_difference = fabs(smoothed_right - smoothed_left);
    
    // 自适应阈值
    static float historical_mean = 0.1f;
    historical_mean = 0.95f * historical_mean + 0.05f * curvature_difference;
    
    // 基于统计的智能决策
    if (curvature_difference < 0.1f) {
        // 直道
        speed = 80;
        angle_pid.Kp = angle_pid_1[0];
        angle_pid.Ki = angle_pid_1[1];
        angle_pid.Kd = angle_pid_1[2];
    } 
    else if (curvature_difference < 0.3f) {
        // 缓弯
        speed = 60;
        angle_pid.Kp = angle_pid_2[0];
        angle_pid.Ki = angle_pid_2[1];
        angle_pid.Kd = angle_pid_2[2];
    }
    else if (curvature_difference < 0.8f) {
        // 中弯
        speed = 50;
        angle_pid.Kp = angle_pid_3[0];
        angle_pid.Ki = angle_pid_3[1];
        angle_pid.Kd = angle_pid_3[2];
    }
    else {
        // 急弯
        speed = 40;
        angle_pid.Kp = angle_pid_4[0];
        angle_pid.Ki = angle_pid_4[1];
        angle_pid.Kd = angle_pid_4[2];
    }
    
    return curvature_difference;
}
//-------------------------------------------------------------------kxy-------------------------------------------------------------------





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
	int dis_l = avg_dis(diss+120, 20);
	int dis_r = avg_dis(diss+242, 20);
	if (dis_l < dis_min && angle_0<0) Servo_Control(0);
	if (dis_r < dis_min && angle_0>0) Servo_Control(0);
}

void run(double kp, double kd, int max_dis)
{
	// lyh的idea 目前的最优算法 update at 2023.10.12
	int dis_min_front = 9999; // 记录前方张角30度内的最短距离
	// 计算前方张角30度内的最短距离
	// for (int i=150; i<=210; ++i){
	// 	if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	// }
	dis_min_front = avg_dis(diss+170,21);
	// 如果前方有障碍物，那么小车减速
	if (dis_min_front < 800 && dis_min_front >= 500) Speed_Control(60);
	else if (dis_min_front < 500) Speed_Control(50);
	else Speed_Control(70);

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
	/*采用中线算法*/
	 
	 double oppsite_side = sqrt(pow(dis_1, 2) + pow(dis_2, 2) - 2 * dis_1 * dis_2 * cos((idx_2 - idx_1) * PI / 180));
	 double mid_line = sqrt((pow(dis_1, 2)+pow(dis_2, 2))/2-pow(oppsite_side, 2)/4);
	 double offset_angle = acos((pow(dis_1, 2)+pow(mid_line, 2)-pow(oppsite_side/2, 2))/(2*dis_1*mid_line));
	 double target_angle = idx_1 + offset_angle * 180 / PI;
	 qvlvpanduan();
	 
	
//	/*采用比例算法*/
//	double lambda = 0.6;
//	// if (diss[(int)idx_1]-diss[(int)idx_2] > 100) lambda = 0.6;
//	// else lambda = 0.4;
//	if (avg_dis(diss+175,10)>1100) lambda = 0.3333;
//	else if (avg_dis(diss+175,10)>900) lambda = 0.4;
//	else if (avg_dis(diss+175,10)>700) lambda = 0.5;
//	else lambda = 0.6;
//	// lambda = 0.5; // 先全部调成0.5 复刻第一轮
//	double dot_ab = dis_1 * dis_2 * cos((idx_2 - idx_1) * PI / 180);
//	double m = sqrt(pow(1-lambda,2)*pow(dis_1,2)+pow(lambda,2)*pow(dis_2,2)+2*(1-lambda)*lambda*dot_ab);
//	double theta = acos(((1-lambda)*pow(dis_1,2)+lambda*dot_ab)/(dis_1*m))*180/PI;
//	double target_angle = idx_1 + theta;

  double error = target_angle - 180;  // 逆时针转的时候error一般小于0，顺时针大于0
  Servo_Control(kp * error + kd * (error - error_last));
	avoid_obstacle();
  error_last = error;
}


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
	

	while(1){
    OLED_Clear();
    MS200_Point* points = MS200_GetPointsData();
			
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
      
			
	OLED_ShowFloat(0,0,angle_0);
	OLED_ShowFloat(60,0,diss[180]/10);
	OLED_ShowFloat(0,32,points[0].distance / 10);
	OLED_ShowFloat(60,32,points[90].distance / 10);
	OLED_ShowFloat(0,48, points[180].distance / 10);
	OLED_ShowFloat(60,48,points[270].distance / 10);
	OLED_Display();

	run(0.6, 0.0, 750);
			/* USER CODE END WHILE */
			
			/* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}




//----------------------------------------------------------------------------------------------------------------------------------------
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

