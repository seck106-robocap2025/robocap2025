#ifndef _LINEARREGRESSION_
#define _LINEARREGRESSION_
/*10,15,20,25,30,35,40,45,50,55,60*/
/*350,345,340,335,330,325,320,315,310,305,300*/


typedef struct {
    double angle; // 角度，以弧度为单位
    double distance; // 距离
} PolarPoint , *PP;
//笛卡尔坐标
typedef struct{
    double x;
    double y;
} CartesianPoint , *CP;
//平均值
typedef struct{
	double X;
	double Y;
} Average , *Aver;
//方程系数
typedef struct{
	double a;
	double b;
} EquationParams , *EP;


#endif
