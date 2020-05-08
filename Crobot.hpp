#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Joystick.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <time.h>

//名称空间
using namespace webots;
using namespace std;

//坐标点
struct Point
{
	double x;				//x坐标
	double y;				//y坐标
	double z;				//z坐标
};

//力
typedef double* Force;

//PID 误差
struct PID_Error
{
	double Current_Error;	//当前误差
	double I_Error;			//误差积分
	double D_Error;			//误差微分
	double Last_Error;		//上次误差
};
//PID 参数
struct PID_Parameters
{
	double K_P;				//比例系数
	double K_I;				//积分系数
	double K_D;				//微分系数
};

//机器人所有设备
struct Crobot_Devices
{
	Motor* Crobot_Motors[4][3];
	PositionSensor* Crobot_PositionSensors[4][3];
	TouchSensor* Crobot_TouchSensors[4];
	InertialUnit* Crobot_IMU;
	Gyro* Crobot_Gyro;
};

//机器人状态
struct Crobot_State
{
	double Crobot_Joint_Angles[4][3];		//关节角
	Point Crobot_Toe_Points[4];				//足端位置
	Force Crobot_Touch_Forces[4];			//足端接触力
	double Crobot_IMU_Values[3];			//IMU：Roll，Pitch，Yaw
	double Crobot_Gyro_Values[3];			//陀螺仪：X，Y，X
	int Crobot_Gait;						//步态
	bool Is_Landing[4];						//足端是否着地
};

//人机交互数据结构
struct Input_Devices
{
	Keyboard* KeyBoard;
	Joystick* JoyStick;
};

struct Joy_Stick_Data
{
	int Axis[6];
	int Povs;
	int PressButton;
};

struct User_Instruction
{
	int Key;								//键盘输入值
	Joy_Stick_Data Joy;					    //手柄输入值
	double Instruction_Time_Stamp;			//指令时间戳
};

struct Gait_Switch_Data
{
	int Last_Gait;
	int Current_Gait;
	double Switch_Time_Stamp;
	Point Switch_Time_Toe_Position[4] = { {36.4,-73.5,0},{36.4,-73.5,0} ,{36.4,-73.5,0} ,{36.4,-73.5,0} };
};

struct Movement_Para
{
	bool direction;
	double period;
	double time_stamp;
	double x;
	double y;
	double z;
};

struct Bezier_Para
{
	double _start_time;
	double _period;
	vector<Point> _bezier_control_point;
};

//常量
#define PI (3.1415926)				            //圆周率PI = 3.1415926
#define TIME_STEP (16)				            //仿真步长
#define Data_Sampling_Period (1)	            //每走 Data_Sampling_Period 步， 记录一次数据
#define L1 (17.0)								//杆1长度,单位mm
#define L2 (80.0)								//杆2长度,单位mm							
#define L3 (93.0)								//杆3长度,单位mm

#define RB_Leg (0)								//右后腿
#define RF_Leg (1)								//右前腿
#define LB_Leg (2)								//左后腿
#define LF_Leg (3)								//左前腿

#define Hip1 (0)								//髋关节1
#define Hip2 (1)								//髋关节2
#define Knee (2)								//膝关节

#define Lay (0)
#define Stand (1)								//原地站立
#define Walk (2)								//walk步态
#define Trot (3)								//Trot步态
#define Run (4)									//Run步态
#define Balance (5)								//姿态平衡


void Sys_Init(Crobot_Devices& _crobot_devices, Robot* _crobot);
void Initial_test(Crobot_Devices _crobot_devices);
void Input_Devices_Init(Input_Devices& _input_devices, Robot* _crobot);
void Update_Sys_Time();
void Update_Current_Crobot_State(Crobot_State& _current_state, Crobot_Devices& _crobot_devices);
void Motor_Run(Motor* _crobot_motors[4][3], double _angles[4][3]);

Point Leg_Kinematic(double _angles[3]);
bool Is_Landing(Force force);
Point Bezier_Generator(vector<Point> _Bezier_Control_Point, double _start_time, double _period);

void Update_Target_Gait(Gait_Switch_Data& _switch_data, User_Instruction _instruction, Crobot_State _current_state);
void Update_Target_Toe_Point(Point* _target_toe_point, Gait_Switch_Data _switch_data);
void Update_Target_Angles(double _target_Angles[4][3], Point* _target_toe_point);

double* Inverse_Kinematic(Point _point);

void Update_User_Operation_Instruction(User_Instruction& _instruction, Input_Devices _devices);
