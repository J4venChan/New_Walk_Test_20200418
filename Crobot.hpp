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

//���ƿռ�
using namespace webots;
using namespace std;

//�����
struct Point
{
	double x;				//x����
	double y;				//y����
	double z;				//z����
};

//��
typedef double* Force;

//PID ���
struct PID_Error
{
	double Current_Error;	//��ǰ���
	double I_Error;			//������
	double D_Error;			//���΢��
	double Last_Error;		//�ϴ����
};
//PID ����
struct PID_Parameters
{
	double K_P;				//����ϵ��
	double K_I;				//����ϵ��
	double K_D;				//΢��ϵ��
};

//�����������豸
struct Crobot_Devices
{
	Motor* Crobot_Motors[4][3];
	PositionSensor* Crobot_PositionSensors[4][3];
	TouchSensor* Crobot_TouchSensors[4];
	InertialUnit* Crobot_IMU;
	Gyro* Crobot_Gyro;
};

//������״̬
struct Crobot_State
{
	double Crobot_Joint_Angles[4][3];		//�ؽڽ�
	Point Crobot_Toe_Points[4];				//���λ��
	Force Crobot_Touch_Forces[4];			//��˽Ӵ���
	double Crobot_IMU_Values[3];			//IMU��Roll��Pitch��Yaw
	double Crobot_Gyro_Values[3];			//�����ǣ�X��Y��X
	int Crobot_Gait;						//��̬
	bool Is_Landing[4];						//����Ƿ��ŵ�
};

//�˻��������ݽṹ
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
	int Key;								//��������ֵ
	Joy_Stick_Data Joy;					    //�ֱ�����ֵ
	double Instruction_Time_Stamp;			//ָ��ʱ���
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

//����
#define PI (3.1415926)				            //Բ����PI = 3.1415926
#define TIME_STEP (16)				            //���沽��
#define Data_Sampling_Period (1)	            //ÿ�� Data_Sampling_Period ���� ��¼һ������
#define L1 (17.0)								//��1����,��λmm
#define L2 (80.0)								//��2����,��λmm							
#define L3 (93.0)								//��3����,��λmm

#define RB_Leg (0)								//�Һ���
#define RF_Leg (1)								//��ǰ��
#define LB_Leg (2)								//�����
#define LF_Leg (3)								//��ǰ��

#define Hip1 (0)								//�Źؽ�1
#define Hip2 (1)								//�Źؽ�2
#define Knee (2)								//ϥ�ؽ�

#define Lay (0)
#define Stand (1)								//ԭ��վ��
#define Walk (2)								//walk��̬
#define Trot (3)								//Trot��̬
#define Run (4)									//Run��̬
#define Balance (5)								//��̬ƽ��


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
