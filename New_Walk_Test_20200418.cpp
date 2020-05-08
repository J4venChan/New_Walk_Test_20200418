// File:          New_Walk_Test_20200418.cpp
// Date:            20200418
// Description:     架构重构
// Author:          JavenChan
// Modifications:

#include "Crobot.hpp"

//系统变量
double Sys_Time(0);                                     //仿真系统时间					 
int Step_Counter(0);					                //仿真步数计数

int Gait = 0;                                           //步态
Movement_Para Crobot_Movement_Para = { 1,1,0,50,0,0 };  //运动参数

Crobot_Devices This_Crobot_Devices;                     //仿真系统设备

Crobot_State Current_Crobot_State;                      //机器人当前状态

Crobot_State Target_Crobot_State;                       //机器人目标状态

Input_Devices Crobot_Input_Devices;                     //人机交互设备

User_Instruction User_Input_Instruction;               //输入指令

double Target_Angles[4][3];
Point Target_Toe_Points[4];
Gait_Switch_Data Crobot_Gait_Switch_Data;


int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot* This_Crobot = new Robot();

  Sys_Init(This_Crobot_Devices, This_Crobot);
  Input_Devices_Init(Crobot_Input_Devices, This_Crobot);
  Initial_test(This_Crobot_Devices);

  while (This_Crobot->step(TIME_STEP) != -1)
  {
      //更新系统时间
      Update_Sys_Time();
      //更新当前系统状态
      Update_Current_Crobot_State(Current_Crobot_State, This_Crobot_Devices);
      //更新用户输入的指令
      Update_User_Operation_Instruction(User_Input_Instruction, Crobot_Input_Devices);
      //根据用户指令更新目标步态
      Update_Target_Gait(Crobot_Gait_Switch_Data, User_Input_Instruction, Current_Crobot_State);
      //cout << "The gait is:" << Gait << endl;
      //根据步态更新足端位置
      Update_Target_Toe_Point( Target_Toe_Points, Crobot_Gait_Switch_Data);
      //根据足端位置更新关节角度
      Update_Target_Angles(Target_Angles, Target_Toe_Points);
      //根据关节目标角度使能电机
      Motor_Run(This_Crobot_Devices.Crobot_Motors, Target_Angles);
  }
  // Enter here exit cleanup code.
  delete This_Crobot;
  return 0;
}
