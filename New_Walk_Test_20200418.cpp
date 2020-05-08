// File:          New_Walk_Test_20200418.cpp
// Date:            20200418
// Description:     �ܹ��ع�
// Author:          JavenChan
// Modifications:

#include "Crobot.hpp"

//ϵͳ����
double Sys_Time(0);                                     //����ϵͳʱ��					 
int Step_Counter(0);					                //���沽������

int Gait = 0;                                           //��̬
Movement_Para Crobot_Movement_Para = { 1,1,0,50,0,0 };  //�˶�����

Crobot_Devices This_Crobot_Devices;                     //����ϵͳ�豸

Crobot_State Current_Crobot_State;                      //�����˵�ǰ״̬

Crobot_State Target_Crobot_State;                       //������Ŀ��״̬

Input_Devices Crobot_Input_Devices;                     //�˻������豸

User_Instruction User_Input_Instruction;               //����ָ��

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
      //����ϵͳʱ��
      Update_Sys_Time();
      //���µ�ǰϵͳ״̬
      Update_Current_Crobot_State(Current_Crobot_State, This_Crobot_Devices);
      //�����û������ָ��
      Update_User_Operation_Instruction(User_Input_Instruction, Crobot_Input_Devices);
      //�����û�ָ�����Ŀ�경̬
      Update_Target_Gait(Crobot_Gait_Switch_Data, User_Input_Instruction, Current_Crobot_State);
      //cout << "The gait is:" << Gait << endl;
      //���ݲ�̬�������λ��
      Update_Target_Toe_Point( Target_Toe_Points, Crobot_Gait_Switch_Data);
      //�������λ�ø��¹ؽڽǶ�
      Update_Target_Angles(Target_Angles, Target_Toe_Points);
      //���ݹؽ�Ŀ��Ƕ�ʹ�ܵ��
      Motor_Run(This_Crobot_Devices.Crobot_Motors, Target_Angles);
  }
  // Enter here exit cleanup code.
  delete This_Crobot;
  return 0;
}
