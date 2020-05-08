#include "Crobot.hpp"

//系统变量
extern double Sys_Time;                                 //仿真系统时间
extern int Step_Counter;					            //仿真步数计数
extern int Gait;										//运动步态
extern Movement_Para Crobot_Movement_Para;				//运动参数


//更新系统仿真时间
void Update_Sys_Time()
{
    Step_Counter++;
    Sys_Time = ((double)Step_Counter * TIME_STEP) / 1000;
}

//更新当前机器人状态
void Update_Current_Crobot_State(Crobot_State& _current_state,Crobot_Devices& _crobot_devices)
{
	double Leg_Angle_Offset[3] = { 0,(-PI / 4),(3 * PI / 4) };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			_current_state.Crobot_Joint_Angles[i][j] = _crobot_devices.Crobot_PositionSensors[i][j]->getValue() + Leg_Angle_Offset[j];
			//_current_state.Crobot_Touch_Forces[i][j] = _crobot_devices.Crobot_TouchSensors[i]->getValues()[j];
		}

		_current_state.Crobot_Toe_Points[i] = Leg_Kinematic(_current_state.Crobot_Joint_Angles[i]);		//计算足端位置
		//_current_state.Is_Landing[i] = Is_Landing(_current_state.Crobot_Touch_Forces[i]);				//判断足端是否着地
	}
	for (int k = 0; k < 3; k++)
	{
		_current_state.Crobot_IMU_Values[k] = _crobot_devices.Crobot_IMU->getRollPitchYaw()[k];
		_current_state.Crobot_Gyro_Values[k] = _crobot_devices.Crobot_Gyro->getValues()[k];
	}
	_current_state.Crobot_Gait = Gait;
}

//更新由键盘或手柄获得的操作指令
void Update_User_Operation_Instruction(User_Instruction& _instruction,Input_Devices _devices )
{
	if (_devices.JoyStick->isConnected())
	{
		for (int i = 0; i < 6; i++)
		{
			_instruction.Joy.Axis[i] = _devices.JoyStick->getAxisValue(i);
			//轴0：	左摇杆上下，上负下正
			//轴1：	左摇杆左右，左负右正
			//轴2：	右摇杆上下，上负下正
			//轴3：	右摇杆左右，左负右正
			//轴4：	L2，正
			//轴5：	R2，正
		}
		_instruction.Joy.Povs = _devices.JoyStick->getPovValue(0);
		//十字方向按钮值：
		//上：		0 
		//下：		16 
		//左：		4096 
		//右：		256
		_instruction.Joy.PressButton = _devices.JoyStick->getPressedButton();
		//按钮值：
		//start：	0
		//select：	1
		//左摇杆：	2
		//右摇杆：	3
		//L1：		4
		//R1：		5
		//A：		8
		//B：		9
		//X：		10
		//Y：		11
	}
	_instruction.Key = _devices.KeyBoard->getKey();

	//以下代码用于解决键盘长按问题，注释于20200505
	//static int _keyboard_timer_counter = 0;
	//_keyboard_timer_counter++;
	//static int _last_key = -1;
	//int _key = _devices.KeyBoard->getKey();
	//if (_key != -1)
	//{
	//	if (_key != _last_key)
	//	{
	//		_last_key = _key;
	//		_instruction.Key = _key;
	//		//_instruction.Time_Stamp = Sys_Time;
	//	}
	//	else
	//	{
	//		_instruction.Key = -1;
	//	}
	//}
	//if (_keyboard_timer_counter * TIME_STEP > 1000)
	//{
	//	_keyboard_timer_counter = 0;
	//	_last_key = -1;
	//}
}

void Update_Target_Gait(Gait_Switch_Data& _switch_data, User_Instruction _instruction, Crobot_State _current_state)
{
	switch (_instruction.Key)
	{
	case('0'):
	{
		if (Gait != Lay)
		{
			_switch_data.Last_Gait = Gait;
			_switch_data.Current_Gait = Lay;
			_switch_data.Switch_Time_Stamp = Sys_Time;
			for (int i = 0; i < 4; i++)
			{
				_switch_data.Switch_Time_Toe_Position[i] = _current_state.Crobot_Toe_Points[i];
			}
			Gait = Lay;
			cout << "You changed the gait to Lay!" << endl;
		}
		break;
	}
	case('1'):
	{
		if (Gait != Stand)
		{
			_switch_data.Last_Gait = Gait;
			_switch_data.Current_Gait = Stand;
			_switch_data.Switch_Time_Stamp = Sys_Time;
			for (int i = 0; i < 4; i++)
			{
				_switch_data.Switch_Time_Toe_Position[i] = _current_state.Crobot_Toe_Points[i];
			}
			Gait = Stand;
			cout << "You changed the gait to Stand!" << endl;
			cout << "Gait is: " << Gait << endl;

		}
		break;
	}
	case('2'):
	{
		if (Gait != Walk)
		{
			_switch_data.Last_Gait = Gait;
			_switch_data.Current_Gait = Walk;
			_switch_data.Switch_Time_Stamp = Sys_Time;
			for (int i = 0; i < 4; i++)
			{
				_switch_data.Switch_Time_Toe_Position[i] = _current_state.Crobot_Toe_Points[i];
			}
			Gait = Walk;
			cout << "You changed the gait to Walk!" << endl;
		}
		break;
	}
	case('3'):
	{
		if (Gait != Trot)
		{
			_switch_data.Last_Gait = Gait;
			_switch_data.Current_Gait = Trot;
			_switch_data.Switch_Time_Stamp = Sys_Time;
			for (int i = 0; i < 4; i++)
			{
				_switch_data.Switch_Time_Toe_Position[i] = _current_state.Crobot_Toe_Points[i];
			}
			Gait = Trot;
			cout << "You changed the gait to Trot!" << endl;
		}
		break;
	}
	case('4'):
	{
		if (Gait != Run)
		{
			_switch_data.Last_Gait = Gait;
			_switch_data.Current_Gait = Run;
			_switch_data.Switch_Time_Stamp = Sys_Time;
			for (int i = 0; i < 4; i++)
			{
				_switch_data.Switch_Time_Toe_Position[i] = _current_state.Crobot_Toe_Points[i];
			}
			Gait = Run;
			cout << "You changed the gait to Run!" << endl;
		}
		break;
	}
	default:
		break;
	}
}


void Update_Target_Toe_Point(Point* _target_toe_point,Gait_Switch_Data _switch_data)
{
	cout << "Gait is: " << Gait << endl;
	switch (Gait)
	{
	case(Lay):
	{
		double Lay_Angles[3] = { 0,-PI / 4,3 * PI / 4 };
		Point Lay_Point = Leg_Kinematic(Lay_Angles);
		double initial_period = 1;

		for (int i = 0; i < 4; i++)
		{
			vector<Point> _CTRL_Point = { _switch_data.Switch_Time_Toe_Position[i],Lay_Point };
			_target_toe_point[i] = Bezier_Generator(_CTRL_Point, _switch_data.Switch_Time_Stamp, initial_period);
			/*cout << "_target_toe_point_x:" << _target_toe_point[i].x << endl;
			cout << "_target_toe_point_y:" << _target_toe_point[i].y << endl;
			cout << "_target_toe_point_z:" << _target_toe_point[i].z << endl;*/
		}
		break;
	}
	case(Stand):
	{
		//Point Stand_Transition_Point = { 20,-150,0 };
		Point Stand_Point = { 0,-140,0 };
		double initial_period = 1;

		for (int i = 0; i < 4; i++)
		{
			vector<Point> _CTRL_Point = { _switch_data.Switch_Time_Toe_Position[i],Stand_Point};
			_target_toe_point[i] = Bezier_Generator(_CTRL_Point,_switch_data.Switch_Time_Stamp, initial_period);
		}
		break;
	}
	case(Trot):
	{
		double initial_period = 0.25;
		double distance = 10;
		Point CTRL_Point1 = { -distance,-140,0 };
		Point CTRL_Point2 = { -distance,-120,0 };
		Point CTRL_Point3 = { distance,-120,0 };
		Point CTRL_Point4 = { distance,-140,0 };
		Point CTRL_Point5 = { distance,-160,0 };
		Point CTRL_Point6 = { -distance,-160,0 };

		vector<Point> Fly_CTRL_Point = { CTRL_Point1,CTRL_Point2,CTRL_Point3,CTRL_Point4 };
		vector<Point> Land_CTRL_Point = { CTRL_Point4,CTRL_Point5,CTRL_Point6,CTRL_Point1 };

		bool Trot_State = 0;
		double Gait_time_stamp = _switch_data.Switch_Time_Stamp + (int)((Sys_Time- _switch_data.Switch_Time_Stamp) / initial_period) * initial_period;
		/*cout << "Switch_Time_Stamp  is: " << _switch_data.Switch_Time_Stamp << endl;
		cout << "Gait_time_stamp  is: " << Gait_time_stamp << endl;
		cout << "Sys_Time  is: " << Sys_Time << endl;*/

		if(((int)((Sys_Time- _switch_data.Switch_Time_Stamp) / initial_period)) % 2)
		{
			Trot_State = 0;
			_target_toe_point[0] = Bezier_Generator(Fly_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[3] = Bezier_Generator(Fly_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[1] = Bezier_Generator(Land_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[2] = Bezier_Generator(Land_CTRL_Point, Gait_time_stamp, initial_period);

		}
		else
		{
			Trot_State = 1;
			_target_toe_point[0] = Bezier_Generator(Land_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[3] = Bezier_Generator(Land_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[1] = Bezier_Generator(Fly_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[2] = Bezier_Generator(Fly_CTRL_Point, Gait_time_stamp, initial_period);
		}
		//cout << "now Trot_State is: " << Trot_State << endl;
		break;

	}
	case(Run):
	{
		double initial_period = 0.2;
		double distance = 50;
		Point CTRL_Point1 = { -distance,-140,0 };
		Point CTRL_Point2 = { -distance,-120,0 };
		Point CTRL_Point3 = { distance,-120,0 };
		Point CTRL_Point4 = { distance,-140,0 };
		Point CTRL_Point5 = { distance,-160,0 };
		Point CTRL_Point6 = { -distance,-160,0 };

		vector<Point> Fly_CTRL_Point = { CTRL_Point1,CTRL_Point2,CTRL_Point3,CTRL_Point4 };
		vector<Point> Land_CTRL_Point = { CTRL_Point4,CTRL_Point5,CTRL_Point6,CTRL_Point1 };

		bool Run_State = 0;
		double Gait_time_stamp = _switch_data.Switch_Time_Stamp + (int)((Sys_Time - _switch_data.Switch_Time_Stamp) / initial_period) * initial_period;
		/*cout << "Switch_Time_Stamp  is: " << _switch_data.Switch_Time_Stamp << endl;
		cout << "Gait_time_stamp  is: " << Gait_time_stamp << endl;
		cout << "Sys_Time  is: " << Sys_Time << endl;*/

		if (((int)((Sys_Time - _switch_data.Switch_Time_Stamp) / initial_period)) % 2)
		{
			Run_State = 0;
			_target_toe_point[0] = Bezier_Generator(Fly_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[2] = Bezier_Generator(Fly_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[1] = Bezier_Generator(Land_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[3] = Bezier_Generator(Land_CTRL_Point, Gait_time_stamp, initial_period);

		}
		else
		{
			Run_State = 1;
			_target_toe_point[0] = Bezier_Generator(Land_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[2] = Bezier_Generator(Land_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[1] = Bezier_Generator(Fly_CTRL_Point, Gait_time_stamp, initial_period);
			_target_toe_point[3] = Bezier_Generator(Fly_CTRL_Point, Gait_time_stamp, initial_period);
		}
		//cout << "now Trot_State is: " << Trot_State << endl;
		break;
	}
	default:
		break;
	}
}

void Update_Target_Angles(double _target_Angles[4][3], Point* _target_toe_point)
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			_target_Angles[i][j] = Inverse_Kinematic(_target_toe_point[i])[j];
		}
	}
}



void Update_Gait_Stamp(Movement_Para _parameter)
{
	int _gait;
	double _gait_stamp;
	switch (_gait)
	{
	case(Trot):
	{
		int n = (int)((Sys_Time - _gait_stamp) / (2 * _parameter.period));
		double RB_Flying_Stamp = _gait_stamp + n * 2 * _parameter.period;
		double RB_Landing_Stamp = RB_Flying_Stamp + _parameter.period;
	}
	default:
		break;
	}
}

Bezier_Para Update_Flying_Bezier_Parameter(Movement_Para _parameter,User_Instruction _instruction)
{
	Point _initial_landing_point = { 50,-150,0 };
	Point _landing_point;

	Bezier_Para _bezier_parameter;
	/*_bezier_parameter._start_time = _instruction.Time_Stamp;*/
	//_bezier_parameter._period = (double)(1 / _parameter.frequence);
	_bezier_parameter._bezier_control_point = { {-50,-150,0},{-50,-110,0}, _initial_landing_point };

	switch (_instruction.Key)
	{
	case(Keyboard::LEFT):
	{
		_parameter.y += 20;
		break;
	}
	case(Keyboard::RIGHT):
	{
		_parameter.y -= 20;
		break;
	}
	case(Keyboard::UP):
	{
		_parameter.x += 5;
		break;
	}
	case(Keyboard::DOWN):
	{
		_parameter.x -= 5;
		break;
	}
	default:
		break;
	}
	_landing_point.x = _initial_landing_point.x + _parameter.x;
	_landing_point.y = _initial_landing_point.y + _parameter.y;
	_landing_point.z = _initial_landing_point.z + _parameter.z;

	_bezier_parameter._bezier_control_point[3] = _landing_point;

	return _bezier_parameter;
}

Bezier_Para Update_Landing_Bezier_Parameter(Movement_Para _parameter, User_Instruction _instruction)
{


}




