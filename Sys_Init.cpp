#include "Crobot.hpp"

//电机名
const char Motor_Names[4][3][30] = {
	{ "RB_RotationMotor1", "RB_RotationMotor2", "RB_RotationMotor3" } ,
	{ "RF_RotationMotor1", "RF_RotationMotor2", "RF_RotationMotor3" } ,
	{ "LB_RotationMotor1", "LB_RotationMotor2", "LB_RotationMotor3" } ,
	{ "LF_RotationMotor1", "LF_RotationMotor2", "LF_RotationMotor3" } };

//位置传感器名
const char PositionSensor_Names[4][3][30] = {
	{ "RB_PositionSensor1", "RB_PositionSensor2", "RB_PositionSensor3" } ,
	{ "RF_PositionSensor1", "RF_PositionSensor2", "RF_PositionSensor3" } ,
	{ "LB_PositionSensor1", "LB_PositionSensor2", "LB_PositionSensor3" } ,
	{ "LF_PositionSensor1", "LF_PositionSensor2", "LF_PositionSensor3" } };

//接触传感器名
const char TouchSensor_Names[4][100] = { "RB_TouchSensor","RF_TouchSensor","LB_TouchSensor","LF_TouchSensor" };
//IMU名
const char IMU_Name[100] = "IMU";
//GYRO名
const char Gyro_Name[100] = "Crobot_Gyro";




//系统初始化
void Sys_Init(Crobot_Devices& _crobot_devices,Robot* _crobot)
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			_crobot_devices.Crobot_Motors[i][j] = _crobot->getMotor(Motor_Names[i][j]);
			_crobot_devices.Crobot_Motors[i][j]->setControlPID(50, 1, 0.1);
			_crobot_devices.Crobot_PositionSensors[i][j] = _crobot->getPositionSensor(PositionSensor_Names[i][j]);
			_crobot_devices.Crobot_PositionSensors[i][j]->enable(TIME_STEP);

		}
		_crobot_devices.Crobot_TouchSensors[i] = _crobot->getTouchSensor(TouchSensor_Names[i]);
		_crobot_devices.Crobot_TouchSensors[i]->enable(TIME_STEP);

	}
	_crobot_devices.Crobot_IMU = _crobot->getInertialUnit(IMU_Name);
	_crobot_devices.Crobot_IMU->enable(TIME_STEP);
	_crobot_devices.Crobot_Gyro = _crobot->getGyro(Gyro_Name);
	_crobot_devices.Crobot_Gyro->enable(TIME_STEP);


	cout << " Crobot has been Initialized!" << endl;
}

void Initial_test(Crobot_Devices _crobot_devices)
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cout << _crobot_devices.Crobot_Motors[i][j]->getName() << endl;
			cout << _crobot_devices.Crobot_PositionSensors[i][j]->getName() << endl;
		}
		cout << _crobot_devices.Crobot_TouchSensors[i]->getName() << endl;
	}
	cout << _crobot_devices.Crobot_IMU->getName() << endl;
	cout << _crobot_devices.Crobot_Gyro->getName() << endl;
}

void Input_Devices_Init(Input_Devices& _input_devices, Robot* _crobot)
{
	_input_devices.KeyBoard = _crobot->getKeyboard();
	_input_devices.KeyBoard->enable(TIME_STEP);
	_input_devices.JoyStick = _crobot->getJoystick();
	_input_devices.JoyStick->enable(TIME_STEP);
}