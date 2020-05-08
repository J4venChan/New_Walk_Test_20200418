#include "Crobot.hpp"

void Motor_Run(Motor* _crobot_motors[4][3],double _angles[4][3])
{
	double Angle_Original_OffSet[3] = { 0,PI / 4,-3 * PI / 4 };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			_crobot_motors[i][j]->setPosition(_angles[i][j]+ Angle_Original_OffSet[j]);
		}
	}
}