#include "Crobot.hpp"

//检测是否着地
bool Is_Landing(Force force)
{
	double _F = pow(force[0], 2) + pow(force[1], 2) + pow(force[2], 2);
	return ((_F > 0.01) ? true : false);
}

//正运动学
Point Leg_Kinematic(double _angles[3])
{
	Point _point;
	_point.x = L2 * sin(_angles[Hip2]) + L3 * sin(_angles[Hip2] + _angles[Knee]);
	_point.y = -cos(_angles[Hip1]) * (L1 + L2 * cos(_angles[Hip2]) + L3 * cos(_angles[Hip2] + _angles[Knee]));
	_point.z = -sin(_angles[Hip1]) * (L1 + L2 * cos(_angles[Hip2]) + L3 * cos(_angles[Hip2] + _angles[Knee]));
	return _point;
}


//逆运动学
double* Inverse_Kinematic(Point _point)
{
	double _Angles[3];
	_Angles[0] = atan(_point.z / _point.y);
	double temp_sin_phi1 = sin(_Angles[0]);
	double temp_cos_phi1 = cos(_Angles[0]);
	double temp_A = sqrt(pow(_point.y, 2) + pow(_point.z, 2)) - L1;
	double temp_B = _point.x;
	double temp_A_2 = pow(temp_A, 2);
	double temp_B_2 = pow(temp_B, 2);
	double temp_L2_2 = pow(L2, 2);
	double temp_L3_2 = pow(L3, 2);
	double temp_A_4 = pow(temp_A, 4);
	double temp_B_4 = pow(temp_B, 4);
	double temp_L2_4 = pow(L2, 4);
	double temp_L3_4 = pow(L3, 4);
	_Angles[1] = -2 * atan((sqrt(-temp_A_4 - 2 * temp_A_2 * temp_B_2 + 2 * temp_A_2 * temp_L2_2 + 2 * temp_A_2 * temp_L3_2 - temp_B_4 + 2 * temp_B_2 * temp_L2_2 + 2 * temp_B_2 * temp_L3_2 - temp_L2_4 + 2 * temp_L2_2 * temp_L3_2 - temp_L3_4) - 2 * temp_B * L2) / (temp_A_2 + 2 * temp_A * L2 + temp_B_2 + temp_L2_2 - temp_L3_2));
	_Angles[2] = 2 * atan(sqrt((temp_A_2 + temp_B_2 - temp_L2_2 + 2 * L2 * L3 - temp_L3_2) * (-temp_A_2 - temp_B_2 + temp_L2_2 + 2 * L2 * L3 + temp_L3_2)) / (temp_A_2 + temp_B_2 - temp_L2_2 + 2 * L2 * L3 - temp_L3_2));
	return _Angles;
}