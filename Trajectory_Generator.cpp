#include "Crobot.hpp"


//�������ݹ�
Point Bezier_Recursion(vector<Point> _Bezier_Control_Point, double _k)
{
	if (_Bezier_Control_Point.size() == 1)
	{
		return _Bezier_Control_Point[0];
	}
	else
	{
		for (size_t i = 0; i < _Bezier_Control_Point.size() - 1; i++)
		{
			_Bezier_Control_Point[i].x = _Bezier_Control_Point[i].x + _k * (_Bezier_Control_Point[i + 1].x - _Bezier_Control_Point[i].x);
			_Bezier_Control_Point[i].y = _Bezier_Control_Point[i].y + _k * (_Bezier_Control_Point[i + 1].y - _Bezier_Control_Point[i].y);
			_Bezier_Control_Point[i].z = _Bezier_Control_Point[i].z + _k * (_Bezier_Control_Point[i + 1].z - _Bezier_Control_Point[i].z);
		}
		_Bezier_Control_Point.pop_back();
		return Bezier_Recursion(_Bezier_Control_Point, _k);
	}
}

extern double Sys_Time;

//�������滮
Point Bezier_Generator(vector<Point> _Bezier_Control_Point, double _start_time,double _period)
{
	double _Time = Sys_Time - _start_time;								//�켣���ɵĲο�ʱ��
	//_Time = (_Time < _period) ? _Time : _period;						//ʱ������
	double k;
	if (_Time < _period)
	{
		k = (double)(_Time / _period) - (int)(_Time / _period);		//���㵱ǰʱ�䴦�ڱ��������ڵĺ�ʱ
	}
	else
	{
		k = 1;
	}
	cout << "k is:" << k << endl;
	return Bezier_Recursion(_Bezier_Control_Point, k);
}

