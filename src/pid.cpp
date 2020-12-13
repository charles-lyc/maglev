#include "pid.h"

#define FLT_EPSILON 1.192092896e-07F

template <typename T>
inline bool is_zero(const T fVal1)
{
	return (fabsf(static_cast<float>(fVal1)) < FLT_EPSILON);
}

PID::PID(
	float kp,
	float ki,
	float kd,
	float out_factor,
	float int_limit,
	float out_limit,
	float dt)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;

	_out_factor = out_factor;
	_int_limit = int_limit;
	_out_limit = out_limit;
	
	_dt = dt;

	_error = 0;
	_error_pre = 0;
	_error_int = 0;
	_error_diff = 0;
	_output = 0;
}

float PID::apply(float error)
{
	float p, i, d, output;

	_error_pre = _error;
	_error = error;

	p = _kp * _error;

	if (is_zero(_ki))
	{
		_error_int = 0;
	}
	else
	{
		_error_int += _error;

		if (_error_int > _int_limit)
		{
			_error_int = _int_limit;
		}
		else if (_error_int < -_int_limit)
		{
			_error_int = -_int_limit;
		}
	}
	i = _ki * _error_int;

	d = _kd * (_error - _error_pre) / _dt;

	output = (p + i + d) * _out_factor;

	if (output > _out_limit)
		output = _out_limit;
	else if (output < -_out_limit)
		output = -_out_limit;

	_output = output;

	return _output;
}

