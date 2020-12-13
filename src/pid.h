#pragma once

#include "types.h"
#include "filter.h"
#include "differential.h"
#include "LowPassFilter2p.hpp"

class PID
{
private:
	// cfg
	float _kp;
	float _ki;
	float _kd;
	float _out_factor;
	float _int_limit;
	float _out_limit;

	float _error;
	float _error_pre;
	float _error_int;
	float _error_diff;
	float _dt;
	float _output;

public:
	PID(
		float kp,
		float ki,
		float kd,
		float out_factor,
		float int_limit,
		float out_limit,
		float dt);
	float apply(float error);
};
