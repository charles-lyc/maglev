#include "pid.h"
#include "sw_conf.h"

void pid_init(pid_t *pid, pid_param_t *pid_param)
{
	pid->params = pid_param;
	pid->error = 0;
	pid->error_pre = 0;
	pid->error_integral = 0;
	pid->error_differential = 0;
	pid->output = 0;
}

float pid_regulator(float error, pid_t *pid)
{
	float p_term, i_term, d_term, output, kp, ki, kd;

	kp = pid->params->kp;
	ki = pid->params->ki;
	kd = pid->params->kd;

	pid->error = error;
	
	p_term = kp * error;

	if (ki == 0)
	{
		pid->error_integral = 0;
	}
	else
	{
		i_term = pid->error_integral + ki * error;

		if (i_term > pid->params->limit_integral)
		{
			pid->error_integral = pid->params->limit_integral;
		}
		else if (i_term < -pid->params->limit_integral)
		{
			pid->error_integral = -pid->params->limit_integral;
		}
		else
		{
			pid->error_integral = i_term;
		}
	}

	d_term = kd * (error - pid->error_pre) / PID_CTRL_DT;
	pid->error_pre = error;

	output = p_term + pid->error_integral + d_term;

	if (output > pid->params->limit_output)
	{
		pid->output = pid->params->limit_output;
	}
	else if (output < -pid->params->limit_output)
	{
		pid->output = -pid->params->limit_output;
	}
	else
	{
		pid->output = output;
	}

	return pid->output;
}

