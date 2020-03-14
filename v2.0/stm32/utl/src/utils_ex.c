#include "datatypes.h"
#include "utils_ex.h"

void utils_norm_angle_in_range(float *angle, float range1, float range2)
{
	while (*angle < range1)
	{
		*angle += 360.0f;
	}

	while (*angle > range2)
	{
		*angle -= 360.0f;
	}
}

signed long utils_round_div(signed long A,signed long B)
{
	if (A<0)
		return (A-B/2)/B;
	else
		return (A+B/2)/B;
}

// Note: angles MUST be normalize by utils_norm_angle_rad()
// output: -pi <= average < pi,
float utils_avg_angles_rad(float *angles, int angles_num)
{
	float *p_angle;
	float angle, angle0;
	float average;

	p_angle = angles;
	angle0 = *angles;
	average = 0;

	for (uint32_t i = 0; i < angles_num; i++)
	{
		angle = *p_angle++;
		average += angle0 + utils_angle_difference_rad(angle, angle0);
	}
	average /= angles_num;
	utils_norm_angle_rad(&average);

	return average;
}

// output: -180 ~ 180
// output: 0 ~ 360
float utils_avg_angles(float *angles, int angles_num)
{
	float *p_angle;
	float angle, angle0;
	float average;

	p_angle = angles;
	angle0 = *angles;
	average = 0;

	for (uint32_t i = 0; i < angles_num; i++)
	{
		angle = *p_angle++;
		average += angle0 + utils_angle_difference(angle, angle0);
	}
	average /= angles_num;
	// utils_norm_angle_in_range(&average, -M_PI, M_PI);
	utils_norm_angle(&average);

	return average;
}




