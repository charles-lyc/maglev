#ifndef UTILS_EX_H_
#define UTILS_EX_H_

#include "datatypes.h"
#include "utils.h"

#define ELEMENT_OF(__BUFFER)						(sizeof(__BUFFER) / sizeof(*(__BUFFER)))

#define SWAP_BYTE_16(__x)							((uint16_t)(((__x >> 8) & 0x00ff) | ((__x << 8) & 0xff00)))

#define OFFSET_ADDR(type, ele)						((unsigned int)(&((type *)0)->ele))

#define UTILS_LP_FAST_INT(value, sample, filter_constant_in_256)	(value -= (filter_constant_in_256) * (value - sample) / 256)

#define UTILS_CALC_DIFF_ROUND_CROSS(_a, _b, _m, _d) \
	{                                               \
		if (_a - _b > _m / 2)                       \
			_d = _b - _a + _m;                      \
		else if (_a - _b < -_m / 2)                 \
			_d = _a - _b + _m;                      \
		else                                        \
			_d = _a - _b;                           \
	}

#define DEGREES_TO_RADIANS(angleDegrees)			((angleDegrees)*M_PI / 180.0f)
#define RADIANS_TO_DEGREES(angleRadians)			((angleRadians)*180.0f / M_PI)

signed long utils_round_div(signed long A,signed long B);
void sincosf(float r, float*s, float*c);
float utils_avg_angles_rad(float *angles, int angles_num);
float utils_avg_angles(float *angles, int angles_num);

#endif
