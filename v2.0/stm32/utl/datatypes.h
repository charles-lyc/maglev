#ifndef DATATYPES_H_
#define DATATYPES_H_

/* commond type ---------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <arm_math.h>

/* user mc define -------------------------- */
#ifndef _MATH_DEFINES_DEFINED
    #define _MATH_DEFINES_DEFINED
    // Definitions of useful mathematical constants
    //
    // Define _USE_MATH_DEFINES before including <math.h> to expose these macro
    // definitions for common math constants.  These are placed under an #ifdef
    // since these commonly-defined names are not part of the C or C++ standards
    #define M_E        2.71828182845904523536f   // e
    #define M_LOG2E    1.44269504088896340736f   // log2(e)
    #define M_LOG10E   0.434294481903251827651f  // log10(e)
    #define M_LN2      0.693147180559945309417f  // ln(2)
    #define M_LN10     2.30258509299404568402f   // ln(10)
    #define M_PI       3.14159265358979323846f   // pi
    #define M_PI_2     1.57079632679489661923f   // pi/2
    #define M_PI_4     0.785398163397448309616f  // pi/4
    #define M_1_PI     0.318309886183790671538f  // 1/pi
    #define M_2_PI     0.636619772367581343076f  // 2/pi
    #define M_2_SQRTPI 1.12837916709551257390f   // 2/sqrt(pi)
    #define M_SQRT2    1.41421356237309504880f   // sqrt(2)
    #define M_SQRT1_2  0.707106781186547524401f  // 1/sqrt(2)
#endif

typedef enum
{
	CAN_FOC_ROLL = 0,
	CAN_FOC_PITCH = 1,
	CAN_FOC_YAW = 2,
	CAN_BGC = 3,
	CAN_BROADCAST = 4,
} can_device_addr_t;

typedef struct
{
	uint32_t dest_addr : 4;
	uint32_t src_addr : 4;
	uint32_t is_tpcan : 1;
	uint32_t priority : 2;
	uint32_t reserve : 21;
} can_std_id_t;

typedef __packed struct
{
	uint32_t hardwareversion;
	uint32_t firmwareversion;
	uint32_t firmwaresize;
	uint16_t firmwarecrc16;
	can_device_addr_t can_addr;
	uint16_t crc16;
} fw_conf_t;
extern fw_conf_t FW_Configure;

typedef struct
{
	int32_t over;
	int32_t overhysteresis;
	int32_t under;
	int32_t underhysteresis;
	// bool use_pid;
} value_limit_t;

typedef struct
{
	float sample_value;
	float over;
	float overhysteresis;
	float under;
	float underhysteresis;
	bool use_pid;
} value_f_limit_t;

typedef enum
{
	SHUNT_TYPE_NONE = 0,
	SHUNT_TYPE_2_SHUNT,
	SHUNT_TYPE_3_SHUNT,
} shunt_type_t;

typedef struct
{
	float kp;
	float ki;
	float kd;

	float limit_integral;
	float limit_output;
} pid_param_t;

typedef struct
{
	pid_param_t *params;

	float error;
	float error_pre;
	float error_integral;
	float error_differential;

	float output;
} pid_t;

typedef enum
{
	ENC_SENSOR_LESS = 0,
	ENC_SENSOR_AS5048_SPI,
	ENC_SENSOR_AS5048_PWM,
	ENC_SENSOR_ABI,
	ENC_SENSOR_LINEAR_HALL,
} encoder_sensor_t;

typedef struct
{
	encoder_sensor_t type;
	bool invert;
	float elec_offset;
	float elec_offset_current;
	float mech_offset;					// for bgc
} encoder_param_t;

typedef struct
{
	pid_param_t torque;
	pid_param_t flux;
	pid_param_t speed;
	pid_param_t position;
	pid_param_t enc_detect_torque;
	pid_param_t enc_detect_flux;
} pid_param_group_t;

typedef struct
{
	uint8_t pair_num;
	shunt_type_t shunt_type;
	bool amplifier_invert;

	value_f_limit_t motor_temp_limit;
	value_f_limit_t mos_temp_limit;

	value_limit_t bus_current_limit_ma;
	value_limit_t phase_current_limit_ma;
	value_limit_t bus_voltage_limit_mv;
	value_limit_t bus_power_limit_mw;

} motor_param_t;

typedef struct
{
	float rawdata;
	float mechanic_angle;		// 0 ~ 360
	float mechanic_angle_lpf;
	float electric_angle;		// 0 ~ 360
	float electric_angle_lpf;
	float speed_dps;
} encoder_status_t;

typedef enum
{
	FOC_STATUS_IDLE = 0,
	FOC_STATUS_TEST,
	FOC_STATUS_AMP_CALBR,
	FOC_STATUS_ENC_DETECT,	
	FOC_STATUS_GIMBAL_CTRL,
	FOC_STATUS_WAITE,
	FOC_STATUS_FAULT,
	FOC_STATUS_STOP,
} systemstatus_t;

typedef struct
{
	uint16_t raw[3];
	uint16_t offset[3];
	int32_t amp[3];			// - offset
	int32_t amp_lpf[3];
	int32_t amp_accurate[3];
	float amp_mA[3];
	float id;
	float iq;
	float bus_mA;
} current_group_t;

typedef struct
{
	pid_t torque;
	pid_t flux;
	pid_t speed;
	pid_t position;
	pid_t enc_detect_torque;
	pid_t enc_detect_flux;
} pid_group_t;

typedef struct
{
	bool override; 		// ignore encoder
	bool position;
	bool speed;
	bool current;
	bool brake;
} ctrl_loop_group_t;

typedef struct
{
	float phase;		// in radian
	float position;
	float speed;

	float id;
	float iq;

	float vd;
	float vq;

	float ibus;
	float vbus;

} ctrl_value_group_t;

typedef struct
{
	int16_t id;
	int16_t iq;

	int16_t vd;
	int16_t vq;

	bool enc_detect;
	bool foc_position_test;
	bool conf_store;

} gimbal_ctrl_t;

typedef struct
{
	float duty_max;
	float i_dq_max;

	float Motor_temp;
	float MOS_temp;
	
} motor_status_t;

typedef struct
{
	pid_param_group_t pid_param;

	encoder_param_t encoder;

	motor_param_t motor;

	uint16_t crc16;
} mc_conf_t;

typedef struct
{
	pid_param_group_t pid_param;

	uint16_t crc16;
} mc_app_conf_t;

typedef struct
{
	systemstatus_t status;

	motor_status_t motor;

	encoder_status_t encoder;

	current_group_t currents;

	pid_group_t pid;

	ctrl_loop_group_t ctrl_loop;

	ctrl_value_group_t ctrl_set;

	gimbal_ctrl_t gimbal_set;

} mc_status_t;

#endif /* DATATYPES_H_ */
