#pragma once

#include "main.h"
#include "types.h"
#include "filter.h"
#include "differential.h"
#include "LowPassFilter2p.hpp"
#include "pid.h"
#include "motor.h"

typedef enum
{
	MODE_STARTUP = 0,
	MODE_READY,
	MODE_RUN,
	MODE_CALIBRATE,
} maglev_status_t;

class Maglev
{
private:
	maglev_status_t _maglev_status;

	volatile uint32_t _tick;

	bool _invert[2];
	uint16_t *_adc_raw;
	float _hall_offset[3];
	float _hall_raw[3];
	float _hall_pos[3];
	float _hall_pos2[3];
	float _hall_spd[2];
	float _hall_spd2[2];

	DifferentialFloat _diffential[2];

	LowpassFilterFloat _filter_pos[3]{{0.2}, {0.2}, {0.08}};
	LowpassFilterFloat _filter_spd[2]{{0.6}, {0.6}};
	LowpassFilterFloat _filter_votage[2]{{0.4}, {0.4}};

	SlideFilterFloat _filter_pos2[2]{{8}, {8}};
	LowPassFilter2p _filter_spd2[2]{{1000, 50}, {1000, 50}};

	PID _pid_pos[2]{{2, 0, 0, 1, 1, 800, 0.001}, {2, 0, 0, 1, 1, 800, 0.001}};
	PID _pid_spd[2]{{2, 0, 0, 10, 1, 800, 0.001}, {2, 0, 0, 10, 1, 800, 0.001}};

	bool _detect(void);
	void (*_drv_en)(bool);
	void (*_motor_write)(float, float);

public:
	Maglev();

	void init(uint16_t raw[3],
			  bool invert_a,
			  bool invert_b,
			  void (*drv_en)(bool),
			  void (*motor_write)(float, float));
	void adc_sample_callback(void);
	void main_loop(void);
	void calibrate(void);
};
