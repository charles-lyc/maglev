#include "maglev.h"
#include "filter.h"

Maglev::Maglev()
{
	_maglev_status = MODE_STARTUP;
}

void Maglev::init(
	uint16_t raw[3],
	bool invert_a,
	bool invert_b,
	void (*drv_en)(bool),
	void (*motor_write)(float, float))
{
	_invert[0] = invert_a;
	_invert[1] = invert_b;
	_drv_en = drv_en;
	_motor_write = motor_write;
	_adc_raw = raw;
	_hall_offset[2] = 0;

	_drv_en(false);
	_maglev_status = MODE_READY;
}

void Maglev::adc_sample_callback(void)
{
	if (_maglev_status == MODE_STARTUP)
		return;

	float tmp;

	_tick++;

	if (_maglev_status == MODE_RUN ||
		_maglev_status == MODE_READY ||
		_maglev_status == MODE_CALIBRATE)
	{
		// adc noise: 10(in 4086)
		// hall pos noise: 10(in 4086)
		// speed noise: 10()

		for (size_t i = 0; i < 2; i++)
		{
			if (_invert[i])
				_hall_raw[i] = 4096 - _adc_raw[i] - _hall_offset[i];
			else
				_hall_raw[i] = _adc_raw[i] - _hall_offset[i];

			_hall_pos[i] = _filter_pos[i].apply(_hall_raw[i]);

			_hall_pos2[i] = _filter_pos2[i].apply(_hall_raw[i]);

			tmp = _diffential[i].apply(_hall_pos[i]);

			_hall_spd[i] = _filter_spd[i].apply(tmp);

			_hall_spd2[i] = _filter_spd2[i].apply(tmp);
		}
	}
	
	_hall_raw[2] = _adc_raw[2] - _hall_offset[2];
	_hall_pos[2] = _filter_pos[2].apply(_hall_raw[2]);

	if (_maglev_status == MODE_RUN)
	{
		float votage[2], out_pos, out_spd;

		for (size_t i = 0; i < 2; i++)
		{
			out_pos = _pid_pos[i].apply(0 - _hall_pos[i]);
			out_spd = _pid_spd[i].apply(0 - _hall_spd[i]);

			votage[i] = _filter_votage[i].apply(out_pos + out_spd);

		}

		_motor_write(votage[0], votage[1]);
	}
	else
	{
		_motor_write(0, 0);
	}
}

void Maglev::main_loop(void)
{
	if (_maglev_status == MODE_STARTUP)
		return;

	_drv_en(_detect());
}

void Maglev::calibrate(void)
{
	if (_maglev_status == MODE_STARTUP)
		return;

	_maglev_status = MODE_CALIBRATE;
	uint32_t tick = _tick;

	for (size_t i = 0; i < 2; i++)
		_hall_offset[i] = 0;
	
	while (tick + 100 > _tick)
		;

	for (size_t i = 0; i < 2; i++)
		_hall_offset[i] = _hall_pos[i];
	if (_hall_offset[2] == 0)
		_hall_offset[2] = _hall_pos[2];

	_maglev_status = MODE_RUN;
}

bool Maglev::_detect(void)
{
	if (_maglev_status == MODE_STARTUP || 
		_maglev_status == MODE_CALIBRATE)
		return false;

	if (_hall_pos[2] < -200)
	{
		_maglev_status = MODE_RUN;
		return true;
	}
	else
	{
		_maglev_status = MODE_READY;
		return false;
	}
}
