#pragma once

#include "types.h"

template <class T>
class SlideFilter
{
private:
	T *_buffer;
	uint16_t _size;
	uint16_t _indx;
public:
	SlideFilter(uint16_t size);
	~SlideFilter();
	T apply(const T sample);
	void reset(void);
};
typedef SlideFilter<int> SlideFilterInt;
typedef SlideFilter<float> SlideFilterFloat;


template <class T>
class LowpassFilter
{
private:
	float _factor;
	T _prev;
public:
	LowpassFilter(float factor);
	T apply(const T sample);
};
typedef LowpassFilter<int> LowpassFilterInt;
typedef LowpassFilter<float> LowpassFilterFloat;


