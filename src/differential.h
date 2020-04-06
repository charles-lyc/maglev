#pragma once

#include "types.h"

template <class T>
class Differential
{
private:
	T _samples[5];

public:
	Differential();
	~Differential();
	T apply(const T sample);
	void reset(void);
};


typedef Differential<int> DifferentialInt;
typedef Differential<float> DifferentialFloat;

