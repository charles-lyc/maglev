#include "differential.h"

template <class T>
Differential<T>::Differential()
{
	reset();
}

template <class T>
Differential<T>::~Differential()
{
}

template <class T>
T Differential<T>::apply(const T sample)
{
	for (size_t i = 0; i < 4; i++)
		_samples[i] = _samples[i + 1];
	_samples[4] = sample;

	return (-_samples[4] + 8 * _samples[3] - 8 * _samples[1] + _samples[0]) / 12;
}

template <class T>
void Differential<T>::reset()
{
	memset(_samples, 0, sizeof(_samples));
}

template class Differential<int>;
template class Differential<float>;
