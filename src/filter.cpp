#include "filter.h"

template <class T>
SlideFilter<T>::SlideFilter(uint16_t size)
{
	_size = size;
	_buffer = (T*)malloc(sizeof(T) * _size);
	if (_buffer == NULL) {
		_size = 0;
		return;
	}
	reset();
}

template <class T>
SlideFilter<T>::~SlideFilter()
{
	if (_buffer != NULL)
		free(_buffer);
}

template <class T>
T SlideFilter<T>::apply(const T sample)
{
	if (_buffer == NULL || _size == 0)
		return sample;

	T sum = 0;

	_indx++;
	_indx %= _size;
	_buffer[_indx] = sample;

	for (size_t i = 0; i < _size; i++)
		sum += _buffer[i];

	return (sum / _size);
}

template <class T>
void SlideFilter<T>::reset(void)
{
	if (_buffer != NULL && _size > 0)
		memset(_buffer, 0, sizeof(T) * _size);
}

template class SlideFilter<int>;
template class SlideFilter<float>;

template <class T>
LowpassFilter<T>::LowpassFilter(float factor)
{
	_factor = factor;
	_prev = 0;
}

template <class T>
T LowpassFilter<T>::apply(const T sample)
{
	T out;

	out = sample * (_factor) + _prev * (1-_factor);
	_prev = out;

	return out;
}

template class LowpassFilter<int>;
template class LowpassFilter<float>;
