#ifndef __COMMON_H__
#define __COMMON_H__

#include <Arduino.h>

extern uint16_t crc16(uint8_t *puchMsg, uint16_t usDataLen);

template <typename T>
T Data_Filter(const T new_data, T * const data_array, 
				const uint8_t length, const uint8_t cut_len)
{
	uint8_t i, j;
	T valid_num = 0;
	T sum_data = 0;
	T *data_cache;

	data_cache = new T[length];

	for (i = 0; i < length - 1; i++)
	{
		data_array[i] = data_array[i + 1];
		data_cache[i] = data_array[i];
	}
	data_array[i] = new_data;
	data_cache[i] = data_array[i];

	T data_temp;
	for (i = 0; i < length - 1; i++)
	{
		for(j = i + 1; j < length; j++)
		{
			if(data_cache[i] > data_cache[j])
			{
				data_temp = data_cache[i];
				data_cache[i] = data_cache[j];
				data_cache[j] = data_temp;
			}
		}
	}
	
	//计算有效值和
	for(i = cut_len; i < length - cut_len; i++)
	{
		sum_data += data_cache[i];
	}

	delete[] data_cache;
	//取平均值
	return sum_data / (length - cut_len);
}

#endif 