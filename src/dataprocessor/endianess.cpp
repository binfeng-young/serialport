//
// Created by binfeng.yang on 2018/12/10.
//
#include "cl_endian.h"


void Endian::swap(void *data, int type_size, int total_times)
{
	if (type_size == 1) return;

	unsigned char *d = (unsigned char *)data;

	for (int j = 0; j < total_times; j++)
	{
		for (int i = 0; i < type_size / 2; i++)
		{
			unsigned char a = d[i];
			d[i] = d[type_size - 1 - i];
			d[type_size - 1 - i] = a;
		}

		d += type_size;
	}
}

bool Endian::is_system_big()
{
	const int i = 1;
	return !(*(char *)(&i));
}

bool Endian::is_system_64bit()
{
	return (sizeof(int*) == 8);
}
