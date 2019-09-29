#include <string.h>
#include "byte_convertion.h"

union{
  uint8_t byteVal[8];
  double doubleVal;
}val8bytes;

union{
    uint8_t byteVal[4];
    float floatVal;
    long longVal;
}val4bytes;

union{
  uint8_t byteVal[2];
  short shortVal;
}val2bytes;

union{
  uint8_t byteVal[1];
  uint8_t charVal;
}val1bytes;


void byte_convertion_init(BYTE_CONVERTION* byte_convertion, uint8_t* buf, uint32_t buf_size, uint32_t start_index)
{
	byte_convertion->buf = buf;
	byte_convertion->buf_size = buf_size;
	byte_convertion->index = start_index;
}

bool byte_convertion_read_byte(BYTE_CONVERTION* byte_convertion, int8_t* data)
{
  uint8_t idx;

	if (byte_convertion->index + 1 > byte_convertion->buf_size)
	{
		return false;
	}
  
  idx = byte_convertion->index;
  val1bytes.byteVal[0] = byte_convertion->buf[idx];
  
	byte_convertion->index = idx + 1;
  
  *data = val1bytes.charVal;
  
  return true;
}

bool byte_convertion_read_ubyte(BYTE_CONVERTION* byte_convertion, uint8_t* data)
{
	uint8_t idx;

	if (byte_convertion->index + 1 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val1bytes.byteVal[0] = byte_convertion->buf[idx];

	byte_convertion->index = idx + 1;

	*data = val1bytes.charVal;

	return true;
}

bool byte_convertion_read_short(BYTE_CONVERTION* byte_convertion, int16_t* data)
{
	uint8_t idx;

	if (byte_convertion->index + 2 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val2bytes.byteVal[0] = byte_convertion->buf[idx];
	val2bytes.byteVal[1] = byte_convertion->buf[idx+1];

	byte_convertion->index = idx + 2;

	*data = val2bytes.shortVal;

	return true;
}

bool byte_convertion_read_ushort(BYTE_CONVERTION* byte_convertion, uint16_t* data)
{
	uint8_t idx;

	if (byte_convertion->index + 2 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val2bytes.byteVal[0] = byte_convertion->buf[idx];
	val2bytes.byteVal[1] = byte_convertion->buf[idx + 1];

	byte_convertion->index = idx + 2;

	*data = val2bytes.shortVal;

	return true;
}

bool byte_convertion_read_int(BYTE_CONVERTION* byte_convertion, int32_t* data)
{
	uint8_t idx;

	if (byte_convertion->index + 4 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val4bytes.byteVal[0] = byte_convertion->buf[idx];
	val4bytes.byteVal[1] = byte_convertion->buf[idx + 1];
	val4bytes.byteVal[2] = byte_convertion->buf[idx + 2];
	val4bytes.byteVal[3] = byte_convertion->buf[idx + 3];

	byte_convertion->index = idx + 4;

	*data = val4bytes.longVal;

	return true;
}

bool byte_convertion_read_uint(BYTE_CONVERTION* byte_convertion, uint32_t* data)
{
	uint8_t idx;

	if (byte_convertion->index + 4 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val4bytes.byteVal[0] = byte_convertion->buf[idx];
	val4bytes.byteVal[1] = byte_convertion->buf[idx + 1];
	val4bytes.byteVal[2] = byte_convertion->buf[idx + 2];
	val4bytes.byteVal[3] = byte_convertion->buf[idx + 3];

	byte_convertion->index = idx + 4;

	*data = val4bytes.longVal;

	return true;
}

bool byte_convertion_read_float(BYTE_CONVERTION* byte_convertion, float* data)
{
	uint8_t idx;

	if (byte_convertion->index + 4 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val4bytes.byteVal[0] = byte_convertion->buf[idx];
	val4bytes.byteVal[1] = byte_convertion->buf[idx + 1];
	val4bytes.byteVal[2] = byte_convertion->buf[idx + 2];
	val4bytes.byteVal[3] = byte_convertion->buf[idx + 3];

	byte_convertion->index = idx + 4;

	*data = val4bytes.floatVal;

	return true;
}

bool byte_convertion_read_buf(BYTE_CONVERTION* byte_convertion, uint8_t* buf, uint32_t size)
{
	uint8_t idx;

	if (byte_convertion->index + size > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;

	memcpy(buf, &byte_convertion->buf[idx], size);

	byte_convertion->index = idx + size;

	return true;
}










bool byte_convertion_write_byte(BYTE_CONVERTION* byte_convertion, int8_t data)
{
	uint8_t idx;

	if (byte_convertion->index + 1 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	byte_convertion->buf[idx] = data;

	byte_convertion->index = idx + 1;

	return true;
}

bool byte_convertion_write_ubyte(BYTE_CONVERTION* byte_convertion, uint8_t data)
{
	uint8_t idx;

	if (byte_convertion->index + 1 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	byte_convertion->buf[idx] = data;

	byte_convertion->index = idx + 1;

	return true;
}

bool byte_convertion_write_short(BYTE_CONVERTION* byte_convertion, int16_t data)
{
	uint8_t idx;

	if (byte_convertion->index + 2 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val2bytes.shortVal = data;

	byte_convertion->buf[idx] = val2bytes.byteVal[0];
	byte_convertion->buf[idx+1] = val2bytes.byteVal[1];

	byte_convertion->index = idx + 2;

	return true;
}

bool byte_convertion_write_ushort(BYTE_CONVERTION* byte_convertion, uint16_t data)
{
	uint8_t idx;

	if (byte_convertion->index + 2 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val2bytes.shortVal = data;

	byte_convertion->buf[idx] = val2bytes.byteVal[0];
	byte_convertion->buf[idx + 1] = val2bytes.byteVal[1];

	byte_convertion->index = idx + 2;

	return true;
}

bool byte_convertion_write_int(BYTE_CONVERTION* byte_convertion, int32_t data)
{
	uint8_t idx;

	if (byte_convertion->index + 4 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val4bytes.longVal = data;

	byte_convertion->buf[idx] = val4bytes.byteVal[0];
	byte_convertion->buf[idx + 1] = val4bytes.byteVal[1];
	byte_convertion->buf[idx + 2] = val4bytes.byteVal[2];
	byte_convertion->buf[idx + 3] = val4bytes.byteVal[3];

	byte_convertion->index = idx + 4;

	return true;
}

bool byte_convertion_write_uint(BYTE_CONVERTION* byte_convertion, uint32_t data)
{
	uint8_t idx;

	if (byte_convertion->index + 4 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val4bytes.longVal = data;

	byte_convertion->buf[idx] = val4bytes.byteVal[0];
	byte_convertion->buf[idx + 1] = val4bytes.byteVal[1];
	byte_convertion->buf[idx + 2] = val4bytes.byteVal[2];
	byte_convertion->buf[idx + 3] = val4bytes.byteVal[3];

	byte_convertion->index = idx + 4;

	return true;
}

bool byte_convertion_write_float(BYTE_CONVERTION* byte_convertion, float data)
{
	uint8_t idx;

	if (byte_convertion->index + 4 > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;
	val4bytes.floatVal = data;

	byte_convertion->buf[idx] = val4bytes.byteVal[0];
	byte_convertion->buf[idx + 1] = val4bytes.byteVal[1];
	byte_convertion->buf[idx + 2] = val4bytes.byteVal[2];
	byte_convertion->buf[idx + 3] = val4bytes.byteVal[3];

	byte_convertion->index = idx + 4;

	return true;
}

bool byte_convertion_write_buf(BYTE_CONVERTION* byte_convertion, uint8_t* buf, uint32_t size)
{
	uint8_t idx;

	if (byte_convertion->index + size > byte_convertion->buf_size)
	{
		return false;
	}

	idx = byte_convertion->index;

	memcpy(&byte_convertion->buf[idx], buf, size);

	byte_convertion->index = idx + size;

	return true;
}


