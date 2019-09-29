#ifndef __BYTE_CONVERTION_H__
#define __BYTE_CONVERTION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct _BYTE_CONVERTION
{
  uint8_t *buf;
  uint32_t buf_size;
  uint32_t index;
}BYTE_CONVERTION;

void byte_convertion_init(BYTE_CONVERTION* byte_convertion, uint8_t* buf, uint32_t buf_size, uint32_t start_index);

bool byte_convertion_read_byte(BYTE_CONVERTION* byte_convertion, int8_t* data);
bool byte_convertion_read_ubyte(BYTE_CONVERTION* byte_convertion, uint8_t* data);
bool byte_convertion_read_short(BYTE_CONVERTION* byte_convertion, int16_t* data);
bool byte_convertion_read_ushort(BYTE_CONVERTION* byte_convertion, uint16_t* data);
bool byte_convertion_read_int(BYTE_CONVERTION* byte_convertion, int32_t* data);
bool byte_convertion_read_uint(BYTE_CONVERTION* byte_convertion, uint32_t* data);
bool byte_convertion_read_float(BYTE_CONVERTION* byte_convertion, float* data);
bool byte_convertion_read_buf(BYTE_CONVERTION* byte_convertion, uint8_t* buf, uint32_t size);

bool byte_convertion_write_byte(BYTE_CONVERTION* byte_convertion, int8_t data);
bool byte_convertion_write_ubyte(BYTE_CONVERTION* byte_convertion, uint8_t data);
bool byte_convertion_write_short(BYTE_CONVERTION* byte_convertion, int16_t data);
bool byte_convertion_write_ushort(BYTE_CONVERTION* byte_convertion, uint16_t data);
bool byte_convertion_write_int(BYTE_CONVERTION* byte_convertion, int32_t data);
bool byte_convertion_write_uint(BYTE_CONVERTION* byte_convertion, uint32_t data);
bool byte_convertion_write_float(BYTE_CONVERTION* byte_convertion, float data);
bool byte_convertion_write_buf(BYTE_CONVERTION* byte_convertion, uint8_t* buf, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif
