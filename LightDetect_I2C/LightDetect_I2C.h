#ifndef _LIGHTDETECT_I2C_
#define _LIGHTDETECT_I2C_

class LIGHTDETECT_I2C
{
public:
  LIGHTDETECT_I2C(int slave_address);

  byte Set_Operate_Mode(byte optMode);
  byte Get_Light_Value(float *readValue);

private:
  byte _device_address;
  byte write(unsigned char memory_address, unsigned char *data, unsigned char size);
  byte read(unsigned char memory_address, unsigned char *data, unsigned char size);
};

#endif
