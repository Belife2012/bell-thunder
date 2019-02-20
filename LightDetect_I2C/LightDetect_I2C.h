#ifndef _LIGHTDETECT_I2C_
#define _LIGHTDETECT_I2C_

class LIGHTDETECT_I2C
{
public:
  LIGHTDETECT_I2C(int slave_address);

  byte Set_Led_Brightness(byte bright_level);
  byte Set_Operate_Mode(byte optMode);
  void Set_Dark_Value(float new_value);
  void Set_Bright_Value(float new_value);
  void Reset_All_Value();
  float Get_Light_Value();

  byte Get_Light_Value(float *readValue);
private:
  byte _device_address;
  float dark_value = 0;
  float bright_value = 100;
  byte write(unsigned char memory_address, unsigned char *data, unsigned char size);
  byte read(unsigned char memory_address, unsigned char *data, unsigned char size);
};

#endif
