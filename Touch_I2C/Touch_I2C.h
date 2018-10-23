#ifndef _TOUCH_I2C_
#define _TOUCH_I2C_

#define TOUCH_ADDRESS_RGB     0x01
#define TOUCH_ADDRESS_VALUE   0x02

class TOUCH_I2C
{
public:
    TOUCH_I2C(int slave_address);                // 配置I2C地址

    byte Get_Status(byte *readValue);
    byte Set_LED_RGBvalue(byte RedValue, byte GreenValue, byte BlueValue);

private:
    int _device_address;
    byte write(unsigned char memory_address, unsigned char *data, unsigned char size);
    byte read(unsigned char memory_address, unsigned char *data, int size);
};

#endif
