#include "Sensor_IIC.h"

#define SELECT_IIC_CHANNEL(channel) do{ Wire.beginTransmission(0x70); \
                                      Wire.write( 0x38 | ( (channel&0x07)==0x07 ? 0x07 : (0x01 << (channel-1)) ) ); \
                                      Wire.endTransmission(true); }while(0)


uint8_t SENSOR_IIC::i2c_channel = 0;
SemaphoreHandle_t SENSOR_IIC::xSemaphore_IIC;

SENSOR_IIC::SENSOR_IIC(int slave_address)
{
  _device_address = (byte)slave_address;
}

// 类内部使用，I2C通讯，发送；返回 0 表示成功完成，非零表示没有成功
byte SENSOR_IIC::write(unsigned char memory_address,const unsigned char *data, unsigned char size, unsigned char channel)
{
  byte rc;

  Take_Semaphore_IIC();
  if(channel > 0 && i2c_channel != channel){
    SELECT_IIC_CHANNEL(channel);
    i2c_channel = channel;
  }
  Wire.beginTransmission(_device_address);
  Wire.write(memory_address);
  if(data != NULL && size != 0){
    Wire.write(data, size);
  }
  rc = Wire.endTransmission();
  #ifdef COMPATIBILITY_OLD_ESP_LIB
  if (rc == I2C_ERROR_BUSY)
  {
    Wire.reset();
  }
  #endif
  Give_Semaphore_IIC();
  return (rc);
}

// 类内部使用，I2C通讯，发送并读取；返回值 非0 表示失败，其中0xFF表示没有读取到数据
byte SENSOR_IIC::read(unsigned char memory_address, unsigned char *data, unsigned char size, unsigned char channel)
{
  byte rc;
  unsigned char cnt = 0;

  Take_Semaphore_IIC();
  if(channel > 0 && i2c_channel != channel){
    SELECT_IIC_CHANNEL(channel);
    i2c_channel = channel;
  }
  Wire.beginTransmission(_device_address); // 开启发送
  Wire.write(memory_address);
  rc = Wire.endTransmission(false); // 结束发送  无参数发停止信息，参数0发开始信息 //返回0：成功，1：溢出，2：NACK，3，发送中收到NACK
  if (!(rc == 0 || rc == 7))
  {
    #ifdef COMPATIBILITY_OLD_ESP_LIB
    if (rc == I2C_ERROR_BUSY){
      Wire.reset();
    }
    #endif
    Give_Semaphore_IIC();
    return rc;
  }

  cnt = Wire.requestFrom(_device_address, size, (byte)true);
  if( 0 != cnt ){
    cnt = 0;
    while (Wire.available() && cnt < size)
    {
      data[cnt] = Wire.read();
      cnt++;
    }
  }
  Give_Semaphore_IIC();

  return (cnt != 0) ? 0 : 0xff;
}

/* 
 * I2C端口选通，变量channelData 相应位(每一bit代表一个通道) 为1 是 选通，可以多通道选通
 * 
 * @parameter: 
 * @return: 返回的是IIC 操作状态码，0 为成功， 非0 为其他状态
 */
uint8_t SENSOR_IIC::Set_I2C_Chanel(uint8_t channelData)
{
  uint8_t ret;
  uint8_t regValue;

  // 保证初始值与channelData 不一致
  regValue = (channelData == 0) ? 0xff : 0;
  // 重复连接 IIC 扩展芯片
  for (uint8_t i = 0; i < 2; i++)
  {
	// TCA9548的地址是 0x70, 因为它的地址位A0 A1 A2都接地了

	Take_Semaphore_IIC();
	Wire.beginTransmission(0x70);
	Wire.write(channelData);
	ret = Wire.endTransmission(true);
#ifdef COMPATIBILITY_OLD_ESP_LIB
	if (ret == I2C_ERROR_BUSY)
	{
	  Wire.reset();
	}
#endif
	Give_Semaphore_IIC();
	if (ret != 0)
	{
	  Serial.printf("### TCA9548 Write I2C Channel Error: %d \n", ret);
	  // delay(100);
	}
#if 0 // 每次的传感器操作都需要调用，为了速度，不能进行读写比较
	else
	{
	  // read TCA9548
	  Take_Semaphore_IIC();
	  if (0 != Wire.requestFrom((byte)0x70, (byte)1, (byte) true))
	  {
		while (Wire.available())
		{
		  regValue = Wire.read();
		}
	  }
	  Give_Semaphore_IIC();

	  if (regValue == channelData)
	  {
		break;
	  }
	  else
	  {
		Serial.println("### TCA9548 Read not equal Write");
		// delay(200);
	  }
	}
#endif
  }

  return ret;
}

/*
 * 选择传感器通道1/2/3，选择后只有当前通道可使用，可用多个相同模块
 * 
 * @parameter：需要使用的传感器接口号
 * @return: 设置成功返回0，发生错误返回非0 的错误码
 *          错误码1：没有相应的传感器端口号
 *          错误码2：硬件不支持选择传感器端口号
 */
uint8_t SENSOR_IIC::Select_Sensor_Channel(uint8_t sensorChannel)
{
  uint8_t ret;

  switch (sensorChannel)
  {
  case 0:
    ret = Set_I2C_Chanel(0x3f);
    break;
  case 1:
    ret = Set_I2C_Chanel(0x39);
    break;
  case 2:
    ret = Set_I2C_Chanel(0x3a);
    break;
  case 3:
    ret = Set_I2C_Chanel(0x3c);
    break;
  default:
    Serial.printf("### No Sensor Channel! ###");
    return 1;
    break;
  }

  i2c_channel = sensorChannel;
  if (ret != 0)
  {
	  return 2;
  }

  return 0;
}

/* 
 * 选通所有传感器通道，初始化时一定要调用后才能使用屏幕等等IIC接口的模块。
 *   调用后，不能使用多个相同模块。
 * 
 * @parameter:
 * @return: 0 表示操作成功， 1 为初始化 IIC 失败
 */
uint8_t SENSOR_IIC::Select_Sensor_AllChannel()
{
  uint8_t ret;

  // reset
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(1);
  digitalWrite(15, HIGH);
  delay(5);

  ret = Set_I2C_Chanel(0x3f); //全选通
  i2c_channel = 0;

  if (ret != 0)
  {
	  return 1;
  }

  return 0;
}

void SENSOR_IIC::CreateSemaphoreIIC()
{
  // 创建 IIC互斥体
  xSemaphore_IIC = xSemaphoreCreateMutex();
  if (xSemaphore_IIC == NULL)
  {
    while (1)
    {
      Serial.println("Mutex_IIC create fail");
    }
  }
}

/* 
 * 用于IIC硬件驱动，阻止其他线程占用IIC，恢复前 不建议使用delay
 * 
 * @parameters:
 * @return
 */
void SENSOR_IIC::Take_Semaphore_IIC()
{
  do
  {
  } while (xSemaphoreTake(xSemaphore_IIC, portMAX_DELAY) != pdPASS);
}
/* 
 * 用于硬件驱动，恢复其他线程使用IIC
 * 
 * @parameters:
 * @return
 */
void SENSOR_IIC::Give_Semaphore_IIC()
{
  xSemaphoreGive(xSemaphore_IIC);
}