#include "iic_thunder.h"

#define IIC_PORTNONE   0x20 // 屏幕一直选通
#define IIC_PORT1   0x21
#define IIC_PORT2   0x22
#define IIC_PORT3   0x24
#define IIC_PORT4   0x60
#define IIC_PORTB   0x28
#define IIC_PORTA   0x30
#define IIC_PORTALL 0x7f // 全部选通是不包括 port4

#define IIC_SELECTOR_ADDR   0x70 // TCA9548的地址是 0x70, 因为它的地址位A0 A1 A2都接地了

uint8_t SENSOR_IIC::i2c_channel = 0;
bool SENSOR_IIC::i2c_enable = false;
SemaphoreHandle_t SENSOR_IIC::xSemaphore_IIC;

SENSOR_IIC::SENSOR_IIC(int slave_address)
{
  if(SENSOR_IIC::i2c_enable == false) {
    Wire.begin(SDA_PIN, SCL_PIN, 100000); //Wire.begin();
    SENSOR_IIC::CreateSemaphoreIIC();
    SENSOR_IIC::Select_Sensor_AllChannel();
    i2c_enable = true;
  }
  _device_address = (byte)slave_address;
}

/**
 * @brief: 如果channel=0xff,则选通 1/2/3/A/B，4没有选通；
 * channel=0,仅仅选通屏幕，屏幕接口是一直选通的；
 * 选通 1/2/3/4/A/B 时，只能选通一个接口；
 * 
 * @param channel:
 */
inline void SENSOR_IIC::SELECT_IIC_CHANNEL(uint8_t channel) 
{
  Wire.beginTransmission(IIC_SELECTOR_ADDR);
  switch (channel)
  {
    case 0: Wire.write(IIC_PORTNONE);
            break;
    case 1: Wire.write(IIC_PORT1);
            break;
    case 2: Wire.write(IIC_PORT2);
            break;
    case 3: Wire.write(IIC_PORT3);
            break;
    case 4: Wire.write(IIC_PORT4);
            break;
    case 5: Wire.write(IIC_PORTA);
            break;
    case 6: Wire.write(IIC_PORTB);
            break;
    default:Wire.write(IIC_PORTALL & channel);
            break;
  }
  Wire.endTransmission(true);

  i2c_channel = channel;
}

/**
 * @brief: I2C通讯，发送；返回 0 表示成功完成，非零表示没有成功
 * 
 * @param memory_address:
 * @param data:
 * @param size:
 * @param channel:channel=0时，通道选择不发生改变，通道的开启依据Select_Sensor_Channel的设置
 * @return byte :
 */
byte SENSOR_IIC::write(unsigned char memory_address,const unsigned char *data, unsigned char size, unsigned char channel)
{
  byte rc;
  TwoWire *p_iic;

  Take_Semaphore_IIC();
  if(channel > 0 && i2c_channel != channel){
    SELECT_IIC_CHANNEL(channel);
  }
  p_iic = &Wire;

  p_iic->beginTransmission(_device_address);
  p_iic->write(memory_address);
  if(data != NULL && size != 0){
    p_iic->write(data, size);
  }
  rc = p_iic->endTransmission();
  #ifdef COMPATIBILITY_OLD_ESP_LIB
  if (rc == I2C_ERROR_BUSY)
  {
    p_iic->reset();
  }
  #endif

  Give_Semaphore_IIC();

  return (rc);
}

byte SENSOR_IIC::read(unsigned char memory_address, unsigned char *data, unsigned char size, unsigned char channel)
{
  byte rc;
  unsigned char cnt = 0;
  TwoWire *p_iic;

  Take_Semaphore_IIC();
  if(channel > 0 && i2c_channel != channel){
    SELECT_IIC_CHANNEL(channel);
  }
  p_iic = &Wire;

  p_iic->beginTransmission(_device_address); // 开启发送
  p_iic->write(memory_address);
  rc = p_iic->endTransmission(false); // 结束发送  无参数发停止信息，参数0发开始信息 //返回0：成功，1：溢出，2：NACK，3，发送中收到NACK
  if (!(rc == 0 || rc == 7))
  {
    #ifdef COMPATIBILITY_OLD_ESP_LIB
    if (rc == I2C_ERROR_BUSY){
      p_iic->reset();
    }
    #endif
    Give_Semaphore_IIC();
    return rc;
  }

  cnt = p_iic->requestFrom(_device_address, size, (byte)true);
  if( 0 != cnt ){
    cnt = 0;
    while (p_iic->available() && cnt < size)
    {
      data[cnt] = p_iic->read();
      cnt++;
    }
  }
  
  Give_Semaphore_IIC();

  return (cnt != 0) ? 0 : 0xff;
}

/*
 * 选择传感器通道1/2/3/4/5(A)/6(B)，选择后只有当前通道可使用，可用多个相同模块
 * sensorChannel=0xff时，选通1/2/3/A/B
 * 
 * @parameter：需要使用的传感器接口号
 * @return: 设置成功返回0，发生错误返回非0 的错误码
 */
uint8_t SENSOR_IIC::Select_Sensor_Channel(uint8_t sensorChannel)
{
  uint8_t ret = 0;

  if(i2c_channel != sensorChannel)
  {
    Take_Semaphore_IIC();
    SELECT_IIC_CHANNEL(sensorChannel);
    Give_Semaphore_IIC();
  }

  return 0;
}

/* 
 * 选通所有传感器通道，
 * 初始化时一定要调用后才能使用屏幕等等IIC接口的模块。
 * 调用后，不能使用多个相同模块。
 * 默认PORT4是作为巡线传感器， 而不是作为IIC接口的
 * 
 * @parameter:
 * @return: 0 表示操作成功， 1 为初始化 IIC 失败
 */
uint8_t SENSOR_IIC::Select_Sensor_AllChannel()
{
  // reset
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(1);
  digitalWrite(15, HIGH);
  delay(5);

  // 
  Select_Sensor_Channel(0x80 | 0x3f); // bit7置位，说明这个channel参数是多通道开启

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

void SENSOR_IIC::Set_Port4_IIC(bool setting)
{
  if(setting == true) {
    Select_Sensor_Channel(4);
    Serial.println("Enable port4 IIC");
  } else {
    // ir_init标志是否为巡线传感器的初始化，如果是巡线传感器的初始化过程，需要执行设置
    if(i2c_channel == 4) {
      Select_Sensor_Channel(0);
      Serial.println("Disable port4 IIC");
    }
  }
}