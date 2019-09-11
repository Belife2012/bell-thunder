#include "bell_barbette.h"
#include "iic_thunder.h"

int currrent_pos = 900;
unsigned long last_shooting_time = 0;
int last_pos = 90;

BELL_BARBETTE::BELL_BARBETTE()
{}

void BELL_BARBETTE::Begin()
{
    /*Wire.begin();
    Wire.beginTransmission(0x70);
    Wire.write(0xff);
    Wire.endTransmission(); */
}

uint8_t BELL_BARBETTE::compute_crc(uint8_t *buf,uint8_t num)
{
    uint8_t crc = 0;
    for(uint8_t i = 0;i < num;i++)
    {
        crc += buf[i];
    }
    return crc;
}


uint8_t BELL_BARBETTE::send_data(uint8_t *data)
{
    uint8_t length = 0;
    uint8_t status = 0;
    uint8_t retry = 0;
    length = data[2] + 4;
    data[length - 1] = compute_crc(data,length - 1);
    
  SENSOR_IIC::Take_Semaphore_IIC();
    Wire.beginTransmission((uint8_t)BARBETTE_ADDR);
    Wire.write(data,length);
    status = Wire.endTransmission();
  SENSOR_IIC::Give_Semaphore_IIC();
    // while(status != 0)
    // {
    //     Wire.reset();
    //     Wire.beginTransmission((uint8_t)(uint8_t)BARBETTE_ADDR);
    //     Wire.write(data,length);
    //     status = Wire.endTransmission();
    //     retry++;
    //     if(retry > 10)
    //     {
    //         break;
    //     }
    // }
    return status;
}

void BELL_BARBETTE::receive_data(uint8_t *data,uint8_t num)
{
    uint8_t count = 0;
    uint8_t erro = 0;
    erro = Wire.requestFrom((uint8_t)(uint8_t)BARBETTE_ADDR, num);
    while(Wire.available() > 0)
    {
        data[count] = Wire.read();
        count++;
        if(count > (num - 1))
        {
            break;
        }
    }
}

void BELL_BARBETTE::Open_Fire()
{
    static uint8_t shoot_buf[5] = {0xa5,0x01,0x01,0xff,0x00};
    send_data(shoot_buf);
}

void BELL_BARBETTE::Stop_Fire()
{
    static uint8_t stop_buf[5] = {0xa5,0x01,0x01,0x00,0x00};
    send_data(stop_buf);
}

uint8_t BELL_BARBETTE::Fire_With_Mode(uint8_t mode)
{
    static uint8_t shoot_buf[5] = {0xa5,0x01,0x01,0x10,0xb7};
    uint8_t status = 0;
    shoot_buf[3] = mode;
    for(uint8_t i = 0;i < 4;i++)
    {
        shoot_buf[4] += shoot_buf[i];
    }
    status = send_data(shoot_buf);
    return status;
}

uint8_t BELL_BARBETTE::Fire_Onece(uint8_t mode)
{
    static uint8_t shoot_onece[5] = {0xa5,0x04,0x01,0x01,0xab};
    static uint8_t shoot_close[5] = {0xa5,0x04,0x01,0x00,0xaa};
    uint8_t status = 0;
    if(mode > 0)
    {
        status = send_data(shoot_onece);
    }
    else
    {
        status = send_data(shoot_close);
    }
    return status;
}

uint16_t BELL_BARBETTE::Get_Bullet()
{
    uint16_t bullet = 0;
    uint8_t crc = 0;
    uint8_t temp[6];
    uint8_t read_bullet_buf[2] = {0xea,0x02};

  SENSOR_IIC::Take_Semaphore_IIC();
    Wire.beginTransmission((uint8_t)BARBETTE_ADDR);
    Wire.write(read_bullet_buf,2);
    Wire.endTransmission(false);
    receive_data(temp,2);
  SENSOR_IIC::Give_Semaphore_IIC();
    bullet = temp[0];
    bullet = (bullet << 8) | temp[1];
    return bullet;
}

uint32_t BELL_BARBETTE::Get_Used_Time()
{
    uint32_t used_time = 0;
    uint8_t crc = 0;
    uint8_t temp[4];
    uint8_t read_bullet_buf[2] = {0xea,0x01};
    
  SENSOR_IIC::Take_Semaphore_IIC();
    Wire.beginTransmission((uint8_t)BARBETTE_ADDR);
    Wire.write(read_bullet_buf,2);
    Wire.endTransmission(false);
    receive_data(temp,4);
  SENSOR_IIC::Give_Semaphore_IIC();
    used_time = temp[0];
    used_time = (used_time << 8) | temp[1];
    used_time = (used_time << 8) | temp[2];
    used_time = (used_time << 8) | temp[3];
    return used_time;
}

void BELL_BARBETTE::Set_Barbette_Pos(uint8_t pos)
{
    static uint8_t pos_buf[5] = {0xa5,0x03,0x01,0x00,0x00};
    if(pos > 180)
    {
        pos = 180;
    }
    pos_buf[3] = pos;
    if((millis() - last_shooting_time) > 30)
    {
        send_data(pos_buf);
        // Wire.reset();
    }
}

void BELL_BARBETTE::Clear_Bullet()
{
    static uint8_t clear_buf[5] = {0xa5,0x05,0x01,0x00,0xab};
    send_data(clear_buf);
}

void BELL_BARBETTE::Auto_Fire(uint8_t mode)
{
    static bool is_press = 0; 
    static bool is_onece_mode = 0; 

    if((mode == 1) && (is_press == 0))
    {
        last_shooting_time = millis();
        Fire_Onece(1);
        is_press = 1;
        is_onece_mode = 1;
    }
    else if((mode == 2) || (mode == 3))
    {
        last_shooting_time = millis();
        Fire_With_Mode(mode);
        is_onece_mode = 0;
    }
    if(mode == 0)
    {
      if(is_onece_mode == 1)
      {
         is_press = 0;
//         Fire_Onece(0);
      }
      else
      {
         Fire_With_Mode(mode);
         last_shooting_time = millis();
      }
    }
}

void BELL_BARBETTE::Adjust_Pos(uint8_t value)
{
   currrent_pos -= (value - 126) / 6;
   if(currrent_pos > 1750)
   {
      currrent_pos = 1750;
   }
   else if(currrent_pos < 10)
   {
      currrent_pos = 10;
   }
   Set_Barbette_Pos(180 - (currrent_pos / 10));
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void BELL_BARBETTE::Fire_Control(int pos,uint8_t mode)
{
    static uint8_t control_buf[6] = {0xa5,0x06,0x02,0x00,0x00,0x00};
    
    if(pos > 100)
    {
        pos = 100;
    }
    else if(pos < -100)
    {
        pos = -100;
    }
    pos = map(pos,-100,100,-70,70);
    if(mode > 3)
    {
        mode = 0;
    }
    control_buf[3] = pos + 90;
    control_buf[4] = mode;
    send_data(control_buf);
}

void BELL_BARBETTE::Fire_Control(int pos,uint8_t mode,uint8_t time)
{
    static uint8_t control_buf[7] = {0xa5,0x07,0x03,0x00,0x00,0x00,0x00};
    
    if(pos > 100)
    {
        pos = 100;
    }
    else if(pos < -100)
    {
        pos = -100;
    }
    pos = map(pos,-100,100,-70,70);
    if(mode > 3)
    {
        mode = 0;
    }
    control_buf[3] = pos + 90;
    control_buf[4] = mode;
    control_buf[5] = time;
    send_data(control_buf);
}

uint16_t BELL_BARBETTE::Thunder_Battery_Get(uint8_t sensorChannel)
{
    // SENSOR_IIC::Select_Sensor_Channel(sensorChannel);
    uint16_t value;
    value = Get_Bullet();
    return value;
}
