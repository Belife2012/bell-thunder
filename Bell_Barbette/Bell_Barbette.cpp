#include "Bell_Barbette.h"
#include "Task_Mesg.h"

int currrent_pos = 900;
unsigned long last_shooting_time = 0;

Bell_Barbette::Bell_Barbette()
{
    
}

void Bell_Barbette::Begin()
{
    /* Wire.begin();
    Wire.beginTransmission(0x70);
    Wire.write(0xff);
    Wire.endTransmission(); */
}

uint8_t Bell_Barbette::compute_crc(uint8_t *buf,uint8_t num)
{
    uint8_t crc = 0;
    for(uint8_t i = 0;i < num;i++)
    {
        crc += buf[i];
    }
    return crc;
}


uint8_t Bell_Barbette::send_data(uint8_t *data)
{
    uint8_t length = 0;
    uint8_t status = 0;
    uint8_t retry = 0;
    length = data[2] + 4;
    data[length - 1] = compute_crc(data,length - 1);
    
  Task_Mesg.Take_Semaphore_IIC();
    Wire.beginTransmission(BARBETTE_ADDR);
    Wire.write(data,length);
    status = Wire.endTransmission();
  Task_Mesg.Give_Semaphore_IIC();
    // while(status != 0)
    // {
    //     Wire.reset();
    //     Wire.beginTransmission(BARBETTE_ADDR);
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

void Bell_Barbette::receive_data(uint8_t *data,uint8_t num)
{
    uint8_t count = 0;
    Wire.requestFrom(BARBETTE_ADDR, num);
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

void Bell_Barbette::Open_Fire()
{
    static uint8_t shoot_buf[5] = {0xa5,0x01,0x01,0xff,0x00};
    send_data(shoot_buf);
}

void Bell_Barbette::Stop_Fire()
{
    static uint8_t stop_buf[5] = {0xa5,0x01,0x01,0x00,0x00};
    send_data(stop_buf);
}

uint8_t Bell_Barbette::Fire_With_Mode(uint8_t mode)
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

uint8_t Bell_Barbette::Fire_Onece(uint8_t mode)
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

void Bell_Barbette::Supply_On()
{
    static uint8_t supply_buf[5] = {0xa5,0x02,0x01,0xff,0x00};
    send_data(supply_buf);
}

void Bell_Barbette::Supply_Off()
{
    static uint8_t supply_buf[5] = {0xa5,0x02,0x01,0x00,0x00};
    send_data(supply_buf);
}

void Bell_Barbette::Supply_With_Rate(uint8_t rate)
{
    static uint8_t supply_buf[5] = {0xa5,0x01,0x01,0x10,0xb7};
    supply_buf[3] = rate;
    for(uint8_t i = 0;i < 4;i++)
    {
        supply_buf[4] += supply_buf[i];
    }
    send_data(supply_buf);
}

uint16_t Bell_Barbette::Get_Bullet()
{
    uint16_t bullet = 0;
    uint8_t crc = 0;
    uint8_t temp[6];
    uint8_t read_bullet_buf[2] = {0xea,0x02};

  Task_Mesg.Take_Semaphore_IIC();
    Wire.beginTransmission(BARBETTE_ADDR);
    Wire.write(read_bullet_buf,2);
    Wire.endTransmission(false);
    receive_data(temp,2);
  Task_Mesg.Give_Semaphore_IIC();
    bullet = temp[0];
    bullet = (bullet << 8) | temp[1];
    return bullet;
}

uint32_t Bell_Barbette::Get_Used_Time()
{
    uint32_t used_time = 0;
    uint8_t crc = 0;
    uint8_t temp[4];
    uint8_t read_bullet_buf[2] = {0xea,0x01};
    
  Task_Mesg.Take_Semaphore_IIC();
    Wire.beginTransmission(BARBETTE_ADDR);
    Wire.write(read_bullet_buf,2);
    Wire.endTransmission(false);
    receive_data(temp,4);
  Task_Mesg.Give_Semaphore_IIC();
    used_time = temp[0];
    used_time = (used_time << 8) | temp[1];
    used_time = (used_time << 8) | temp[2];
    used_time = (used_time << 8) | temp[3];
    return used_time;
}

void Bell_Barbette::Set_Barbette_Pos(uint8_t pos)
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

void Bell_Barbette::Clear_Bullet()
{
    static uint8_t clear_buf[5] = {0xa5,0x05,0x01,0x00,0xab};
    send_data(clear_buf);
}

void Bell_Barbette::Auto_Fire(uint8_t mode)
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
        Fire_With_Mode(mode - 1);
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

void Bell_Barbette::Adjust_Pos(uint8_t value)
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