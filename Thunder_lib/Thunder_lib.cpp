/************************************************
 * 
 * 公司：贝尔科教集团
 * 公司网站：https://www.bell.ai
 * 
 * 
 * 
 * 雷霆库文件
 * 
 *   创建日期： 20180606
 *   作者：     宋博伟
 *   邮箱：     songbw123@163.com
 *
 *   版本：     v0.2
 *   修改日期   20180721
 *   修改：     宋博伟
 *   邮箱：     songbw123@163.com
 *   修改内容： 
 * 
 *   
 * 
 * 功能列表：
 *  1.  void Setup_All(void);                           // 所有模块初始化
 *  2.  void Stop_All(void);                            // 全部终止(电机)
 *  3.  void Setup_Battery(void);                       // 电池电压检测初始化配置
 *  4.  float Get_Battery_Data(void);                   // 获取电池电压
 *  5.  void En_Motor(void);                            // 编码电机  闭环计算
 *  6.  void Enable_En_Motor(void);                     // 打开编码电机计算
 *  7.  void Disable_En_Motor(void);                    // 关闭编码电机计算
 *  8.  void Setup_Servo(void);                         // 舵机初始化配置
 *  9.  void Servo_Turn(int servo, int angle);          // 舵机角度控制
 *  10. void Setup_IR(void);                            // 巡线IR传感器初始化配置
 *  11. void Get_IR_Data(uint8_t data[]);               // 获取巡线IR数据
 *  12. void Line_Tracing(void);                        // 巡线模式
 *  13. void Start_Show(void);                          // 开机动画/声效
 *  14. void Wait_BLE(void);                            // 等待蓝牙连接动画 (有串口数据也跳出)
 *  15. void Set_LED_Show_No(uint8_t Show_No);          // 设置将要播放的内置动画编号
 *  16. void LED_Show(void);                            // 循环执行的内置动画控制程序
 *  17. void Check_Communication(void);                 // 通信确认，蓝牙/串口
 *  18. void Check_Protocol(void);                      // 协议解析
 *  19. void Reset_Rx_Data(void);                       // 清空接收数据
 *  20. void Get_Queue_Encoder(void);                   // 通过队列获取编码器数据
 *  21. uint8_t Set_I2C_Chanel(uint8_t channelData);    // 增加了TCA9548芯片，选通I2C端口，变量channelData相应位为1 是 选通
 * 
 * 
 ************************************************/

#include <Thunder_lib.h>
#include "esp_adc_cal.h"

// #define PRINT_UART_COMMAND
// #define DEBUG_LINE_TRACING

THUNDER Thunder;

THUNDER_BLE Thunder_BLE;
THUNDER_MOTOR Thunder_Motor;

//ADC 校准使用
esp_adc_cal_characteristics_t adc_chars;
esp_adc_cal_value_t cal_value_type;

// 音频
WT588 Speaker = WT588(AUDIO, BUSY); // 配置引脚16 --> AUDIO 和 4 --> BUSY

// 超声波
US_I2C US(ADD_I2C_US);

// 颜色识别
BH1745NUC Colour_Sensor(ADD_I2C_COLOUR); //I2C从机地址

// 彩色LED
XT1511_I2C I2C_LED(0x11);

// 单色色LED
DOT_MATRIX_LED Dot_Matrix_LED;

// 触碰传感器
TOUCH_I2C Touch_Sensor(TOUCH_ADDR_DEVICE);

// 光电传感器
LIGHTDETECT_I2C Light_Sensor(LIGHT_ADDR_DEVICE);

// 蓝牙
uint8_t Rx_Data[16] = {0};
uint8_t Tx_Data[16] = {0};
bool deviceConnected = false;

// 固件版本号 4 bytes, 第一个是区分测试版本和 正式版本的前缀
// 测试固件命名规则 Tx.x.x
// 正式发布固件命名 Vx.x.x
// 版本号第一位数字，发布版本具有重要功能修改
// 版本号第二位数字，当有功能修改和增减时，相应地递增
// 版本号第三位数字，每次为某个版本修复BUG时，相应地递增
const uint8_t Version_FW[4] = {'T', 0, 2, 36};
// const uint8_t Version_FW[4] = {0, 21, 0, 0};

// 所有模块初始化
void THUNDER::Setup_All(void)
{
  // delay(6000);

  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.printf("\n\n======\n");
  Serial.printf("\nFirmware VERSION : %c%d.%d.%d \n", 
                Version_FW[0], Version_FW[1], Version_FW[2], Version_FW[3]);

  Serial.printf("\nInitialize All modules of -Thunder Go- \n\n");

  Wire.begin(SDA_PIN, SCL_PIN, 100000); //Wire.begin();
  Select_Sensor_AllChannel();


  #ifndef DEBUG_LINE_TRACING
  Thunder_BLE.Setup_EEPROM(); // 配置EEPROM
  Thunder_BLE.Setup_BLE();    // 配置BLE
  #endif

  Colour_Sensor.Setup();           // 配置颜色传感器
  Thunder_Motor.Setup_Motor();     // 配置电机
  Thunder_Motor.Setup_Motor_PID(); // 配置左右两个电机编码器
  Enable_En_Motor();               // 打开编码电机计算

  Setup_Servo();          // 舵机初始化配置
  Setup_IR();             // 巡线IR传感器初始化配置
  Setup_Battery();        // 电池电压检测初始化配置

  Dot_Matrix_LED.Setup(); // 初始化单色LED驱动IC配置

  I2C_LED.LED_OFF(); // 彩灯全关，立即刷新

  Task_Mesg.Set_Flush_Task(FLUSH_MATRIX_LED); // 把 LED点阵显示动画效果刷新工作 交给后台守护线程进行
  Task_Mesg.Set_Flush_Task(FLUSH_COLOR_LED); // 把 彩灯刷新工作 交给后台守护线程进行
  Task_Mesg.Set_Flush_Task(FLUSH_MOTOR_PID_CTRL); // 把 电机闭环控制 交给后台守护线程进行
  Task_Mesg.Set_Flush_Task(FLUSH_CHARACTER_ROLL); // 把 滚动显示的刷新工作 交给后台守护线程进行
  Task_Mesg.Set_Flush_Task(FLUSH_BATTERY_MEASURE); // 每300ms检测一次电池电压，以保证每次测量都有之前的数据作为滤波数据
  Task_Mesg.Create_Deamon_Threads(); // 创建并开始 守护线程

  Serial.printf("\n*** Initial Completes ***\n\n");

  // 开机动画/声效
  Start_Show();
  
  Serial.printf( "Battery Vlotage: %fV\n", ((float)Get_Battery_Data()/1000) );
}

// 全部终止(电机)
void THUNDER::Stop_All(void)
{
  Serial.printf("Stop Motor... \n");

  Thunder_Motor.Set_L_Target(0); // 50ms 最大40  //编码器计数值
  Thunder_Motor.Set_R_Target(0);

  Disable_En_Motor(); // 关闭编码电机计算

  Thunder_Motor.Motor_Move(1, 0, 1); // 参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
  Thunder_Motor.Motor_Move(2, 0, 2); // 参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
}

// 电池电压检测初始化配置
void THUNDER::Setup_Battery()
{
  adc_power_on();
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_11db);

  if( ESP_OK == esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) ){
    Serial.println("\nsupport Efuse.");
  }else{
    Serial.println("\nNo Efuse.");
  }

  pinMode(BATTERY_ADC_PIN, INPUT);
  cal_value_type = esp_adc_cal_characterize(
                    ADC_UNIT_1, 
                    ADC_ATTEN_11db, 
                    ADC_WIDTH_12Bit, 
                    3300, 
                    &adc_chars);
  Serial.print("ADC Vref: ");
  if (cal_value_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.println("eFuse Vref");
  } else if (cal_value_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.println("Two Point");
  } else {
    Serial.println("Default");
  }
}

uint32_t THUNDER::Battery_Power_Filter(uint32_t new_data)
{
  uint32_t i, valid_num = 0;
  uint32_t sum_data = 0, max_data = 0, min_data = 15000;
  for( i = 0; i < BATTERY_FILTER_NUM - 1; i++){
    battery_filter_data[i] = battery_filter_data[i+1];
  }
  battery_filter_data[BATTERY_FILTER_NUM - 1] = new_data;

  for( i = 0; i < BATTERY_FILTER_NUM; i++){
    if( 3300 < battery_filter_data[i] && battery_filter_data[i] < 15000 ){
      valid_num++;
      if(battery_filter_data[i] < min_data){
        min_data = battery_filter_data[i];
      }
      if(max_data < battery_filter_data[i]){
        max_data = battery_filter_data[i];
      }
      sum_data += battery_filter_data[i];
    }
  }
  if(valid_num > 3){
    sum_data -= min_data;
    sum_data -= max_data;
    valid_num -= 2;
  }
  //取平均值
  if(valid_num == 0){
    return 0;
  }else{
    return sum_data/valid_num;
  }
}

// 获取电池电压
uint32_t THUNDER::Get_Battery_Data()
{
  uint32_t ADC_mV;
  uint32_t Battery_Voltage;

  ADC_mV = adc1_get_raw(ADC1_CHANNEL_7);
  // Serial.printf("\nADC : %d\n", ADC_mV);
  
  if(cal_value_type == ESP_ADC_CAL_VAL_DEFAULT_VREF){
    // efuse 的校准信息不存在，所以要手动计算原始ADC值
    ADC_mV = ADC_mV * 3300 / 4095;
  }else{
    esp_adc_cal_get_voltage(ADC_CHANNEL_7, &adc_chars, &ADC_mV);
  }
  // Serial.printf("ADC Voltage: %dmV\n", ADC_mV);
  Battery_Voltage = ADC_mV * (ADC_R_1 + ADC_R_2) / ADC_R_1;  // 分压电阻为：51k;100k

  // Filter
  Battery_Voltage = Battery_Power_Filter(Battery_Voltage);
  // Serial.printf("Bat Voltage: %dmV\n", Battery_Voltage);

  Indicate_Lowpower(Battery_Voltage);

  return Battery_Voltage;
}

/* 
 * 显示低电量 LED Matrix图案
 * 连续响三声 G5 声
 * 
 * @parameters: 
 * @return: 
 */
void THUNDER::Indicate_Lowpower(uint32_t Battery_Voltage)
{
  if(Battery_Voltage < 7000 && Battery_Voltage < (Battery_Power - 300)){
    lowpower_flag = 1;

    Dot_Matrix_LED.Play_LED_HT16F35B_Show(101);

    Speaker.Play_Song(79);
    delay(300);
    Speaker.Play_Song(79);
    delay(300);
    Speaker.Play_Song(79);

    Serial.printf("\nLow Power: %dmV\n", Battery_Voltage);
    Battery_Power = Battery_Voltage;
  }
}

// 编码电机  闭环计算
void THUNDER::En_Motor(void)
{
  if (En_Motor_Flag == 1)
  {
    if (xSemaphoreTake(Timer_PID_Flag, portMAX_DELAY) == pdTRUE) // 控制周期PID_dt[ms]
    {
      Thunder_Motor.PID_Speed();

      // Serial.printf("%d  %d  %d  %d  \n",Thunder_Motor.Get_L_Speed(),Thunder_Motor.Get_R_Speed(),Thunder_Motor.Get_L_Target(),Thunder_Motor.Get_R_Target());
      // Serial.printf("%d\n",Thunder_Motor.Get_L_Speed());
    }
  }
}

// 打开编码电机计算
void THUNDER::Enable_En_Motor(void)
{
  En_Motor_Flag = 1;
}

// 关闭编码电机计算
void THUNDER::Disable_En_Motor(void)
{
  En_Motor_Flag = 0;
}

// 舵机初始化配置
void THUNDER::Setup_Servo(void)
{
  ledcSetup(SERVO_CHANNEL_0, SERVO_BASE_FREQ, SERVO_TIMER_13_BIT);
  ledcSetup(SERVO_CHANNEL_1, SERVO_BASE_FREQ, SERVO_TIMER_13_BIT);

  ledcAttachPin(SERVO_A, SERVO_CHANNEL_0);
  ledcAttachPin(SERVO_B, SERVO_CHANNEL_1);
}

// 舵机角度控制
// 参数1 --> 舵机编号；1-->A口；2-->B口
// 参数2 --> 角度[°]；范围为0-180
void THUNDER::Servo_Turn(int servo, int angle)
{
  if (angle > 180)
  {
    angle = 180;
  }
  else if (angle < 0)
  {
    angle = 0;
  }

  if (servo == 1) //A口
  {
    ledcWrite(SERVO_CHANNEL_0, Servo_MIN + Servo_Range * angle / 180); //大的舵机，最小260，最大950
  }
  else if (servo == 2) //B口
  {
    ledcWrite(SERVO_CHANNEL_1, Servo_MIN + Servo_Range * angle / 180); //大的舵机，最小260，最大950
  }
  else
  {
    Serial.printf("### No needed Servo\n");
  }
}

// 巡线IR传感器初始化配置
void THUNDER::Setup_IR()
{
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
}

// 获取巡线IR数据
void THUNDER::Get_IR_Data(uint8_t data[])
{
  data[0] = digitalRead(IR_1);
  data[1] = digitalRead(IR_2);
}

void Wait_For_Motor_Slow()
{
  Thunder_Motor.Set_L_Motor_Power(0);  
  Thunder_Motor.Set_R_Motor_Power(0);

  while(Thunder_Motor.Get_L_Speed() > 15 || Thunder_Motor.Get_R_Speed() > 15)
  {}
}
#if 1
/* 
 * 前驱电机的巡线
 * 1、控制没有使用PID控制，直接列举方式将传感器状态值作为参考量，
 *    电机运动功率分三个级别 Line_H_Speed Line_M_Speed Line_L_Speed（无PID控制速度）
 *    对应着不同的状态值：
 *    0(两点黑线) 
 *    1(偏右时间超过 50ms，需要大幅度偏左运动) 
 *    2 (偏左时间超过 50ms，需要大幅度偏右运动) 
 *    3(偏右时间小于 50ms，需要小幅度偏左运动) 
 *    4 (偏左时间小于 50ms，需要小幅度偏右运动) 
 *    5(两个白点持续长时间)
 *  a、快速出线，打转
 * 2、电机安装在前面，传感器安装在前面
 * 3、传感器安装高度升为 1.5cm，与电机间隔两个安装孔位置
 * 
 * @parameters: 
 * @return: 
 */
#define WAIT_DIRECTION_COMFIRM_TIME       100 //ms
#define MAYBE_STRAIGHT_DIRECTION          300 //ms
#define LITTLE_L_R_POWER_DIFF             7
#define SPIN_L_R_DIFF_ROTATEVALUE         350 //编码器数值的差量300为打转90度

void THUNDER::Line_Tracing(void)
{
  int last_L_R_diffrotate;
  int current_L_R_diffrotate;
  int rotate_back_quantity;

  uint32_t L_last_time; // 上次偏左的时间戳
  uint32_t R_last_time; // 上次偏右的时间戳

  Line_last_time = millis();
  Line_last_led_time = millis();
  Line_last_sound_time = millis();
  LED_counter = 99;

  Speaker.Play_Song(131);
  while (1)
  {
    // 接收到巡线停止指令， 则退出巡线循环
    if(Rx_Data[0] == 0x61){
      break;
    }
    Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
    // Serial.printf("*** Left: %d ___ Right: %d ***\n", IR_Data[0], IR_Data[1]);

    current_time = millis();

    if ((IR_Data[0] == 0) & (IR_Data[1] == 0)) //Serial.printf("SSSSSSSSSS 没线 SSSSSSSSSS\n");
    {
      // Speaker.Play_Song(7); //test用---------
      if (((current_time - 5000) > Line_last_time) & (current_time > 19000)) //超时未找到线停止
      {
        Thunder_Motor.Set_L_Motor_Power(0);  
        Thunder_Motor.Set_R_Motor_Power(0);
        line_state = 5;
      }
      else if (line_state == 0) // 忽然从全黑变为全零过程中出线 
      {
        // 左右扭头查找黑线
        while (1)
        {
          Thunder_Motor.Set_L_Motor_Power(0);  
          Thunder_Motor.Set_R_Motor_Power(Line_M_Speed);
          Line_last_time = millis();
          current_time = millis();
          while( current_time - 500 < Line_last_time ){
            Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
            if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
              break;
            }
            current_time = millis();
          }
          if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
            break;
          }
          Thunder_Motor.Set_L_Motor_Power(Line_M_Speed);  
          Thunder_Motor.Set_R_Motor_Power(0);
          Line_last_time = millis();
          current_time = millis();
          while( current_time - 500 < Line_last_time ){
            Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
            if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
              break;
            }
            current_time = millis();
          }
          if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
            break;
          }
          
          if(Rx_Data[0] == 0x61){
            break;
          }
        }
      }
      else if (line_state == 3) //短时间偏右的过程中出线，打转(可能是直角转弯)
      {
        // 左右打转90度查找黑线
        rotate_back_quantity = 0;
        while (1)
        {
          Thunder_Motor.Set_L_Motor_Power(Line_B_Speed);  
          Thunder_Motor.Set_R_Motor_Power(Line_L_Speed);
          current_L_R_diffrotate = Thunder_Motor.Get_L_RotateValue() - Thunder_Motor.Get_R_RotateValue();
          last_L_R_diffrotate = current_L_R_diffrotate;
          while( current_L_R_diffrotate + SPIN_L_R_DIFF_ROTATEVALUE + rotate_back_quantity > last_L_R_diffrotate ){
            Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
            if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
              break;
            }
            current_L_R_diffrotate = Thunder_Motor.Get_L_RotateValue() - Thunder_Motor.Get_R_RotateValue();
          }
          if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
            break;
          }

          Thunder_Motor.Set_L_Motor_Power(Line_L_Speed);  
          Thunder_Motor.Set_R_Motor_Power(Line_B_Speed);
          current_L_R_diffrotate = Thunder_Motor.Get_L_RotateValue() - Thunder_Motor.Get_R_RotateValue();
          last_L_R_diffrotate = current_L_R_diffrotate;
          while( current_L_R_diffrotate - SPIN_L_R_DIFF_ROTATEVALUE*2 < last_L_R_diffrotate ){
            Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
            if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
              break;
            }
            current_L_R_diffrotate = Thunder_Motor.Get_L_RotateValue() - Thunder_Motor.Get_R_RotateValue();
          }
          if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
            break;
          }
          
          rotate_back_quantity = SPIN_L_R_DIFF_ROTATEVALUE; // 回转要增加旋转量
          
          if(Rx_Data[0] == 0x61){
            break;
          }
        }
        Wait_For_Motor_Slow();
      }
      else if ( line_state == 4 ) //短时间偏左的过程中出线，打转(可能是直角转弯)
      {
        // 左右打转90度查找黑线
        rotate_back_quantity = 0;
        while (1)
        {
          Thunder_Motor.Set_L_Motor_Power(Line_L_Speed);  
          Thunder_Motor.Set_R_Motor_Power(Line_B_Speed);
          current_L_R_diffrotate = Thunder_Motor.Get_L_RotateValue() - Thunder_Motor.Get_R_RotateValue();
          last_L_R_diffrotate = current_L_R_diffrotate;
          while( current_L_R_diffrotate - SPIN_L_R_DIFF_ROTATEVALUE - rotate_back_quantity < last_L_R_diffrotate ){
            Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
            if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
              break;
            }
            current_L_R_diffrotate = Thunder_Motor.Get_L_RotateValue() - Thunder_Motor.Get_R_RotateValue();
          }
          if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
            break;
          }

          Thunder_Motor.Set_L_Motor_Power(Line_B_Speed);  
          Thunder_Motor.Set_R_Motor_Power(Line_L_Speed);
          current_L_R_diffrotate = Thunder_Motor.Get_L_RotateValue() - Thunder_Motor.Get_R_RotateValue();
          last_L_R_diffrotate = current_L_R_diffrotate;
          while( current_L_R_diffrotate + SPIN_L_R_DIFF_ROTATEVALUE*2 > last_L_R_diffrotate ){
            Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
            if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
              break;
            }
            current_L_R_diffrotate = Thunder_Motor.Get_L_RotateValue() - Thunder_Motor.Get_R_RotateValue();
          }
          if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
            break;
          }
          
          rotate_back_quantity = SPIN_L_R_DIFF_ROTATEVALUE; // 回转要增加旋转量
          
          if(Rx_Data[0] == 0x61){
            break;
          }
        }
        Wait_For_Motor_Slow();
      }
      else if (line_state == 1) //偏右的过程中出线，快速漂移打转(弯道转弯)
      {
        Thunder_Motor.Set_L_Motor_Power(Line_B_Speed);  
        Thunder_Motor.Set_R_Motor_Power(Line_M_Speed);
        while(1){
          Line_last_time = millis(); // 不改变line_state，刷新时间
          Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
          if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
            break;
          }
        }
        Wait_For_Motor_Slow();
      }
      else if (line_state == 2) //偏左的过程中出线，快速漂移打转(弯道转弯)
      {
        Thunder_Motor.Set_L_Motor_Power(Line_M_Speed);
        Thunder_Motor.Set_R_Motor_Power(Line_B_Speed);
        while(1){
          Line_last_time = millis(); // 不改变line_state，刷新时间
          Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
          if( (IR_Data[0] != 0) || (IR_Data[1] != 0) ){
            break;
          }
        }
        Wait_For_Motor_Slow();
      }
      else
      {
        Thunder_Motor.Set_L_Motor_Power(0);  
        Thunder_Motor.Set_R_Motor_Power(0);
      }
    }
    else if (IR_Data[0] == 0) //Serial.printf("SSSSSSSSSS 右转 SSSSSSSSSS\n");
    {
      if ((line_state == 1) | (line_state == 3)) //从左转过来的需要更新时间
      {
        Line_last_time = millis(); // 不改变运动状态
        continue; // 保持前运动状态继续运动
      }

        if( R_last_time + MAYBE_STRAIGHT_DIRECTION < current_time ){  // 如果上次偏右时间已经超过200ms，那这次偏左需要偏向运动
          Thunder_Motor.Set_L_Motor_Power(Line_M_Speed);  
          Thunder_Motor.Set_R_Motor_Power(Line_L_Speed);
        }else{
          Thunder_Motor.Set_L_Motor_Power(Line_M_Speed);  
          Thunder_Motor.Set_R_Motor_Power(Line_M_Speed - LITTLE_L_R_POWER_DIFF);
        }
        
      if (line_state == 2)
      {
        Line_last_time = millis(); //一直为偏左出线，所以一直更新状态时间
      }
      else if ((current_time - WAIT_DIRECTION_COMFIRM_TIME) > Line_last_time && line_state == 4)
      {
        line_state = 2; // 偏左时间已经超过 50ms
        L_last_time = current_time;
      }
      else
      {
        line_state = 4; // 偏左时间小于 50ms，急转偏右运动一小段时间
        Thunder_Motor.Set_L_Motor_Power(Line_H_Speed);  
        Thunder_Motor.Set_R_Motor_Power(Line_L_Speed);
      }
    }
    else if (IR_Data[1] == 0) //Serial.printf("SSSSSSSSSS 左转 SSSSSSSSSS\n");
    {
      if ((line_state == 2) | (line_state == 4)) //从右转过来的需要更新时间
      {
        Line_last_time = millis(); // 不改变运动状态
        continue; // 保持前运动状态继续运动
      }

        if( L_last_time + MAYBE_STRAIGHT_DIRECTION < current_time ){  // 如果上次偏右时间已经超过200ms，那这次偏左需要偏向运动
          Thunder_Motor.Set_L_Motor_Power(Line_L_Speed);
          Thunder_Motor.Set_R_Motor_Power(Line_M_Speed);
        }else{
          Thunder_Motor.Set_L_Motor_Power(Line_M_Speed - LITTLE_L_R_POWER_DIFF);  
          Thunder_Motor.Set_R_Motor_Power(Line_M_Speed);
        }
        
      if (line_state == 1)
      {
        Line_last_time = millis();//一直为偏右出线，所以一直更新状态时间, 不改变运动状态
      }
      else if ((current_time - WAIT_DIRECTION_COMFIRM_TIME) > Line_last_time && line_state == 3)
      {
        line_state = 1; // 偏右时间已经超过 50ms
        R_last_time = current_time;
      }
      else
      {
        Thunder_Motor.Set_L_Motor_Power(Line_L_Speed);  
        Thunder_Motor.Set_R_Motor_Power(Line_H_Speed);
        line_state = 3; // 偏右时间小于 50ms，急转偏右运动一小段时间
      }
    }
    else //Serial.printf("SSSSSSSSSS 线上 SSSSSSSSSS\n");
    {
      Thunder_Motor.Set_L_Motor_Power(Line_M_Speed);  
      Thunder_Motor.Set_R_Motor_Power(Line_M_Speed);
      line_state = 0;
      Line_last_time = millis();
    }

    ////////////////////////////////// (start)巡线时LED画面、播放的声音 //////////////////////////////////
    if ((current_time - 50) > Line_last_led_time)
    {
      if ((LED_counter > 98) & (line_state != 5))
      {
        LED_counter = 77;
      }
      else if ((LED_counter > 100) & (line_state == 5))
      {
        LED_counter = 99;
        Line_last_sound_time = millis();
      }

      Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_counter); //单色点阵图案
      LED_counter++;

      Line_last_led_time = millis();
    }

    if (((current_time - 19000) > Line_last_sound_time) & (current_time > 19000))
    {
      Speaker.Play_Song(139);
      Line_last_sound_time = millis();
    }
    // En_Motor();
    ////////////////////////////////// (end)巡线时LED画面、播放的声音 //////////////////////////////////
  }
  Speaker.Play_Song(130);
}
#else

#define LINE_TRACE_PERIOD_MS        5
#define LINE_TRACE_ACCELERATION     1

#define LINE_TRACE_ACCE_P           2 //5
#define LINE_DEVIATION_ACCE_P       2 //5
#define LINE_DEVIATION_DEEP_ACCE_P  2 //5
#define LINE_OUTSIDE_ACCE_P         2 //7

#define LINE_INITIAL_MAX_POWER    140 // 初始化时最大的功率，如果最大功率达不到最大速度，会提高最大功率left_max_power right_max_power
#define LINE_INITIAL_POWER        50  // 初始化时的功率，为了快速加速起来，初始化功率作为偏置量，
#define LINE_RUN_MIN_POWER        70  // 运动轮子的最低功率，也是为了轮子可以从停止状态或反转状态 快速加速起来
#define LINE_CHECK_MAX_POWER      90 // 搜索黑线时的初始速度
#define LINE_CHECK_MIN_POWER      70  // 搜索黑线时的结束速度
#define LINE_CHECK_ROTATE         5  // 30  搜索黑线时摇头的幅度

#define LINE_TRACE_MAX_SPEED      20  // 运动轮子的最大速度（也是偏向时的高速度轮子的速度）
#define LINE_TRACE_MIN_SPEED      7  // 运动轮子的最小速度（也是偏向时的低速度轮子的速度）
#define LINE_TRACE_BACK_SPEED     5   // 出界时后退的速度

typedef enum{
  LINE_STATE_START = 0,
  LINE_STATE_STRAIGHT,
  LINE_STATE_LEFT,
  LINE_STATE_RIGHT,
  LINE_STATE_LEFT_DEEP,
  LINE_STATE_RIGHT_DEEP,
  LINE_STATE_LEFT_OVER,
  LINE_STATE_RIGHT_OVER,
  LINE_STATE_LOST
}enum_line_state;

/* 
 * 至少要有一个点是放置在黑线上面的
 * 
 * @parameters: 
 * @return: 
 */
void Wait_Line_Location()
{
  byte history_data[2] = {0x00, 0x00}; 
  byte sensor_data[2];

  while(1){
    for(uint8_t i = 0; i < 8; i++){
      Thunder.Get_IR_Data(sensor_data);
      history_data[0] = history_data[0] << 1 | sensor_data[0];
      history_data[1] = history_data[1] << 1 | sensor_data[1];
    }

    Serial.printf("*** L: 0x%2X   R: 0x%2X ***\n", history_data[0], history_data[1]);
    if(history_data[0] == 0xFF || history_data[1] == 0xFF){
      break;
    }
    // Speaker.Play_Song(103); // 播放声音：oh, no
    delay(2000);
  }
  // Speaker.Play_Song(101); // 播放声音：lets go
  delay(2000);
}

float left_power = 0; // 电机使用的功率
float right_power = 0;
float left_power_I[3] = {0, 0, 0}; // 当前状态的积分量
float right_power_I[3] = {0, 0, 0};
float left_power_pre[3] = {0, 0, 0}; // 之前状态最后使用的功率
float right_power_pre[3] = {0, 0, 0};

float left_max_power = LINE_INITIAL_MAX_POWER;
float right_max_power = LINE_INITIAL_MAX_POWER;

// bit0 是左边有白线
byte lost_recover = 0;

int32_t current_line_rotate = 0;
int32_t search_line_rotate = 0;
// 0 为没有搜索动作，1为出去左搜索，2为左搜索回退，3为出去右搜索，4为右搜索回退
uint32_t search_line_status = 0;

/* 
 * 当巡线状态处于LINE_STATE_LOST 时，要去判断当前是否处于黑线中的位置
 * (可以再加强：如果中间是白色，而且两边都是黑线，就继续跟随白线走，
 *             否则移动到黑色，再判断黑线的两边是否都为白色）
 * 处于黑线中的位置判断条件：当前位置两点传感器数值都为黑色，然后来回探测左右两边是否有白色
 * 如果当前是两点黑，左右探测回来的数据 lost_recover bit0 bit1都是标记为白色，
 * 那就是LINE_STATE_LOST 已经恢复为 LOST_STATE_START
 * 
 * @parameters: 
 * @return: 
 */
void Check_Line_When_Lost()
{
  uint8_t currentIrData = 0;

  currentIrData = Thunder.history_data[1] & 0x01;
  // 右边数据在 bit1, 左边数据在 bit0
  currentIrData = (currentIrData << 1) | (Thunder.history_data[0] & 0x01); 

  // 每次搜索动作行程 完成后 再判断状态
  switch(search_line_status){
  case 0:
    // 设置功率为 0 后，需要等待电机停止
    left_power = 0;
    right_power = 0; 
    while(Thunder_Motor.Get_L_Speed() != 0
           || Thunder_Motor.Get_R_Speed() != 0);

    if(currentIrData == 0x03){
      left_power = 0;
      right_power = LINE_CHECK_MAX_POWER;
      lost_recover = 0; // init bit0 and bit1

      search_line_status = 1;
      search_line_rotate = Thunder_Motor.Get_R_RotateValue();
    }
  break;

  case 1:
    current_line_rotate = Thunder_Motor.Get_R_RotateValue();
    if(current_line_rotate < search_line_rotate + LINE_CHECK_ROTATE){
      if(right_power > LINE_CHECK_MIN_POWER){
        right_power -= 1;
      }
      return;
    }

    if(currentIrData == 0x03 || currentIrData == 0x01){
      lost_recover &= 0xFE; // bit0 No white

    }else{
      lost_recover |= 0x01; // bit0 white
    }
    left_power = 0;
    right_power = -LINE_CHECK_MAX_POWER;
    search_line_status = 2;
    search_line_rotate = Thunder_Motor.Get_R_RotateValue();

  break;
  
  case 2:
    current_line_rotate = Thunder_Motor.Get_R_RotateValue();
    if(current_line_rotate > search_line_rotate - LINE_CHECK_ROTATE){
      if(right_power < -LINE_CHECK_MIN_POWER){
        right_power += 1;
      }
      return;
    }

    left_power = LINE_CHECK_MAX_POWER;
    right_power = 0;
    lost_recover &= 0xFD; // init bit1
    search_line_status = 3;
    search_line_rotate = Thunder_Motor.Get_L_RotateValue();
  break;

  case 3:
    current_line_rotate = Thunder_Motor.Get_L_RotateValue();
    if(current_line_rotate < search_line_rotate + LINE_CHECK_ROTATE){
      if(left_power > LINE_CHECK_MIN_POWER){
        left_power -= 1;
      }
      return;
    }
    
    if(currentIrData == 0x03 || currentIrData == 0x02){
      lost_recover &= 0xFD; //bit1 No white
    }else{
      lost_recover |= 0x02; // bit0 white
    }

    left_power = -LINE_CHECK_MAX_POWER;
    right_power = 0;
    search_line_status = 4;
    search_line_rotate = Thunder_Motor.Get_L_RotateValue();
  break;
  
  case 4:
    current_line_rotate = Thunder_Motor.Get_L_RotateValue();
    if(current_line_rotate > search_line_rotate - LINE_CHECK_ROTATE){
      if(left_power < -LINE_CHECK_MIN_POWER){
        left_power += 1;
      }
      return;
    }
    
    left_power = 0;
    right_power = 0;
    search_line_status = 0;
  break;

  default:
  break;
  }

  // 左右探测回来的数据 lost_recover bit0 bit1都是标记为白色, 
  // 就将 LINE_STATE_LOST 恢复为 LOST_STATE_START
  if(search_line_status == 0){
    if( lost_recover == 3 ){
      left_power = 0;
      right_power = 0;
      while(Thunder_Motor.Get_L_Speed() != 0
            || Thunder_Motor.Get_R_Speed() != 0);
      search_line_status = 0;
      lost_recover = 0;

      Thunder.line_state = LINE_STATE_START;

      Serial.println("line tracing recover!");
      return;
    }else{
      delay(2000);// 延时方便用户在这个时间内重新放置好小车。
    }
  }
}

#define DIRECTION_TURN_MAX_POWER  60  // 控制方向的左右轮速度差最大值

// 左右轮最大速度差，这个速度差相当于控制方向，值越大，拐的角度越大
// 这里的值蕴含着运动最大曲率的问题，这个值与运动速度可以计算出最大转弯的曲率
#define LINE_DIFF_MAX_SPEED       3.0 
#define LINE_RUN_SPEED        20.0 // 固定速度，做PI控制的巡线是选用的初始速度

#if 0
#define LINE_TRACE_P              5.0   // PI 控制速度的参数 Kp
#define LINE_TRACE_ACCE_I         0.1   // PI 控制速度加速的参数 Ki
#define LINE_TRACE_DECE_I         0.01  // PI 控制速度减速的参数 Ki
#define LINE_TRACE_SPEED_D        15    // PI 控制速度的偏差改变量因子 Kd

#define LINE_DEVIATION_I          0.5   // PI 控制偏差的参数 Ki
#define LINE_DEVIATION_P          10.0  // PI 控制偏差的参数 Kp
#define LINE_DEVIATION_D          0.2   // PI 控制偏差的参数 Kd

#define DEVIATION_VALUE           1
#define DEVIATION_DEEP_VALUE      2
#define DEVIATION_OUT_VALUE       3
#else
// 3.000 0.100 0.010 5.000 10.00 0.500 0.100  直线走的很好，不能正常进弯道，不能出弯道
// 3.000 0.100 0.000 5.000 10.00 0.500 1.000
// 3.000 0.100 0.000 10.00 15.00 0.200 1.000  可以走完一圈
// 3.000 0.090 0.000 15.00 15.00 0.130 0.000  可以走完一圈
// 3.000 0.130 0.000 10.00 15.00 0.300 0.600  可以有点顺的走完一圈，但是一直偏向外边的白线
// 3.000 0.130 0.000 10.00 8.000 0.280 0.000  可以有点顺的走完一圈，比上一个参数走的快点，而且可以回归到黑线

// #define DIRECTION_TURN_MAX_POWER  35
// #define LINE_TRACE_MAX_SPEED 10 时采用的参数
// 3.000 0.130 0.000 10.00 10.00 0.280 0.100  

// #define DIRECTION_TURN_MAX_POWER  45
// #define LINE_TRACE_MAX_SPEED 15 时
// 2.000 0.080 0.020 8.00 20.00 0.370 0.020
// 2.000 0.080 0.012 8.000 10.00 0.200 0.000 加减速还可以，转向控制不好
// 2.000 0.090 0.012 8.000 10.00 0.200 0.000 可以走完一圈
// 2.000 0.100 0.016 5.000 10.00 0.200 1.000
// 2.000 0.080 0.014 3.000 10.00 0.200 1.000

// 方向控制PID
// 3.000 0.400 0.000 0.000 2.000 0.016 2.000
// 3.000 0.400 0.000 0.000 2.000 0.018 1.300
// 3.000 0.400 0.000 0.000 2.000 0.015 1.300  // #define LINE_RUN_SPEED        10.0
// 3.000 0.400 0.000 0.000 1.500 0.018 1.300  // #define LINE_RUN_SPEED        20.0
float LINE_TRACE_P = 3.0;                // PI 控制速度的参数 Kp
float LINE_TRACE_ACCE_I = 0.40;           // PI 控制速度加速的参数 Ki

float LINE_TRACE_DECE_I = 0.00;           // PI 控制速度减速的参数 Ki
float LINE_TRACE_SPEED_D = 0.0;          // PI 控制速度的偏差改变量因子 Kd

float LINE_DEVIATION_P = 1.50;            // PI 控制偏差的参数 Kp
float LINE_DEVIATION_I = 0.018;            // PI 控制偏差的参数 Ki
float LINE_DEVIATION_D = 1.30;            // PI 控制偏差的参数 Kd

float DEVIATION_VALUE = 1;           
float DEVIATION_DEEP_VALUE = 2;      
float DEVIATION_OUT_VALUE = 3;       
#endif

int deviation[2] = {0, 0}; // 无偏差时为0；左偏差时 正数，右偏差时 负数
float tempValue;

/* 
 * 加速度 acceleration，所以电机功率对时间进行积分
 * 
 * @parameters: 
 * @return: 
 */
void Calculate_Motor_Power()
{
  int16_t diffSpeed_left;
  int16_t diffSpeed_right;

  int diffDeviation;

  // 保存好上一次的偏移量
  deviation[1] = deviation[0];

  switch(Thunder.line_state){
    case LINE_STATE_START: // 刚启动状态
      deviation[1] = 0;
      deviation[0] = 0;

      left_power_I[0] = 0;
      right_power_I[0] = 0;
      left_power_I[1] = 0;
      right_power_I[1] = 0;
      left_power_I[2] = 0;
      right_power_I[2] = 0;

      left_power = LINE_INITIAL_POWER;
      right_power = LINE_INITIAL_POWER;
    break;
    case LINE_STATE_STRAIGHT: // 直行状态
      deviation[0] = 0;
    break;
      
    case LINE_STATE_LEFT: // 偏左状态, 左轮加速
      deviation[0] = DEVIATION_VALUE;
    break;
    case LINE_STATE_RIGHT: // 偏右状态, 右轮加速
      deviation[0] = -DEVIATION_VALUE;
    break;

    case LINE_STATE_LEFT_DEEP: // 太偏左状态, 整体减速
      deviation[0] = DEVIATION_DEEP_VALUE;
    break;
    case LINE_STATE_RIGHT_DEEP: // 太偏右状态, 整体减速
      deviation[0] = -DEVIATION_DEEP_VALUE;
    break;

    case LINE_STATE_LEFT_OVER: // 左向出界, 可以打转
      deviation[0] = DEVIATION_OUT_VALUE;
    break;
    case LINE_STATE_RIGHT_OVER: // 右向出界, 可以打转
      deviation[0] = -DEVIATION_OUT_VALUE;
    break;

    case LINE_STATE_LOST: // 迷失状态，需要尝试左右旋转寻找出路
      deviation[1] = 0;
      deviation[0] = 0;

      left_power_I[0] = 0;
      right_power_I[0] = 0;
      left_power_I[1] = 0;
      right_power_I[1] = 0;
      left_power_I[2] = 0;
      right_power_I[2] = 0;

      Check_Line_When_Lost();
      Thunder_Motor.Set_L_Motor_Power( (int)left_power );
      Thunder_Motor.Set_R_Motor_Power( (int)right_power );
      return;
    break;

    default:
    break;
  }

  // // 直线状态会加速，否则会先减速
  // if(deviation[0] == 0){

  //   left_power_I[1] = 0;
  //   right_power_I[1] = 0;
  // }else{
  //   // 方向 逆向修复量
  //   tempValue = left_power_I[1];
  //   left_power_I[1] += LINE_DEVIATION_D * ((diffDeviation > 0)? right_power_I[1] : 0);
  //   right_power_I[1] += LINE_DEVIATION_D * ((diffDeviation > 0)? tempValue : 0);

  //   diffSpeed_left = Thunder_Motor.Get_L_Speed(); // 速度越大，减速效果越大
  //   diffSpeed_right = Thunder_Motor.Get_R_Speed();

  //   // 减速的依据有 目前速度值 和 方向偏移量
  //   left_power_I[0] -= (left_power_I[0] < 50)? 0 : LINE_TRACE_DECE_I * diffSpeed_left * abs(deviation[0]);
  //   right_power_I[0] -= (right_power_I[0] < 50)? 0 : LINE_TRACE_DECE_I * diffSpeed_right * abs(deviation[0]);
  // }
  
  /***********************************方向控制PID**************************************/
  // P: PI控制的 比例P 量，左右轮有个固定的功率差
  left_power_pre[1] = LINE_DEVIATION_P * deviation[0];
  right_power_pre[1] = -LINE_DEVIATION_P * deviation[0];

  // I: 根据偏向量deviation 调节左右轮的速度. deviation[0] 在左偏时为正数，右偏时为负数
  tempValue = left_power_I[1] + LINE_DEVIATION_I * deviation[0];
  if( abs(tempValue) < LINE_DIFF_MAX_SPEED ){
    left_power_I[1] = tempValue;
  }
  tempValue = right_power_I[1] - LINE_DEVIATION_I * deviation[0];
  if( abs(tempValue) < LINE_DIFF_MAX_SPEED ){
    right_power_I[1] = tempValue;
  }

  // 偏移量变化时给一个速度变化量
  diffDeviation = abs(deviation[1]) - abs(deviation[0]);
  // 分级减速, 用于出线时减速，进线时提速
  left_power_I[2] += (LINE_DEVIATION_D * diffDeviation);
  right_power_I[2] += (LINE_DEVIATION_D * diffDeviation);

  // 得到的 左右轮子的速度控制目标 left_power_pre[2] right_power_pre[2]
  left_power_pre[2] = LINE_RUN_SPEED + left_power_pre[1] + left_power_I[1] + left_power_I[2];
  right_power_pre[2] = LINE_RUN_SPEED + right_power_pre[1] + right_power_I[1] + right_power_I[2];

  /***********************************速度控制PID**************************************/
  // 计算 速度控制目标 与 速度编码器 的差值
  diffSpeed_left = left_power_pre[2] - Thunder_Motor.Get_L_Speed();
  diffSpeed_right = right_power_pre[2] - Thunder_Motor.Get_R_Speed();

  // 积分
  left_power_I[0] += ( abs(left_power_I[0] + LINE_TRACE_ACCE_I * diffSpeed_left) > 255 )?
                      0 : LINE_TRACE_ACCE_I * diffSpeed_left;
  right_power_I[0] += ( abs(right_power_I[0] + LINE_TRACE_ACCE_I * diffSpeed_right) > 255 )?
                      0 : LINE_TRACE_ACCE_I * diffSpeed_right;
  // 比例
  left_power_pre[0] = LINE_TRACE_P * diffSpeed_left; 
  right_power_pre[0] = LINE_TRACE_P * diffSpeed_right;

  // 根据速度控制得出 电机功率
  left_power = left_power_pre[0] + left_power_I[0];
  right_power = right_power_pre[0] + right_power_I[0];


  // if(left_power > left_max_power){
  //   // 已经设置为最大功率了，如果因为电池原因达不到最大速度，则将最大功率调高
  //   if(Thunder_Motor.Get_L_Speed() < LINE_TRACE_MAX_SPEED){
  //     left_max_power = (left_max_power<255-LINE_TRACE_ACCELERATION)?(left_max_power+LINE_TRACE_ACCELERATION):255;
  //   }else if(Thunder_Motor.Get_L_Speed() > LINE_TRACE_MAX_SPEED + 3){
  //     left_max_power -= LINE_TRACE_ACCELERATION;
  //   }
  //   left_power = left_max_power;
  // }
  // if(right_power > right_max_power){
  //   // 已经设置为最大功率了，如果因为电池原因达不到最大速度，则将最大功率调高
  //   if(Thunder_Motor.Get_R_Speed() < LINE_TRACE_MAX_SPEED){
  //     right_max_power = (right_max_power<255-LINE_TRACE_ACCELERATION)?(right_max_power+LINE_TRACE_ACCELERATION):255;
  //   }else if(Thunder_Motor.Get_R_Speed() > LINE_TRACE_MAX_SPEED + 3){
  //     right_max_power -= LINE_TRACE_ACCELERATION;
  //   }
  //   right_power = right_max_power;
  // }

  // 处理越界问题，不能简单处理，需要保证左右轮的差量合适
  if(left_power > 255) left_power = 255;
  else if(left_power < -255) left_power = -255;
  if(right_power > 255) right_power = 255;
  else if(right_power < -255) right_power = -255;

  Thunder_Motor.Set_L_Motor_Power( (int)left_power );
  Thunder_Motor.Set_R_Motor_Power( (int)right_power );

  #ifdef DEBUG_LINE_TRACING
    // Serial.printf("*** Power L: %d   R: %d ***\n", (int)left_power, (int)right_power);
    Serial.printf("P0: %6.2f %6.2f ", left_power_pre[0], right_power_pre[0]);
    Serial.printf("I0: %6.2f %6.2f ", left_power_I[0], right_power_I[0]);
    Serial.printf("P1: %6.2f %6.2f ", left_power_pre[1], right_power_pre[1]);
    Serial.printf("I1: %6.2f %6.2f\n", left_power_I[1], right_power_I[1]);
  #endif
}

/* 
 * 后驱电机的巡线
 * 1、控制使用了 方向PID、速度PID、分级减速的方式进行
 *    a/将传感器状态值输进状态机，以获取到位置状态；
 *    b/将位置状态转变为一个偏移量数值，以偏移量作为参考量
 *    c/以一个固定速度行驶，方向PID控制两轮间的差速，以偏移量的变化量进行分级减速（后面可以改一版：减速也可以进行PID控制）
 *      ，得到最后的速度值，然后速度PID以速度值作为参考量进行控制电机功率。
 * 2、车型：电机安装在后面，传感器安装在前面
 * 3、传感器安装高度升为 1.5cm
 * 
 * @parameters: 
 * @return: 
 */
void THUNDER::Line_Tracing(void)
{
  history_data[0] = 0xFF;
  history_data[1] = 0xFF;

  #ifdef DEBUG_LINE_TRACING
    // 等待串口输入新的PID参数
    uint32_t beginWaitTime;
    beginWaitTime = millis();
    Serial.print("Kp Ki Ki Kd Kp Ki Kd: ");
    //5.000 0.100 0.010 10.00 10.00 0.500 0.200
    while( Serial.available() < 41){
      current_time = millis();
      if(current_time > beginWaitTime + 5000){
        break;
      }
    }
    if(Serial.available() >= 41){
      LINE_TRACE_P = Serial.parseFloat();
      LINE_TRACE_ACCE_I = Serial.parseFloat();
      LINE_TRACE_DECE_I = Serial.parseFloat();
      LINE_TRACE_SPEED_D = Serial.parseFloat();
      LINE_DEVIATION_P = Serial.parseFloat();
      LINE_DEVIATION_I = Serial.parseFloat();
      LINE_DEVIATION_D = Serial.parseFloat();
    }
    Serial.printf("\nK: %f %f\n", LINE_TRACE_P, LINE_DEVIATION_D);
  #endif

  Line_last_time = millis();
  Line_last_led_time = millis();
  Line_last_sound_time = millis();
  LED_counter = 99;

  Wait_Line_Location();
  line_state = LINE_STATE_START;

  // Speaker.Play_Song(131);
  while (Rx_Data[1] == 1)
  {
    Get_IR_Data(IR_Data); //更新IR数据 //0-->白; 1-->黑
    history_data[0] = history_data[0] << 1 | IR_Data[0];
    history_data[1] = history_data[1] << 1 | IR_Data[1];

    current_time = millis();
    if( current_time - Line_last_time > LINE_TRACE_PERIOD_MS ){
    // Serial.printf("*** Left: %d   Right: %d ***\n", IR_Data[0], IR_Data[1]);
      if ((IR_Data[0] == 0) & (IR_Data[1] == 0)) // 两点白
      {
        switch(line_state){
          case LINE_STATE_LEFT: // 偏左状态
            line_state = LINE_STATE_LEFT_DEEP;
            break;
          case LINE_STATE_RIGHT: // 偏右状态
            line_state = LINE_STATE_RIGHT_DEEP;
            break;
          case LINE_STATE_LEFT_DEEP: // 太偏左状态
            line_state = LINE_STATE_LEFT_DEEP;
            break;
          case LINE_STATE_RIGHT_DEEP: // 太偏右状态
            line_state = LINE_STATE_RIGHT_DEEP;
            break;
          case LINE_STATE_LEFT_OVER: // 左向出界
            line_state = LINE_STATE_LEFT_DEEP;
            break;
          case LINE_STATE_RIGHT_OVER: // 右向出界
            line_state = LINE_STATE_RIGHT_DEEP;
            break;
          case LINE_STATE_START: // 刚启动状态
          case LINE_STATE_STRAIGHT: // 直行状态
          case LINE_STATE_LOST: // 迷失状态，需要尝试左右旋转寻找出路
          default:
            line_state = LINE_STATE_LOST;
            break;
        }
      }
      else if (IR_Data[0] == 0) // 左点白
      {
        switch(line_state){
          case LINE_STATE_START: // 刚启动状态
            line_state = LINE_STATE_LEFT;
            break;
          case LINE_STATE_STRAIGHT: // 直行状态
            line_state = LINE_STATE_LEFT;
            break;
          case LINE_STATE_LEFT: // 偏左状态
            line_state = LINE_STATE_LEFT;
            break;
          case LINE_STATE_LEFT_DEEP: // 太偏左状态
            line_state = LINE_STATE_LEFT;
            break;
          case LINE_STATE_RIGHT_OVER: // 右向出界
            line_state = LINE_STATE_RIGHT_OVER;
            break;
          case LINE_STATE_RIGHT_DEEP: // 太偏右状态
            line_state = LINE_STATE_RIGHT_OVER;
            break;
          case LINE_STATE_LOST: // 迷失状态，需要尝试左右旋转寻找出路
          case LINE_STATE_RIGHT: // 偏右状态
          case LINE_STATE_LEFT_OVER: // 左向出界
          default:
            line_state = LINE_STATE_LOST;
            break;
        }
      }
      else if (IR_Data[1] == 0) // 右点白
      {
        switch(line_state){
          case LINE_STATE_START: // 刚启动状态
            line_state = LINE_STATE_RIGHT;
            break;
          case LINE_STATE_STRAIGHT: // 直行状态
            line_state = LINE_STATE_RIGHT;
            break;
          case LINE_STATE_RIGHT: // 偏右状态
            line_state = LINE_STATE_RIGHT;
            break;
          case LINE_STATE_LEFT_DEEP: // 太偏左状态
            line_state = LINE_STATE_LEFT_OVER;
            break;
          case LINE_STATE_RIGHT_DEEP: // 太偏右状态
            line_state = LINE_STATE_RIGHT;
            break;
          case LINE_STATE_LEFT_OVER: // 左向出界
            line_state = LINE_STATE_LEFT_OVER;
            break;
          case LINE_STATE_LOST: // 迷失状态，需要尝试左右旋转寻找出路
          case LINE_STATE_LEFT: // 偏左状态
          case LINE_STATE_RIGHT_OVER: // 右向出界
          default:
            line_state = LINE_STATE_LOST;
            break;
        }
      }
      else // 两点黑
      {
        switch(line_state){
          case LINE_STATE_START: // 刚启动状态
            line_state = LINE_STATE_STRAIGHT;
            break;
          case LINE_STATE_STRAIGHT: // 直行状态
            line_state = LINE_STATE_STRAIGHT;
            break;
          case LINE_STATE_LEFT: // 偏左状态
            line_state = LINE_STATE_STRAIGHT;
            break;
          case LINE_STATE_RIGHT: // 偏右状态
            line_state = LINE_STATE_STRAIGHT;
            break;
          case LINE_STATE_LEFT_DEEP: // 太偏左状态
          case LINE_STATE_RIGHT_DEEP: // 太偏右状态
          case LINE_STATE_LEFT_OVER: // 左向出界
            line_state = LINE_STATE_LEFT_OVER;
            break;
          case LINE_STATE_RIGHT_OVER: // 右向出界
            line_state = LINE_STATE_RIGHT_OVER;
            break;
          case LINE_STATE_LOST: // 迷失状态，需要尝试左右旋转寻找出路
          default:
            line_state = LINE_STATE_LOST;
            break;
        }
      }

      // Serial.printf("*** line state: %d *** \n", line_state);
      Calculate_Motor_Power();
      Line_last_time = millis();
    }


    ////////////////////////////////// 巡线时LED画面 //////////////////////////////////
    if ((current_time - 50) > Line_last_led_time)
    {
      if ((LED_counter > 98) & (line_state != LINE_STATE_LOST))
      {
        LED_counter = 77;
      }
      else if ((LED_counter > 100) & (line_state == LINE_STATE_LOST))
      {
        LED_counter = 99;
        Line_last_sound_time = millis();
      }

      Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_counter); //单色点阵图案
      LED_counter++;

      Line_last_led_time = millis();
    }

    ////////////////////////////////// 巡线时播放的声音 //////////////////////////////////
 /*    if (((current_time - 19000) > Line_last_sound_time) & (current_time > 19000))
    {
      Speaker.Play_Song(139);
      Line_last_sound_time = millis();
    } */

    // En_Motor();
  }
  Speaker.Play_Song(130);
}
#endif

// 开机动画/声效
void THUNDER::Start_Show(void)
{
  Speaker.Play_Song(79);                    //声音编号79
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(1); //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(2); //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(3); //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(4); //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(5); //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(4); //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(5); //单色点阵图案
  delay(200);
  Dot_Matrix_LED.Play_LED_HT16F35B_Show(4); //单色点阵图案
  delay(200);
}

// 等待蓝牙连接动画 (有串口数据也跳出)
#if 0 // 测试超声波 连续IIC 读取出错 问题
void THUNDER::Wait_BLE(void)
{
  float US_Data_cm = 0;
  while((deviceConnected == false) & (Usart_Communication == 0))
  {
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(6); //单色点阵图案
    
  US_Data_cm = US.Get_US_cm();
  if(US_Data_cm < 1.0){
    Serial.printf("### US_Data_cm : %.1f [cm]################1###############\n", US_Data_cm);
  }else{
    Serial.printf("*** US_Data_cm : %.1f [cm]***\n",US_Data_cm);
  }
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(7); //单色点阵图案
  delay(5);
  US_Data_cm = US.Get_US_cm();
  if(US_Data_cm < 1.0){
    Serial.printf("### US_Data_cm : %.1f [cm]#################2##############\n", US_Data_cm);
  }else{
    Serial.printf("*** US_Data_cm : %.1f [cm]***\n",US_Data_cm);
  }
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(8); //单色点阵图案
    
  US_Data_cm = US.Get_US_cm();
  if(US_Data_cm < 1.0){
    Serial.printf("### US_Data_cm : %.1f [cm]#################3##############\n", US_Data_cm);
  }else{
    Serial.printf("*** US_Data_cm : %.1f [cm]***\n",US_Data_cm);
  }

  // uint16_t lightValue = 0;
  // while((deviceConnected == false) & (Usart_Communication == 0))
  // {
  //   Dot_Matrix_LED.Play_LED_HT16F35B_Show(6); //单色点阵图案
    
  // lightValue = Light_Sensor.Get_Light_Value();
  // Serial.printf("light Value: %d \n", lightValue);

  //   delay(200);
  //   Dot_Matrix_LED.Play_LED_HT16F35B_Show(7); //单色点阵图案

  // delay(5);
  // lightValue = Light_Sensor.Get_Light_Value();
  // Serial.printf("light Value: %d \n", lightValue);

  //   delay(200);
  //   Dot_Matrix_LED.Play_LED_HT16F35B_Show(8); //单色点阵图案

  // lightValue = Light_Sensor.Get_Light_Value();
  // Serial.printf("light Value: %d \n", lightValue);

    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(9); //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(10); //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(11); //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(12); //单色点阵图案
    delay(200);
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(13); //单色点阵图案
    delay(200);

    if(Serial.available())
    {
      Usart_Communication = 1;
    }  
  }
}
#else
// 等待蓝牙连接动画 (有串口数据也跳出)
void THUNDER::Wait_BLE(void)
{
  while ((deviceConnected == false) & (Usart_Communication == 0))
  {
    if(lowpower_flag == 0){
      Dot_Matrix_LED.Play_LED_HT16F35B_Show(6); //单色点阵图案
      delay(200);
      Dot_Matrix_LED.Play_LED_HT16F35B_Show(7); //单色点阵图案
      delay(200);
      Dot_Matrix_LED.Play_LED_HT16F35B_Show(8); //单色点阵图案
      delay(200);
      Dot_Matrix_LED.Play_LED_HT16F35B_Show(9); //单色点阵图案
      delay(200);
      Dot_Matrix_LED.Play_LED_HT16F35B_Show(10); //单色点阵图案
      delay(200);
      Dot_Matrix_LED.Play_LED_HT16F35B_Show(11); //单色点阵图案
      delay(200);
      Dot_Matrix_LED.Play_LED_HT16F35B_Show(12); //单色点阵图案
      delay(200);
      Dot_Matrix_LED.Play_LED_HT16F35B_Show(13); //单色点阵图案
      delay(200);
    }
    if (Serial.available())
    {
      Usart_Communication = 1;
    }
  }
}
#endif

// 设置将要播放的内置动画编号
void THUNDER::Set_LED_Show_No(uint8_t Show_No)
{
  LED_show_No = Show_No;
}

// 循环执行的内置动画控制程序
void THUNDER::LED_Show(void)
{
  switch (LED_show_No)
  {
  case 0:
    // Serial.printf("SSSSSSSSSS 无表情 SSSSSSSSSS\n");
    break;
  case 1:
    break;
  case 3: //眨眼 8帧
    LED_delay_time = 200;
    if (LED_counter < 8)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_3[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 4: //爱心  5帧
    LED_delay_time = 100;
    if (LED_counter < 5)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_4[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 5: //眼镜  8帧
    if (LED_counter < 8)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_5[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_delay_time = LED_time_5[LED_counter];
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 6: //严肃 4帧
    LED_delay_time = 200;
    if (LED_counter < 4)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_6[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 7:                                      //刹车
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(29); //单色点阵图案
    break;
  case 8: //流汗  3帧
    if (LED_counter < 3)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_8[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_delay_time = LED_time_8[LED_counter];
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 9: //开心  5帧
    LED_delay_time = 200;
    if (LED_counter < 5)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_9[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 10: //等待  3帧
    LED_delay_time = 1000;
    if (LED_counter < 3)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_10[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 11: //眯眼 4帧
    LED_delay_time = 200;
    if (LED_counter < 4)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_11[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 12: //启动 4帧
    LED_delay_time = 200;
    if (LED_counter < 4)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_12[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 13: //凶  5帧
    LED_delay_time = 500;
    if (LED_counter < 5)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_13[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 14: //哭泣  9帧
    LED_delay_time = 200;
    if (LED_counter < 9)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_14[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  case 15: //语音图  5帧
    LED_delay_time = 100;
    if (LED_counter < 5)
    {
      current_time = millis();
      if ((current_time - last_led_time) > LED_delay_time)
      {
        Dot_Matrix_LED.Play_LED_HT16F35B_Show(LED_show_15[LED_counter]); //单色点阵图案
        last_led_time = millis();
        LED_counter++;
      }
    }
    else
    {
      LED_show_No = 0;
      LED_counter = 0;
      last_led_time = 0;
      LED_delay_time = 0;
    }
    break;
  }
}

// 通信确认，蓝牙/串口
void THUNDER::Check_Communication(void)
{
  Wait_BLE(); //如果蓝牙没有连接，等待蓝牙连接 (有串口数据也跳出)

  if (Rx_Data[0] != 0)
  {
    Check_Protocol();
    if (Tx_Data[0] != 0)
    {
      Tx_Data[5] = Tx_Data[0] + Tx_Data[1] + Tx_Data[2] + Tx_Data[3] + Tx_Data[4];
      Thunder_BLE.Tx_BLE(Tx_Data, 6); //通过蓝牙发送数据;参数1 --> 数据数组；参数2 -->字节数
    }
  }
  else
  {
    while (Serial.available())
    {
      Rx_Data[0] = Serial.read(); //Serial.parseInt();  //读取整数

#ifdef PRINT_UART_COMMAND
      Serial.printf("* recv UART cmd: %x *\n", Rx_Data[0]);
#endif
      //////////////////////////////////////// 其它特殊指令 //////////////////////////////////
      if (Rx_Data[0] == 0xC2) //刷新左侧彩色灯
      {
        uint8_t SUM = Rx_Data[0];
        for (int i = 1; i < 19; i++)
        {
          Thunder.I2C_LED_BUFF1[i - 1] = Serial.read();
          SUM += Thunder.I2C_LED_BUFF1[i - 1];
          // Serial.printf(": %x \n",Thunder.I2C_LED_BUFF1[i-1]);
        }

        if (SUM != Serial.read())
        {
          Rx_Data[0] = 0;
          Serial.printf("\n#  0xC2 cmd CKsum error #\n");
          Serial.printf("* SUM: %x *\n", SUM);
        }
        else
        {
          // Serial.printf("SSS ___ 0xC2 Complete ___ SSS \n");
        }
      }
      else if (Rx_Data[0] == 0xC3) //刷新右侧彩色灯
      {
        uint8_t SUM = Rx_Data[0];
        for (int i = 1; i < 19; i++)
        {
          Thunder.I2C_LED_BUFF2[i - 1] = Serial.read();
          SUM += Thunder.I2C_LED_BUFF2[i - 1];
          // Serial.printf(": %x \n",Thunder.I2C_LED_BUFF2[i-1]);
        }

        if (SUM != Serial.read())
        {
          Rx_Data[0] = 0;
          Serial.printf("\n# 0xC3 cmd CKsum error #\n");
          Serial.printf("* SUM: %x *\n", SUM);
        }
        else
        {
          // Serial.printf("SSS ___ 0xC3 Complete ___ SSS \n");
        }
      }
      else if (Rx_Data[0] == 0xD3) //单色点阵灯一次性刷新前半部分灯
      {
        uint8_t SUM = Rx_Data[0];
        for (int i = 1; i < 15; i++)
        {
          Thunder.LED_BUFF_Dot[i] = Serial.read();
          SUM += Thunder.LED_BUFF_Dot[i];
        }

        if (SUM != Serial.read())
        {
          Rx_Data[0] = 0;
          Serial.printf("\n# 0xD3 cmd CKsum error #\n");
          Serial.printf("* SUM: %x *\n", SUM);
        }
        else
        {
          // Serial.printf("SSS ___ 0xD3 Complete ___ SSS \n");
        }
      }
      else if (Rx_Data[0] == 0xD4) //单色点阵灯一次性刷新后半部分灯
      {
        uint8_t SUM = Rx_Data[0];
        for (int i = 1; i < 15; i++)
        {
          Thunder.LED_BUFF_Dot[i + 14] = Serial.read();
          SUM += Thunder.LED_BUFF_Dot[i + 14];
        }

        if (SUM != Serial.read())
        {
          Rx_Data[0] = 0;
          Serial.printf("\n# 0xD4 cmd CKsum error #\n");
          Serial.printf("* SUM: %x *\n", SUM);
        }
        else
        {
          // Serial.printf("SSS ___ 0xD4 Complete ___ SSS \n");
        }
      }
      else
      {
        Rx_Data[1] = Serial.read();
        Rx_Data[2] = Serial.read();
        Rx_Data[3] = Serial.read();
        Rx_Data[4] = Serial.read();
        Rx_Data[5] = Serial.read();

        if (Rx_Data[5] != (uint8_t)(Rx_Data[0] + Rx_Data[1] + Rx_Data[2] + Rx_Data[3] + Rx_Data[4]))
        {
          Serial.printf("# Rx_Data[0]: %x CKsum error # \n", Rx_Data[0]);
          Serial.printf("* SUM error __ Rx_Data[5]: %x __ sum: %x *\n", Rx_Data[5], (uint8_t)(Rx_Data[0] + Rx_Data[1] + Rx_Data[2] + Rx_Data[3] + Rx_Data[4]));
          Reset_Rx_Data();
        }
      }
    }
    Check_Protocol();
    if (Tx_Data[0] != 0)
    {
      Tx_Data[5] = Tx_Data[0] + Tx_Data[1] + Tx_Data[2] + Tx_Data[3] + Tx_Data[4];

      //串口返回数据
      Serial.flush();
      Serial.write(Tx_Data[0]);
      Serial.write(Tx_Data[1]);
      Serial.write(Tx_Data[2]);
      Serial.write(Tx_Data[3]);
      Serial.write(Tx_Data[4]);
      Serial.write(Tx_Data[5]);
      Serial.flush();

      Tx_Data[0] = 0;
    }
  }
}

// 协议解析
void THUNDER::Check_Protocol(void)
{
  switch (Rx_Data[0])
  {
  case 0x00:
    // Serial.printf("SSSSSSSSSS 串口乱入 SSSSSSSSSS\n");
    break;

  case 0x51:              // IR数据读取
    Get_IR_Data(IR_Data); //更新IR数据
    Tx_Data[0] = 0x51;
    Tx_Data[1] = IR_Data[0];
    Tx_Data[2] = IR_Data[1];
    Tx_Data[3] = 0x00;
    Tx_Data[4] = 0x00;
    // Serial.printf("SSSSSSSSSS ___ IR_Data[0] : %d ___ IR_Data[1] : %d ___ SSSSSSSSSS\n",IR_Data[0],IR_Data[1]);
    break;

  case 0x52:                 // 获取US数据 16进制两个8位
    US.Get_US_Data(US_Data); //更新US数据
    Tx_Data[0] = 0x52;
    Tx_Data[1] = US_Data[0];
    Tx_Data[2] = US_Data[1] >> 8;
    Tx_Data[3] = 0x00;
    Tx_Data[4] = 0x00;
    // Serial.printf("SSSSSSSSSS ___ US_Data[0] : %x ___ US_Data[1] : %x ___ SSSSSSSSSS\n",US_Data[0],US_Data[1]);
    break;

  case 0x53: // 获取颜色传感器数据 RGBC分4个8位传
    Colour_Sensor.Get_RGBC_Data(RGBC);
    Colour_Sensor.RGBtoHSV(RGBC, HSV); // 计算HSV

    Tx_Data[0] = 0x53;
    Tx_Data[1] = RGBC[0] >> 4; //R >> 8
    Tx_Data[2] = RGBC[1] >> 4; //G >> 8
    Tx_Data[3] = RGBC[2] >> 4; //B >> 8
    // Tx_Data[4] = RGBC[3] >> 4;  //C >> 8
    Tx_Data[4] = (uint8_t)HSV[0]; //H

    // Serial.printf("SSSSSSSSSS R : %d ___ G : %d ___ B : %d ___ C : %d SSSSSSSSSS\n",RGBC[0],RGBC[1],RGBC[2],RGBC[3]);  //(RED)(GREEN)(BLUE)(CLEAR)
    // Serial.printf("SSSSSSSSSS H  %f ___ S : %f ___ V : %f ___ 颜色 : %d SSSSSSSSSS\n",HSV[0],HSV[1],HSV[2],Colour_Num);  //HSV
    break;

  case 0x54: //获取电池电压数据
    uint32_t Battery_Data;
    Battery_Data = Get_Battery_Data();

    Tx_Data[0] = 0x54;
    Tx_Data[1] = Battery_Data >> 8; //读取一次电池电压
    Tx_Data[2] = Battery_Data;
    break;

  case 0x55: //获取固件版本号
    Tx_Data[0] = 0x55;

    // 复制版本号的“整数”和“小数” 到 发送Buffer
    Tx_Data[1] = Version_FW[0];
    Tx_Data[2] = Version_FW[1];
    Tx_Data[3] = Version_FW[2];
    Tx_Data[4] = Version_FW[3];

    break;

  case 0xA1:                       //蓝牙命名  //仅限通过蓝牙，串口不支持重命名
    Thunder_BLE.Write_BLE_Name(ADD_BLE_NAME); //从地址0开始写入命名的蓝牙
    break;

  case 0xA2:                            //控制舵机
    Servo_Turn(Rx_Data[1], Rx_Data[2]); //参数1：1-->机械臂，2-->机械爪；参数2：角度[%](0~100)
    break;

  case 0xA3:                       //控制声音播放
    Speaker.Play_Song(Rx_Data[1]); //播放第编号段音频
    Speaker.Set_Sound_Volume(Rx_Data[2]);
    break;

  case 0xB1:            //控制单个电机
    Disable_En_Motor(); // En_Motor_Flag = 0;

    Thunder_Motor.Motor_Move(Rx_Data[1], Rx_Data[2], Rx_Data[3]); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
    break;

  case 0xB2:            //控制两个电机
    Disable_En_Motor(); // En_Motor_Flag = 0;

    Thunder_Motor.Motor_Move(1, Rx_Data[1], Rx_Data[2]); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
    Thunder_Motor.Motor_Move(2, Rx_Data[3], Rx_Data[4]); //参数1 --> 电机编号；参数2 --> 速度(0-255)；参数3 -->方向
    break;

  case 0xB3:           //控制单个闭环电机
    Enable_En_Motor(); // En_Motor_Flag = 1;

    if (Rx_Data[1] == 1)
    {
      if (Rx_Data[3] == 1)
      {
        Thunder_Motor.Set_L_Target(Rx_Data[2]);
      }
      else
      {
        Thunder_Motor.Set_L_Target((-1) * Rx_Data[2]);
      }
    }
    else
    {
      if (Rx_Data[3] == 1)
      {
        Thunder_Motor.Set_R_Target(Rx_Data[2]);
      }
      else
      {
        Thunder_Motor.Set_R_Target((-1) * Rx_Data[2]);
      }
    }
    break;

  case 0xB4:           //控制两个闭环电机
    Enable_En_Motor(); // En_Motor_Flag = 1;

    if (Rx_Data[2] == 1)
    {
      Thunder_Motor.Set_L_Target(Rx_Data[1]);
    }
    else
    {
      Thunder_Motor.Set_L_Target((-1) * Rx_Data[1]);
    }

    if (Rx_Data[4] == 1)
    {
      Thunder_Motor.Set_R_Target(Rx_Data[3]);
    }
    else
    {
      Thunder_Motor.Set_R_Target((-1) * Rx_Data[3]);
    }
    break;

  case 0xB5: //获取当前电机速度(编码器计数值)  需要在Enable_En_Motor()状态下
    L_Speed = Thunder_Motor.Get_L_Speed();
    R_Speed = Thunder_Motor.Get_R_Speed();

    Tx_Data[0] = 0xB5;
    if (L_Speed >= 0)
    {
      Tx_Data[1] = L_Speed;
      Tx_Data[2] = 1;
    }
    else
    {
      Tx_Data[1] = (-1) * L_Speed;
      Tx_Data[2] = 2;
    }

    if (R_Speed >= 0)
    {
      Tx_Data[3] = R_Speed;
      Tx_Data[4] = 1;
    }
    else
    {
      Tx_Data[3] = (-1) * R_Speed;
      Tx_Data[4] = 2;
    }
    // Serial.printf("L_Speed:%d; R_Speed:%d; \n",Thunder_Motor.Get_L_Speed(),Thunder_Motor.Get_R_Speed());
    break;

  case 0xC1:                                                              //控制单个彩色灯颜色
    I2C_LED.Set_LED_Data(Rx_Data[1], Rx_Data[2], Rx_Data[3], Rx_Data[4]); //第几个灯(1开始)，R,G,B
    I2C_LED.LED_Updata();                                                 //按照现有数据刷新
    break;

  case 0xC2:                                                        //控制左侧彩色灯颜色
    I2C_LED.Set_LEDs_Data(1, I2C_LED_BUFF1, sizeof(I2C_LED_BUFF1)); //写入多个寄存器数据
    I2C_LED.LED_Updata();                                           //按照现有数据刷新
    break;
  case 0xC3:                                                        //控制右侧彩色灯颜色
    I2C_LED.Set_LEDs_Data(7, I2C_LED_BUFF2, sizeof(I2C_LED_BUFF2)); //写入多个寄存器数据
    I2C_LED.LED_Updata();                                           //按照现有数据刷新
    break;

  case 0xD1: //控制单色点阵灯开关
    LED_BUFF_Dot[Rx_Data[1]] = Rx_Data[2];
    HT16D35B.LED_Show(LED_BUFF_Dot, sizeof(LED_BUFF_Dot));
    break;

  case 0xD2: //显示预设的单色点阵灯图案
    Dot_Matrix_LED.Play_LED_HT16F35B_Show(Rx_Data[1]);
    break;

  case 0xD3: //单色点阵灯一次性刷新前半部分灯
    HT16D35B.LED_Show(LED_BUFF_Dot, sizeof(LED_BUFF_Dot));
    break;
  case 0xD4: //单色点阵灯一次性刷新后半部分灯
    HT16D35B.LED_Show(LED_BUFF_Dot, sizeof(LED_BUFF_Dot));
    break;

  case 0xE1: //内置单色点阵图案动画显示命令
    Set_LED_Show_No(Rx_Data[1]);
    break;

  case 0x66: //_______________________ Demo ___________________
    if (Rx_Data[1] == 1)
    {
      // Serial.printf("* 巡线 *\n");
      // 使用开环控制电机，然后在巡线里面 以偏离黑线的时间长度作为参量 做速度闭环控制
      Disable_En_Motor(); // En_Motor_Flag = 0;
      Line_Tracing();
    }
    Stop_All();
    break;

  default:
    Serial.printf("# No cmd#\n");
    break;
  }
  Reset_Rx_Data();
}

// 清空接收数据
void THUNDER::Reset_Rx_Data()
{
  for (int i = 0; i < 6; i++)
  {
    Rx_Data[i] = 0;
  }
}

/* 
 * 开环控制电机时，获取编码器数据，用于测试电机编码器(一个PID控制周期才能获取一次)
 * 数据 通过队列发到打印线程
 *   打印线程在loop里面
 */
void THUNDER::Get_Queue_Encoder(void)
{
  if (xSemaphoreTake(Timer_PID_Flag, portMAX_DELAY) == pdTRUE) // 控制周期PID_dt[ms]
  {
#ifdef PRINT_DEBUG_INFO
    float F_encoder_left;
    float F_encoder_right;

    // Serial.printf("left: %d\n", (int)Encoder_Counter_Left);
    F_encoder_left = Encoder_Counter_Left;
    xQueueSend(Task_Mesg.Queue_encoder_left, &F_encoder_left, 0);

    // Serial.printf("right: %d\n\n", (int)Encoder_Counter_Right);
    F_encoder_right = Encoder_Counter_Right;
    xQueueSend(Task_Mesg.Queue_encoder_right, &F_encoder_right, 0);
#endif
  }
}

/* 
 * I2C端口选通，变量channelData 相应位(每一bit代表一个通道) 为1 是 选通，可以多通道选通
 * 
 * @parameter: 
 * @return: 返回的是IIC 操作状态码，0 为成功， 非0 为其他状态
 */
uint8_t THUNDER::Set_I2C_Chanel(uint8_t channelData)
{
  uint8_t ret;
  uint8_t regValue;

  // 保证初始值与channelData 不一致
  regValue = (channelData == 0) ? 0xff : 0;
  // 重复连接 IIC 扩展芯片
  for (uint8_t i = 0; i < 2; i++)
  {
    // TCA9548的地址是 0x70, 因为它的地址位A0 A1 A2都接地了

    Task_Mesg.Take_Semaphore_IIC();
    Wire.beginTransmission(0x70);
    Wire.write(channelData);
    ret = Wire.endTransmission(true);
    #ifdef COMPATIBILITY_OLD_ESP_LIB
    if (ret == I2C_ERROR_BUSY)
    {
      Wire.reset();
    }
    #endif
    Task_Mesg.Give_Semaphore_IIC();
    if (ret != 0)
    {
      I2C_channel_opened = 0x00;
      Serial.printf("### TCA9548 Write I2C Channel Error: %d \n", ret);
      delay(100);
    }
    else
    {
      // read TCA9548
      Task_Mesg.Take_Semaphore_IIC();
      if( 0 != Wire.requestFrom((byte)0x70, (byte)1, (byte)true) ){
        while (Wire.available())
        {
          regValue = Wire.read();
        }
      }
      Task_Mesg.Give_Semaphore_IIC();

      if (regValue == channelData)
      {
        I2C_channel_opened = channelData;
        break;
      }
      else
      {
        Serial.println("### TCA9548 Read not equal Write");
        delay(200);
      }
    }
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
uint8_t THUNDER::Select_Sensor_Channel(uint8_t sensorChannel)
{
  uint8_t ret;

  switch (sensorChannel)
  {
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
uint8_t THUNDER::Select_Sensor_AllChannel()
{
  uint8_t ret;

  // reset
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(10);
  digitalWrite(15, HIGH);
  delay(10);

  ret = Set_I2C_Chanel(0x3f); //全选通

  if (ret != 0)
  {
    return 1;
  }

  return 0;
}
