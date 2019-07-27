#include <Thunder_lib.h>

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  while (!Serial);
  
  Wire.begin(SDA_PIN, SCL_PIN, 100000); //Wire.begin();
  // 1.9版的电路板增加了IIC扩展芯片，初始化需要调用此函数进行IIC初始化
  Thunder.Select_Sensor_AllChannel();

  // 一定要有调用配置 Setup_Motor_PID()，电机旋转量记录才有效
  Thunder_Motor.Setup_Motor_PID();

  // 初始化单色LED驱动IC配置，才有LED点阵显示
  Dot_Matrix_LED.Setup();   // 初始化单色LED驱动IC配置

  // 新建 loop 后 函数setup_1() loop_1()才有效
  Task_Mesg.Create_New_Loop();

  // 开启后台守护线程
  Task_Mesg.Set_Flush_Task(1);
  Task_Mesg.Create_Deamon_Threads();
}

void loop() {

  Serial.printf("left rotate: %d\n", Thunder_Motor.Get_L_RotateValue());
  Serial.printf("Right rotate: %d\n\n", Thunder_Motor.Get_R_RotateValue());

  Thunder.Set_LED_Show_No(5);

  delay(3000);
}

/****************************setup_1()、loop_1()**************************************/
void setup_1()
{

}

//超声波数据
float US_Data_cm = 0;
void loop_1()
{
  // 获取两个超声波模块的数据
  Thunder.Select_Sensor_Channel(1);
  Serial.println("Port 1: ");
  US_Data_cm = US.Get_US_cm();
  if(US_Data_cm < 1.0){
    Serial.printf("### US_Data_cm : %.1f [cm]###############################\n", US_Data_cm);
  }else{
    Serial.printf("*** US_Data_cm : %.1f [cm]***\n",US_Data_cm);
  }

  Thunder.Select_Sensor_Channel(2);
  Serial.println("Port 2: ");
  US_Data_cm = US.Get_US_cm();
  if(US_Data_cm < 1.0){
    Serial.printf("### US_Data_cm : %.1f [cm]###############################\n", US_Data_cm);
  }else{
    Serial.printf("*** US_Data_cm : %.1f [cm]***\n\n",US_Data_cm);
  }


  delay(1000);
}
