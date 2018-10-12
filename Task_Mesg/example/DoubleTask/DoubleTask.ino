#include <Thunder_lib.h>
#include <Task_Mesg.h>

void setup() {
  // put your setup code here, to run once:
  Thunder.Setup_All();

  // 舵机位置初始化
  Thunder.Servo_Turn(1, 90);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)
  Thunder.Servo_Turn(2, 90);  //参数1--> 舵机编号；参数2 --> 角度[%](0~180)

  // 新建 loop 和 启动后台线程
  Task_Mesg.Create_New_Loop();
  Task_Mesg.Create_Deamon_Threads();
}

void loop() {

  Serial.printf("left rotate: %d\n", Thunder_Motor.Get_L_RotateValue());
  Serial.printf("Right rotate: %d\n\n", Thunder_Motor.Get_R_RotateValue());

  delay(3000);
}

/***********************************new loop**************************************/
void setup_1()
{

}

//超声波数据
float US_Data_cm = 0;
void loop_1()
{
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
