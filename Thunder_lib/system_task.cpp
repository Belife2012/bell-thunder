#include "system_task.h"
#include "iic_thunder.h"
#include "bell_thunder.h"

#ifdef COMPETITION_FW_001
  #define MAX_APPS_TASK_COUNTER_AUTOCTRL  (MAX_APPS_TASK_COUNTER)

  TaskHandle_t Task_Apps_AutoCtrl[MAX_APPS_TASK_COUNTER_AUTOCTRL];
  struct_Apps_Param *task_param_AutoCtrl[MAX_APPS_TASK_COUNTER_AUTOCTRL];

  uint8_t tasks_num_AutoCtrl;
  uint8_t competition_action_status = 0;
#endif

SYSTEM_TASK System_Task;
uint32_t led_indication_counter;

/**************************************Deamon Thread*******************************************/
void Driver_Flush(void *pvParameters)
{
  uint32_t current_time;
  uint32_t character_roll_time;
  uint32_t battery_measure_time;
  uint32_t color_led_ctrl_time;
  // uint32_t communications_time;

  character_roll_time = millis();
  battery_measure_time = millis();
  color_led_ctrl_time = millis();
  // communications_time = millis();
  for (;;)
  {
    current_time = millis();
    // 
    /* if (System_Task.Get_flush_Tasks() & (0x00000001 << FLUSH_COMMUNICATIONS))
    {
      if(current_time - communications_time > 50){
        Bell_Thunder.Check_UART_Communication();
        communications_time = millis();
      }
    } */
    // 
    if (System_Task.Get_flush_Tasks() & (0x00000001 << FLUSH_MATRIX_LED))
    {
      Display_Screen.Animation_Control();
    }
    // 
    if (System_Task.Get_flush_Tasks() & (0x00000001 << FLUSH_CHARACTER_ROLL))
    {
      if (current_time - character_roll_time > 150)
      {
        Display_Screen.Play_String_NextFrame();
        character_roll_time = millis();
      }
    }
    // 
    if (System_Task.Get_flush_Tasks() & (0x00000001 << FLUSH_COLOR_LED))
    {
      if(current_time - color_led_ctrl_time > 30){
        LED_Color.LED_Flush();
        color_led_ctrl_time = millis();
      }
    }
    // 
    if (System_Task.Get_flush_Tasks() & (0x00000001 << FLUSH_BATTERY_MEASURE))
    {
      if (current_time - battery_measure_time > 300)
      {
        Bell_Thunder.Get_Battery_Data();
        battery_measure_time = millis();
      }
    }

    // 每5ms进行一次查询
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
void Deamon_Motor(void *pvParameters)
{
  for (;;)
  {
    if (System_Task.Get_flush_Tasks() & (0x00000001 << FLUSH_MOTOR_PID_CTRL))
    {
      Bell_Thunder.En_Motor();
    }
    if(Bell_Thunder.line_tracing_running == true){
      vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD));
    }else{
      vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD));
    }
  }
}
void Proc_Ble_Command(void *pvParameters)
{
  for (;;)
  {
    // 等待 xSemaphore_BLE, 如果没有give xSemaphore_BLE, 这个task是阻塞状态的
    // Serial.println("wait BLE command...");
    int ble_mesg_type;
    ble_mesg_type = BLE_THUNDERGO::Take_Semaphore_BLE();
    if (System_Task.Get_flush_Tasks() & (0x00000001 << FLUSH_COMMUNICATIONS))
    {
      // 只有标志了 FLUSH_COMMUNICATIONS，才解析 BLE数据
      if(ble_mesg_type == BLE_SERVER_SEMAPHORE_RX){ // 作为Server，接收到client的信息
        Bell_Thunder.Check_BLE_Communication();
      }else if(ble_mesg_type == BLE_CLIENT_SEMAPHORE_CONN){
        // if(BLE_THUNDERGO::GetBleConnectType() == BLE_CLIENT_DISCONNECT)
        { // 搜索到BLE Server，进行连接动作
          BLE_Client.Connect_Ble_Server();
        }
        // else if(BLE_THUNDERGO::GetBleConnectType() == BLE_CLIENT_CONNECTED){ // 接收到Server的信息
        //   Bell_Thunder.Check_BLE_Communication();
        // }
      }
    }
  }
}
void Proc_Uart_Command(void *pvParameters)
{
  for (;;)
  {
    if (System_Task.Get_flush_Tasks() & (0x00000001 << FLUSH_COMMUNICATIONS))
    {
      Bell_Thunder.Check_UART_Communication();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
void Operator_Mode_Deamon(void *pvParameters)
{
  for(;;)
  {
    if(Bell_Thunder.line_tracing_running == true){
      Bell_Thunder.Line_Tracing();
      // Bell_Thunder.Line_Tracing_Speed_Ctrl();
    }
    // 作为辅线程搜索BLE Server
    if(BLE_THUNDERGO::GetBleConnectType() == BLE_CLIENT_DISCONNECT){
      vTaskDelay(pdMS_TO_TICKS(1000));
      BLE_Client.Scan_Ble_Server(); // 搜索BLE Server
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
/* 
 * 轮询 按键、指示灯LED 等等交互的外设
 * 巡查周期为10ms
 * 
 * @parameters: 
 * @return: 
 */
void Polling_Check(void *pvParameters)
{
  // 单位为ms
  led_indication_counter = 0;
#ifdef COMPETITION_FW_001
  static uint32_t timer_competition = 0;
#endif

  for(;;)
  {
    /* 指示灯LED轮询，模拟PWM */
    Bell_Thunder.Update_Function_Timer();
    Bell_Thunder.Update_Led_Indication_Status(led_indication_counter);
    Bell_Thunder.Check_Function_Button_Value();

    vTaskDelay(pdMS_TO_TICKS(POLLING_CHECK_PERIOD));
    led_indication_counter += POLLING_CHECK_PERIOD;
#ifdef COMPETITION_FW_001
    if(thunder_system_parameter == 1){
      if(competition_action_status == 1){
        if(timer_competition > 40000){
          Clear_All_Loops_AutoCtrl();
        }else{
          timer_competition += POLLING_CHECK_PERIOD;
        }
      }else if(competition_action_status == 2){
        if(timer_competition > 160000){
          ESP.restart();
        }else{
          timer_competition += POLLING_CHECK_PERIOD;
        }
      }
    }
#endif

#ifdef PRINT_SYS_MANAGER_INFO
    Serial.print("Free Heap: ");
    Serial.println(ESP.getFreeHeap());
#endif

  }
}

/************************************new app Thread*****************************************/
void Programs_System(void)
{
  if(BELL_THUNDER::program_change_to == PROGRAM_USER_1)
  {
#ifdef COMPETITION_FW_001
    if(thunder_system_parameter == 1){
      BLE_Remoter.Enable_Remote(false);
      System_Task.Toggle_Competition_Status(2);
      Program_AutoCtrl();
    }else{
      Program_1();
    }
#else
    Program_1();
#endif
    BELL_THUNDER::program_change_to = PROGRAM_RUNNING;
  }
  else if(BELL_THUNDER::program_change_to == PROGRAM_USER_2)
  {
    Program_2();
    BELL_THUNDER::program_change_to = PROGRAM_RUNNING;
  }
  else if(BELL_THUNDER::program_change_to == PROGRAM_USER_3)
  {
    Program_3();
    BELL_THUNDER::program_change_to = PROGRAM_RUNNING;
  }
  else if(BELL_THUNDER::program_change_to == PROGRAM_USER_4)
  {
    Program_4();
    BELL_THUNDER::program_change_to = PROGRAM_RUNNING;
  }
  else if(BELL_THUNDER::program_change_to == PROGRAM_THUNDER_GO)
  {
    Program_ThunderGo();
    BELL_THUNDER::program_change_to = PROGRAM_RUNNING;
  }
}

void New_Loop_Task(void *pvParameters)
{
  ( (struct_Apps_Param *)pvParameters )->Mysetup();
  for (;;)
  {
    ( (struct_Apps_Param *)pvParameters )->Myloop();
  }
}

SYSTEM_TASK::SYSTEM_TASK()
{
  // BUG（arduino-esp32更新到1.0.2后改正了）: 删除会影响到BLE的Advertise
  // xQueueCreate(3, sizeof(float));
  // xQueueCreate(3, sizeof(float));

  for (uint8_t i = 0; i < MAX_APPS_TASK_COUNTER; i++)
  {
    Task_Apps[i] = NULL;
    task_param[i] = NULL;
    
#ifdef COMPETITION_FW_001
    Task_Apps_AutoCtrl[i] = NULL;
    task_param_AutoCtrl[i] = NULL;
#endif
  }

  vPortCPUInitializeMutex(&spinlockMUX);

  tasks_num = 0;
  flush_Tasks = 0;
  deamon_task_running = 0;
}

SYSTEM_TASK::~SYSTEM_TASK()
{
}

/* 
 * 进入临界区
 * 
 * @parameters: 
 * @return: 
 */
void SYSTEM_TASK::Enter_Task_Critical()
{
  portENTER_CRITICAL(&spinlockMUX);
}

/* 
 * 退出临界区
 * 
 * @parameters: 
 * @return: 
 */
void SYSTEM_TASK::Exit_Task_Critical()
{
  portEXIT_CRITICAL(&spinlockMUX);
}

/*
 * 创建多一个线程，新的loop_1和 setup_1就可以使用了
 * 
 * @parameters: 
 * @return: 返回0表示创建成功
 *          返回1表示创建数目已经达到最大值，不能继续创建
 */
uint8_t SYSTEM_TASK::Create_New_Loop(uint8_t program_sequence, 
                  func_Program_Setup program_setup, func_Program_Loop program_loop )
{
  if (tasks_num >= MAX_APPS_TASK_COUNTER)
  {
    Serial.println("ERROS: Apps amount had reached max");
    return 1;
  }

  task_param[tasks_num] = new struct_Apps_Param();
  task_param[tasks_num]->sequence = program_sequence;
  task_param[tasks_num]->index = tasks_num;
  task_param[tasks_num]->Mysetup = program_setup;
  task_param[tasks_num]->Myloop = program_loop;
  /***create New tasks, Priority: 1~7 ***/
  xTaskCreatePinnedToCore(New_Loop_Task, "newLoopTask", 8192, (void *)task_param[tasks_num], 1, &Task_Apps[tasks_num], 1);
  tasks_num++;

  return 0;
}

#ifdef COMPETITION_FW_001

uint8_t Create_New_Loop_AutoCtrl(uint8_t program_sequence, 
                  func_Program_Setup program_setup, func_Program_Loop program_loop )
{
  if (tasks_num_AutoCtrl >= MAX_APPS_TASK_COUNTER)
  {
    Serial.println("ERROS: Apps amount had reached max");
    return 1;
  }

  task_param_AutoCtrl[tasks_num_AutoCtrl] = new struct_Apps_Param();
  task_param_AutoCtrl[tasks_num_AutoCtrl]->sequence = program_sequence;
  task_param_AutoCtrl[tasks_num_AutoCtrl]->index = tasks_num_AutoCtrl;
  task_param_AutoCtrl[tasks_num_AutoCtrl]->Mysetup = program_setup;
  task_param_AutoCtrl[tasks_num_AutoCtrl]->Myloop = program_loop;
  /***create New tasks, Priority: 1~7 ***/
  xTaskCreatePinnedToCore(New_Loop_Task, "newLoopTask", 8192, 
                          (void *)task_param_AutoCtrl[tasks_num_AutoCtrl], 1, 
                          &Task_Apps_AutoCtrl[tasks_num_AutoCtrl], 1);
  tasks_num_AutoCtrl++;

  return 0;
}

void SYSTEM_TASK::Toggle_Competition_Status(int status_index)
{
  if(status_index == 1){
    byte colorData[36] = {40, 25, 0, 40, 25, 0, 40, 25, 0, 40, 25, 0, 40, 25, 0, 40, 25, 0,
                          40, 25, 0, 40, 25, 0, 40, 25, 0, 40, 25, 0, 40, 25, 0, 40, 25, 0};
    LED_Color.Set_LEDs_Data(0x01, colorData, sizeof(colorData));
    LED_Color.LED_Updata();

    Speaker_Thunder.Play_Song(99);
  }else if(status_index == 2){
    competition_action_status = 1;

    byte colorData[36] = {50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0,
                          50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0};
    LED_Color.Set_LEDs_Data(0x01, colorData, sizeof(colorData));
    LED_Color.LED_Updata();

    Speaker_Thunder.Play_Song(104);
  }else if(status_index == 3){
    competition_action_status = 2;

    Bell_Thunder.Toggle_Led_mode(1000, 100, 100, 2);

    byte colorData[36] = {0, 25, 0, 0, 25, 0, 0, 25, 0, 0, 25, 0, 0, 25, 0, 0, 25, 0,
                          0, 25, 0, 0, 25, 0, 0, 25, 0, 0, 25, 0, 0, 25, 0, 0, 25, 0};
    LED_Color.Set_LEDs_Data(0x01, colorData, sizeof(colorData));
    LED_Color.LED_Updata();

    Speaker_Thunder.Play_Song(101);
  }
}
#endif

void SYSTEM_TASK::Clear_All_Loops()
{
  uint8_t ret;

  SENSOR_IIC::Take_Semaphore_IIC();// 拿到资源控制权才开始删除线程
  for (uint8_t i = 0; i < MAX_APPS_TASK_COUNTER; i++)
  {
    if(Task_Apps[i] != NULL)
    {
      vTaskDelete(Task_Apps[i]);
      Task_Apps[i] = NULL;
    }
    if(task_param[i] != NULL)
    {
      delete task_param[i];
      task_param[i] = NULL;
    }
  }
  SENSOR_IIC::Give_Semaphore_IIC();

  tasks_num = 0;
  Bell_Thunder.Reset_All_Components();
  Bell_Thunder.Set_Ble_Type(BLE_TYPE_CLIENT); // 每次等待启动时，都是处于 BLE Client
}

#ifdef COMPETITION_FW_001
void Clear_All_Loops_AutoCtrl()
{
  SENSOR_IIC::Take_Semaphore_IIC();
  for (uint8_t i = 0; i < MAX_APPS_TASK_COUNTER; i++)
  {
    if(Task_Apps_AutoCtrl[i] != NULL)
    {
      vTaskDelete(Task_Apps_AutoCtrl[i]);
      Task_Apps_AutoCtrl[i] = NULL;
    }
    if(task_param_AutoCtrl[i] != NULL)
    {
      delete task_param_AutoCtrl[i];
      task_param_AutoCtrl[i] = NULL;
    }
  }
  SENSOR_IIC::Give_Semaphore_IIC();

  tasks_num_AutoCtrl = 0;
  Bell_Thunder.Stop_All();

  // 创建遥控阶段的线程
  Program_1();
  BLE_Remoter.Enable_Remote(true);
  System_Task.Toggle_Competition_Status(3);
}
#endif

/*
 * 创建后台线程，守护电机PID控制和检测通信数据/屏幕刷新控制：
 * 一个用于需要定时刷新的功能，要求实时性不高
 * 另外一个用于电机PID固定调节周期使用
 * 
 * @parameters: 
 * @return: 
 */
void SYSTEM_TASK::Create_Deamon_Threads()
{
  // deamon task已经开始，不能重复创建
  if (deamon_task_running == 1)
  {
    return;
  }
  // deamon Tasks , Priority: 8~10
  Serial.println("\nCreating deamons...");
  xTaskCreatePinnedToCore(Driver_Flush, "DriverFlush", 8192, NULL, 2, NULL, 1);
  Serial.println("1");
  xTaskCreatePinnedToCore(Deamon_Motor, "deamonMotor", 4096, NULL, 5, NULL, 1);
  Serial.println("2");
  xTaskCreatePinnedToCore(Proc_Ble_Command, "BleCommand", 4096, NULL, 3, NULL, 1);
  Serial.println("3");
  xTaskCreatePinnedToCore(Proc_Uart_Command, "UartCommand", 4096, NULL, 3, NULL, 1);
  Serial.println("4");
  xTaskCreatePinnedToCore(Operator_Mode_Deamon, "operDeamon", 4096, NULL, 1, NULL, 1);
  Serial.println("5");
  xTaskCreatePinnedToCore(Polling_Check, "pollCheck", 4096, NULL, 2, NULL, 1);
  Serial.println("6");

  deamon_task_running = 1;
  Serial.println("Deamons ready");
}

/*
 * 获得后台线程的标识变量；
 * 根据返回的数据可以得出有哪些后台线程在运行
 */
UBaseType_t SYSTEM_TASK::Get_flush_Tasks()
{
  return flush_Tasks;
}

/*
 * 设置某个刷新线程 在后台守护线程运行
 * 
 * @parameters: 
 *      0 通信检查，Thunder.Check_Communication();
 *      1 LED点阵显示刷新，Thunder.LED_Show();
 *      ......
 * @return: 
 */
void SYSTEM_TASK::Set_Flush_Task(byte flushType)
{
  if (flushType >= FLUSH_MAX_NUM)
  {
    return;
  }
  else
  {
    flush_Tasks |= (0x00000001 << flushType);
  }
}
/*
 * 移除某个刷新线程 在后台守护线程运行
 * 
 * @parameters: 
 *      0 通信检查，Thunder.Check_Communication();
 *      1 LED点阵显示刷新，Thunder.LED_Show();
 *      ......
 * @return: 
 */
void SYSTEM_TASK::Remove_Flush_Task(byte flushType)
{
  if (flushType >= FLUSH_MAX_NUM)
  {
    return;
  }
  else
  {
    flush_Tasks &= ~(0x00000001 << flushType);
  }
}
