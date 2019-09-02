#include <Arduino.h>
#include "mult_devices.h"

MULT_DEVICES *Mult_Devices = NULL;

// #define DEBUG_MASTER_RX

#define TAKE_MESG_UART_MUTEX	do{} while(pdPASS != xSemaphoreTake(mutex_mesg_uart, portMAX_DELAY))
#define GIVE_MESG_UART_MUTEX	do{xSemaphoreGive(mutex_mesg_uart);} while(0)

MULT_DEVICES::MULT_DEVICES()
{
}

MULT_DEVICES::~MULT_DEVICES()
{
}

void MULT_DEVICES::Uart_Init(void)
{
	Serial1.begin(THUNDER_MULTI_BAUD, SERIAL_8N1, 32, 18);
	Serial1.write(0x55);
	Serial1.write(0x55);
	Serial1.write(0x55);
	Serial1.write(0x55);
	Serial1.write(0x55);
	pinMode(32, INPUT_PULLUP);
}
void MULT_DEVICES::Uart_Close(void)
{
	Uart_ClearRxBuf();
	Serial1.end();
}

void MULT_DEVICES::uart_sendByte(unsigned char dat) 
{ 
		Serial1.write(dat);
} 

//通过串口发送 1 个字节的数据，dat 为发送的数据 
int MULT_DEVICES::uart_recByte(unsigned char* readValue) 
{ 
	unsigned int timeOut;
	int flag; 

	timeOut = millis();
	while(!Serial1.available()){
		if(timeOut + RECEIVE_TIMEOUT < millis()){
			return -1;
		}
		yield();
	}
	*readValue = Serial1.read();

	return Serial1.available(); 
	
}

int MULT_DEVICES::GetRxFIFOCnt()
{
	return Serial1.available();
}

void MULT_DEVICES::Uart_ClearRxBuf(void)
{
	while(Serial1.available()){
		Serial1.read();
	}
	Serial1.flush();
}
void MULT_DEVICES::Uart_WaitClear(void)
{
	Serial.flush();
}

unsigned char MULT_DEVICES::GetInterruptFlag()
{
	unsigned char read_value;

	read_value = Wk2114ReadReg(WK2XXX_GPORT,WK2XXX_GIFR);

	return read_value;
}

/**************************************Wk2114WriteReg***********************************/
//函数功能：写寄存器函数（前提是该寄存器可写，某些寄存器如果你写1，可能会自动置1，具体见数据手册)
//参数：port:为子串口的数(C0C1)
//      reg:为寄存器的地址(A3A2A1A0)
//      dat:为写入寄存器的数据
//注意：在子串口被打通的情况下，向FDAT写入的数据会通过TX引脚输出
//*************************************************************************/
void MULT_DEVICES::Wk2114WriteReg(unsigned char port,unsigned char reg,unsigned char dat)
{	 
	TAKE_MESG_UART_MUTEX;
	 uart_sendByte(((port-1)<<4)+reg);	//写指令，对于指令的构成见数据手册
	 uart_sendByte(dat);//写数据
	GIVE_MESG_UART_MUTEX;
}


/*************************************Wk2114ReadReg************************************/
//函数功能：读寄存器函数
//参数：port:为子串口的数(C0C1)
//      reg:为寄存器的地址(A3A2A1A0)
//      rec_data:为读取到的寄存器值
//注意：在子串口被打通的情况下，读FDAT，实际上就是读取uart的rx接收的数据
/*************************************************************************/
unsigned char MULT_DEVICES::Wk2114ReadReg(unsigned char port,unsigned char reg)
{	 
    unsigned char rec_data;

	TAKE_MESG_UART_MUTEX;
    uart_sendByte(0x40+((port-1)<<4)+reg); //写指令，对于指令的构成见数据手册
	uart_recByte(&rec_data);	 //接收返回的寄存器值	
	GIVE_MESG_UART_MUTEX;			

	return rec_data;
}



/**************************************Wk2114writeFIFO***********************************/
//函数功能：写FIFO函数（该函数写入的数据会通过uart的TX发送出去)
//参数：port:为子串口的数(C0C1)
//      *dat：为写入数据指针
//      num：为写入数据的个数，不超过16个字节（N3N2N1N0）
/*************************************************************************/
int MULT_DEVICES::Wk2114writeFIFO(unsigned char port,unsigned char *send_da,unsigned char num)
{	 
	unsigned char i;
	if( num > WK2XXX_TX_FIFO_SIZE ){
		return -1;
	}

	TAKE_MESG_UART_MUTEX;
	if(device_role == ROLE_MASTER){
		uart_sendByte(0x80+((port-1)<<4)+(num-1)); //写指令,对于指令构成见数据手册
	}

	for(i=0;i<num;i++)
	{
		uart_sendByte( *(send_da+i) );//写数据
	}
	GIVE_MESG_UART_MUTEX;

	 return i;
}
int MULT_DEVICES::Wk2114writeFIFO(unsigned char *send_da,unsigned char num)
{	 
	unsigned char i;
	if( num > WK2XXX_TX_FIFO_SIZE ){
		return -1;
	}

	TAKE_MESG_UART_MUTEX;
	for(i=0;i<num;i++)
	{
		uart_sendByte( *(send_da+i) );//写数据
	}
	GIVE_MESG_UART_MUTEX;

	 return i;
}

/***************************************Wk2114readFIFO**********************************/
//函数功能：读FIFO函数（该函数读取的数据是FIFO缓存中的数据，实际上是uart的rx接收的数据)
//参数：port:为子串口的数(C0C1)
//      *dat：为读到数据指针
//      num：为读出数据的个数，不超过16个字节（N3N2N1N0）
/*************************************************************************/
int MULT_DEVICES::Wk2114readFIFO(unsigned char port,unsigned char *rev_da,unsigned char num)
{
	unsigned char n;
	if( num > WK2XXX_RX_FIFO_SIZE ){
		return -1;
	}
	
	TAKE_MESG_UART_MUTEX;
	if(device_role == ROLE_MASTER){
		uart_sendByte(0xc0+((port-1)<<4)+(num-1));
	}

	for(n=0; n<num; n++)
	{	
		if( 0 > uart_recByte( rev_da+n ) ){
			break;
		}
	}
	GIVE_MESG_UART_MUTEX;

	return n;
}
int MULT_DEVICES::Wk2114readFIFO(unsigned char *rev_da,unsigned char num)
{
	unsigned char n;
	if( num > WK2XXX_RX_FIFO_SIZE ){
		return -1;
	}
	
	for(n=0; n<num; n++)
	{	
		if( 0 > uart_recByte( rev_da + n ) ){
			break;
		}
	}

	return n;
}

/******************************Wk2114Init*******************************************/
//函数功能：本函数主要会初始化一些芯片基本寄存器；
/*********************************************************************************/
void MULT_DEVICES::Wk2114Init(unsigned char port)
{
    unsigned char gena,grst,gier,sier,scr;
	//使能子串口时钟
    gena=Wk2114ReadReg(WK2XXX_GPORT,WK2XXX_GENA); 
	switch (port)
    {
          case 1://使能子串口1的时钟
              gena|=WK2XXX_UT1EN;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		  case 2://使能子串口2的时钟
              gena|=WK2XXX_UT2EN;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		   case 3://使能子串口3的时钟
              gena|=WK2XXX_UT3EN;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		   case 4://使能子串口4的时钟
              gena|=WK2XXX_UT4EN;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
	 }	
	//软件复位子串口
	grst=Wk2114ReadReg(WK2XXX_GPORT,WK2XXX_GRST); 
	switch (port)
    {
          case 1://软件复位子串口1
              grst|=WK2XXX_UT1RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		  case 2://软件复位子串口2
              grst|=WK2XXX_UT2RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		   case 3://软件复位子串口3
              grst|=WK2XXX_UT3RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		   case 4://软件复位子串口4
             grst|=WK2XXX_UT4RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
	 }	
  //使能子串口中断，包括子串口总中断和子串口内部的接收中断，和设置中断触点
	gier=Wk2114ReadReg(WK2XXX_GPORT,WK2XXX_GIER); 
	switch (port)
    {
          case 1://软件复位子串口1
              gier|=WK2XXX_UT1RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GIER,gier);
              break;
		  case 2://软件复位子串口2
              gier|=WK2XXX_UT2RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GIER,gier);
              break;
		   case 3://软件复位子串口3
              gier|=WK2XXX_UT3RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GIER,gier);
              break;
		   case 4://软件复位子串口4
              gier|=WK2XXX_UT4RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GIER,gier);
              break;
	 }	 
	 //使能子串口接收触点中断和超时中断
	 sier=Wk2114ReadReg(port,WK2XXX_SIER); 
	 sier |= WK2XXX_RFTRIG_IEN;
	 Wk2114WriteReg(port,WK2XXX_SIER,sier);
	 // 初始化FIFO和设置固定中断触点
	 Wk2114WriteReg(port,WK2XXX_FCR,0XFF);

	 //设置任意中断触点，如果下面的设置有效，那么上面FCR寄存器中断的固定中断触点将失效
	 Wk2114WriteReg(port,WK2XXX_SPAGE,1);//切换到page1
	 Wk2114WriteReg(port,WK2XXX_RFTL,PACKAGE_MIN_LEN+2);//设置接收触点
	 Wk2114WriteReg(port,WK2XXX_TFTL,0X10);//设置发送触点
	 Wk2114WriteReg(port,WK2XXX_SPAGE,0);//切换到page0 

	 //使能子串口的发送和接收使能
	 scr=Wk2114ReadReg(port,WK2XXX_SCR); 
	 scr|=WK2XXX_TXEN|WK2XXX_RXEN;
	 Wk2114WriteReg(port,WK2XXX_SCR,scr);
}
/******************************Wk2114Close*******************************************/
//函数功能：本函数会关闭当前子串口，和复位初始值；
/*********************************************************************************/

void MULT_DEVICES::Wk2114Close(unsigned char port)
{
    unsigned char gena,grst;
	//复位子串口
	grst=Wk2114ReadReg(WK2XXX_GPORT,WK2XXX_GRST); 
	switch (port)
    {
          case 1://软件复位子串口1
              grst&=~WK2XXX_UT1RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		  case 2://软件复位子串口2
              grst&=~WK2XXX_UT2RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		   case 3://软件复位子串口3
              grst&=~WK2XXX_UT3RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		   case 4://软件复位子串口4
              grst&=~WK2XXX_UT4RST;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
	 }	
	//关闭子串口时钟
    gena=Wk2114ReadReg(WK2XXX_GPORT,WK2XXX_GENA); 
	switch (port)
    {
          case 1://使能子串口1的时钟
              gena&=~WK2XXX_UT1EN;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		  case 2://使能子串口2的时钟
              gena&=~WK2XXX_UT2EN;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		   case 3://使能子串口3的时钟
              gena&=~WK2XXX_UT3EN;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		   case 4://使能子串口4的时钟
              gena&=~WK2XXX_UT4EN;
		      Wk2114WriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
	 }	
}
/**************************Wk2114SetBaud*******************************************************/
//函数功能：设置子串口波特率函数、此函数中波特率的匹配值是根据11.0592Mhz下的外部晶振计算的
// port:子串口号
// baud:波特率大小.波特率表示方式，
//
/**************************Wk2114SetBaud*******************************************************/
void MULT_DEVICES::Wk2114SetBaud(unsigned char port,int baud)
{  
	unsigned char baud1,baud0,pres,scr;
	//如下波特率相应的寄存器值，是在外部时钟为11.0592的情况下计算所得，如果使用其他晶振，需要重新计算
	switch (baud) 
	{
        case 9600:
			baud1=0x00;
			baud0=0x47;
			pres=0;
			break;
        case 19200:
			baud1=0x00;
			baud0=0x23;
			pres=0;
			break;
        case 57600:
			baud1=0x00;
			baud0=0x0b;
			pres=0;
            break;
        case 115200:
			baud1=0x00;
			baud0=0x05;
			pres=0;
			break;
        case 230400:
			baud1=0x00;
			baud0=0x02;
			pres=0;
			break;
        case 691200:
			baud1=0x00;
			baud0=0x00;
			pres=0;
			break;
        default:
			baud1=0x00;
			baud0=0x00;
			pres=0;
    }
	//关掉子串口收发使能
	scr=Wk2114ReadReg(port,WK2XXX_SCR); 
	Wk2114WriteReg(port,WK2XXX_SCR,0);
	//设置波特率相关寄存器
	Wk2114WriteReg(port,WK2XXX_SPAGE,1);//切换到page1
	Wk2114WriteReg(port,WK2XXX_BAUD1,baud1);
	Wk2114WriteReg(port,WK2XXX_BAUD0,baud0);
	Wk2114WriteReg(port,WK2XXX_PRES,pres);
	Wk2114WriteReg(port,WK2XXX_SPAGE,0);//切换到page0 
	//使能子串口收发使能
	Wk2114WriteReg(port,WK2XXX_SCR,scr);
	
	
}
/* 
 * 查询多机通信接口作为Master 还是 Slaver
 * 
 * @parameters: 
 * @return: ROLE_SLAVER/ROLE_MASTER
 */
int MULT_DEVICES::CheckMultiHost()
{
	Uart_Init();
	delay(10);
	Uart_ClearRxBuf();

	Wk2114WriteReg(WK2XXX_GPORT, WK2XXX_GENA, 0xF0);
	delay(5);

	unsigned char regValue;
	int backCode;
	
	TAKE_MESG_UART_MUTEX;
	uart_sendByte(0x40+WK2XXX_GENA); //写指令，对于指令的构成见数据手册
	backCode = uart_recByte(&regValue);	 //接收返回的寄存器值	
	GIVE_MESG_UART_MUTEX;

	// 判断读出来的值是否与 写入的一致
	if(backCode != -1 && regValue == 0xF0)
	{
		device_role = ROLE_MASTER;
		
		Wk2114Init(1);//初始化设置
		Wk2114SetBaud(1,THUNDER_MULTI_BAUD);//配置子串口波特率
		Wk2114Init(2);
		Wk2114SetBaud(2,THUNDER_MULTI_BAUD);
		Wk2114Init(3);
		Wk2114SetBaud(3,THUNDER_MULTI_BAUD);
		Wk2114Init(4);
		Wk2114SetBaud(4,THUNDER_MULTI_BAUD);
		delay(5);

	}else{
		device_role = ROLE_SLAVER;
	}

	return device_role;
}

int MULT_DEVICES::CheckPackage(unsigned char port)
{
	unsigned char recv_byte;
	unsigned char recv_index = 0;
	
	Wk2114readFIFO(port, (unsigned char *)&recv_head_info + recv_head_info.valid_cnt, 3 - recv_head_info.valid_cnt);
	for(int i=0; i<3; i++){
		if(recv_head_info.head_info[0] != PACKAGE_HEAD_1){
			if( i == 2 ){
				recv_head_info.valid_cnt = 0;
				return -1;
			}
			recv_head_info.head_info[0] = recv_head_info.head_info[1];
			recv_head_info.head_info[1] = recv_head_info.head_info[2];
		}else{
			if(i == 0){
				recv_head_info.valid_cnt = 0;//开始验证数据，清零本次的帧头信息
				break;
			}else{
				recv_head_info.valid_cnt = 3-i;
				return -2;
			}
		}
	}
	if(recv_head_info.head_info[1] != PACKAGE_HEAD_2){
		return -3;
	}
	if(recv_head_info.head_info[2] < PACKAGE_MIN_LEN
		|| recv_head_info.head_info[2] > PACKAGE_PAYLOAD_MAX_LEN + PACKAGE_MIN_LEN)
	{
		return -4;
	}
	unsigned char data_length = recv_head_info.head_info[2];

	recv_package.length = data_length;
	data_length -= 1; //剩余的数据的长度
	recv_index = 1;

	// 读取FIFO时，每次读取最多16字节
	while(data_length > 16){
		Wk2114readFIFO(port, (unsigned char *)(&recv_package) + recv_index, 16);
		data_length -= 16;
		recv_index += 16;
	}
	Wk2114readFIFO(port,  (unsigned char *)(&recv_package) + recv_index, data_length );
	recv_index += data_length;
	data_length = 0;
	recv_package.checksum = *( (unsigned char *)(&recv_package) + recv_index - 1 );

	// 除了校验字节，将所有数据相加，取后八位用于校验
	if(CalculateChecksum8(&recv_package, recv_package.length - 1) != recv_package.checksum){
		memset(&recv_package, 0, sizeof(recv_package));
		return -5;
	}

	#ifdef DEBUG_MULTIPLE_MESSAGE
	Serial.printf("port %d Recv Data:", port);
	for(unsigned char i=0; i<recv_package.length; i++){
		Serial.printf(" %d", *( (unsigned char *)(&recv_package) + i) );
	}
	Serial.println();
	#endif
	
	return 0;
}

int MULT_DEVICES::CheckPackage()
{
	unsigned char recv_byte;
	unsigned char recv_index = 0;
	
	// 包括帧头，数据帧最短6 byte
	if(GetRxFIFOCnt() < 6)
	{
		return -1;
	}
	// 帧头检测第一个字节
	Wk2114readFIFO(&recv_byte, 1);
	if(recv_byte != PACKAGE_HEAD_1){
		return -2;
	}
	// 帧头检测第二个字节
	Wk2114readFIFO(&recv_byte, 1);
	if(recv_byte != PACKAGE_HEAD_2){
		return -3;
	}
	// 读取数据帧的长度
	unsigned char data_length;
	Wk2114readFIFO(&data_length, 1);
	if(data_length < PACKAGE_MIN_LEN || data_length > PACKAGE_PAYLOAD_MAX_LEN + PACKAGE_MIN_LEN){
		return -4;
	}

	recv_package.length = data_length;
	data_length -= 1; //剩余的数据的长度
	recv_index = 1;

	// 读取FIFO时，每次读取最多16字节
	while(data_length > 16){
		Wk2114readFIFO((unsigned char *)(&recv_package) + recv_index, 16);
		data_length -= 16;
		recv_index += 16;
	}
	Wk2114readFIFO( (unsigned char *)(&recv_package) + recv_index, data_length );
	recv_index += data_length;
	data_length = 0;
	recv_package.checksum = *( (unsigned char *)(&recv_package) + recv_index - 1 );

	// 除了校验字节，将所有数据相加，取后八位用于校验
	if(CalculateChecksum8(&recv_package, recv_package.length - 1) != recv_package.checksum){
		memset(&recv_package, 0, sizeof(recv_package));
		return -5;
	}

	#ifdef DEBUG_MULTIPLE_MESSAGE
	Serial.printf("Recv Data:");
	for(unsigned char i=0; i<recv_package.length; i++){
		Serial.printf(" %d", *( (unsigned char *)(&recv_package) + i) );
	}
	Serial.println();
	#endif

	Uart_ClearRxBuf();
	return 0;
}

int MULT_DEVICES::SendPackage(unsigned char s_addr, unsigned char s_func, 
					unsigned char *s_payload, unsigned char s_payload_len)
{
    struct_Mesg_Package send_package;

	send_package.length = s_payload_len + PACKAGE_MIN_LEN;
	send_package.addr = s_addr;
	send_package.func = s_func;
	memcpy(&send_package.payload, s_payload, s_payload_len);
	send_package.checksum = CalculateChecksum8(&send_package, send_package.length -1);
	
	xQueueSend(tx_queue_handle, &send_package, pdMS_TO_TICKS(20));

	return 0;
}

int MULT_DEVICES::AnalyseRxPackage(unsigned char rx_port)
{
	if(recv_package.length == 0){
		return -1;
	}

	if(device_role == ROLE_MASTER){
		// 如果接收地址不等于 0 ，主机就会将数据帧转发出去
		if(recv_package.addr != 0)
		{
			SendPackage(recv_package.addr, recv_package.func, 
					recv_package.payload, recv_package.length - PACKAGE_MIN_LEN);

			return 0;
		}
	}
	// 解析数据，将命名整形数据传到数据容器
	if(recv_package.func == PACKAGE_FUNC_NAME_INT){
		char recv_name[PACKAGE_PAYLOAD_MAX_LEN];
		unsigned char name_length;

		name_length = recv_package.length -  sizeof(int) - PACKAGE_MIN_LEN;
		if(name_length > PACKAGE_NAME_MAX_LEN){
			return -2;
		}

		memcpy(recv_name, (char *)recv_package.payload, name_length);
		recv_name[name_length] = '\0';

		int var_value;
		memcpy(&var_value, (unsigned char *)recv_package.payload + name_length, sizeof(int));

		std::vector<struct_Int_Message>::iterator i;
		for(i=recv_int_message->begin(); i!=recv_int_message->end(); i++){
			if( i->message == std::string(recv_name) ){
				i->value = var_value;
				break;
			}
		}
		if(i == recv_int_message->end()){
			struct_Int_Message new_message = {std::string(recv_name), var_value};
			recv_int_message->push_back(new_message);
		}
	}
	return 0;
}

int MULT_DEVICES::SendNameVarInt(unsigned char addr, const char *name, int var_value)
{
	unsigned char name_length;
	unsigned char payload_len;

	if(device_role == ROLE_MASTER){
		if(addr == 0){
			// master发给自己的数据，直接解析到数据容器
			std::vector<struct_Int_Message>::iterator i;
			for(i=recv_int_message->begin(); i!=recv_int_message->end(); i++){
				if( i->message == std::string(name) ){
					i->value = var_value;
					break;
				}
			}
			if(i == recv_int_message->end()){
				struct_Int_Message new_message = {std::string(name), var_value};
				recv_int_message->push_back(new_message);
			}

			return 0;
		}
	}

	name_length = strlen(name);
	if(name_length > PACKAGE_NAME_MAX_LEN){
		return -1;
	}
	payload_len = name_length + sizeof(int); // 4byte int数据的长度

	unsigned char *payload;
	payload = (unsigned char *)malloc(payload_len);
	memcpy(payload, name, name_length);
	memcpy(payload+name_length, (unsigned char *)&var_value, sizeof(int));

	if(device_role == ROLE_MASTER){
		// master主动发送，需要转移队列在RX里面再移到TX队列
		struct_Mesg_Package send_package;
		send_package.length = payload_len + PACKAGE_MIN_LEN;
		send_package.addr = addr;
		send_package.func = PACKAGE_FUNC_NAME_INT;
		memcpy(&send_package.payload, payload, payload_len);
		send_package.checksum = CalculateChecksum8(&send_package, send_package.length -1);
		
		xQueueSend(master_tx_queue, &send_package, pdMS_TO_TICKS(20));
	}else{
		SendPackage(addr, PACKAGE_FUNC_NAME_INT, payload, payload_len);
	}

	free(payload);
	return 0;
}

void MULT_DEVICES::MasterRxTask(void *pvParameters)
{
	uint8_t inter_flag;
	int back_code;
	for(;;)
	{
		inter_flag = Mult_Devices->GetInterruptFlag();
		#ifdef DEBUG_MASTER_RX
		Serial.printf("\ninter_flag: %d\n", inter_flag & 0x0F);
		#endif

		if(inter_flag & WK2XXX_UT1INT){
			back_code = Mult_Devices->CheckPackage(1);
			if( 0 == back_code ){
				Mult_Devices->AnalyseRxPackage(1);
			}
			#ifdef DEBUG_MASTER_RX
			else{
				Serial.printf("1BackCode: %d\n", back_code);
			}
			#endif
		}
		if(inter_flag & WK2XXX_UT2INT){
			back_code = Mult_Devices->CheckPackage(2);
			if( 0 == back_code ){
				Mult_Devices->AnalyseRxPackage(2);
			}
			#ifdef DEBUG_MASTER_RX
			else{
				Serial.printf("2BackCode: %d\n", back_code);
			}
			#endif
		}
		if(inter_flag & WK2XXX_UT3INT){
			back_code = Mult_Devices->CheckPackage(3);
			if( 0 == back_code ){
				Mult_Devices->AnalyseRxPackage(3);
			}
			#ifdef DEBUG_MASTER_RX
			else{
				Serial.printf("3BackCode: %d\n", back_code);
			}
			#endif
		}
		if(inter_flag & WK2XXX_UT4INT){
			back_code = Mult_Devices->CheckPackage(4);
			if( 0 == back_code ){
				Mult_Devices->AnalyseRxPackage(4);
			}
			#ifdef DEBUG_MASTER_RX
			else{
				Serial.printf("4BackCode: %d\n", back_code);
			}
			#endif
		}
		// 获取Master 的主动发送的队列，转发出去，限制发送间隔最小 5ms
		static unsigned int last_send_time = 0;
		if(millis() > last_send_time + 5){
			if(uxQueueMessagesWaiting(Mult_Devices->master_tx_queue) != 0){
				struct_Mesg_Package tx_package;
				xQueueReceive(Mult_Devices->master_tx_queue, &tx_package, pdMS_TO_TICKS(5));
				xQueueSend(Mult_Devices->tx_queue_handle, &tx_package, pdMS_TO_TICKS(20));

				last_send_time = millis();
			}
		}

		// 间隔2ms 查询一次接收FIFO中断
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
void MULT_DEVICES::SlaverRxTask(void *pvParameters)
{
	for(;;)
	{
		if(0 == Mult_Devices->CheckPackage()){
			Mult_Devices->AnalyseRxPackage(0);
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
void MULT_DEVICES::TxTask(void *pxParameters)
{
	struct_Mesg_Package tx_data;
	for(;;)
	{
		do{
		} while ( xQueueReceive(Mult_Devices->tx_queue_handle, &tx_data, portMAX_DELAY) != pdTRUE );

		*( (unsigned char *)(&tx_data) + tx_data.length - 1 ) = tx_data.checksum;

		unsigned char package_head[2] = {PACKAGE_HEAD_1, PACKAGE_HEAD_2};
		Mult_Devices->Wk2114writeFIFO(tx_data.addr, package_head, sizeof(package_head));

		unsigned char package_len;
		unsigned char send_index = 0;
		package_len = tx_data.length;
		while(package_len > 16){
			Mult_Devices->Wk2114writeFIFO(tx_data.addr, (unsigned char *)(&tx_data) + send_index, 16);
			send_index += 16;
			package_len -= 16;
		}
		Mult_Devices->Wk2114writeFIFO(tx_data.addr, (unsigned char *)(&tx_data) + send_index, package_len);

		if(Mult_Devices->device_role == ROLE_MASTER){
			vTaskDelay(pdMS_TO_TICKS(1));
		}else{
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}

void MULT_DEVICES::ManagerTask(void *pxParameters)
{
	for(;;){
		do{} while(pdPASS != xSemaphoreTake(Mult_Devices->task_clear_start, portMAX_DELAY));

		Mult_Devices->Uart_Close();

		vTaskDelete(Mult_Devices->rx_task_handle);
		vTaskDelete(Mult_Devices->tx_task_handle);

		xSemaphoreGive(Mult_Devices->task_clear_end);
		vTaskDelete(NULL);
	}
}

void MULT_DEVICES::OpenCommunicate(std::vector<struct_Int_Message> *message_store)
{
	if(device_role != ROLE_TURNOFF){
		return;
	}
	recv_int_message = message_store;

	mutex_mesg_uart = xSemaphoreCreateMutex();
	task_clear_start = xSemaphoreCreateBinary();
	task_clear_end =  xSemaphoreCreateBinary();
	
	CheckMultiHost();

	if(ROLE_MASTER == device_role){
		tx_queue_handle = xQueueCreate(10, sizeof(struct_Mesg_Package));
		master_tx_queue = xQueueCreate(2, sizeof(struct_Mesg_Package));
		xTaskCreatePinnedToCore(MULT_DEVICES::MasterRxTask, "MasterRxTask", 4096, NULL, 1, &rx_task_handle, 1);
	}else{
		tx_queue_handle = xQueueCreate(5, sizeof(struct_Mesg_Package));
		xTaskCreatePinnedToCore(MULT_DEVICES::SlaverRxTask, "SlaverRxTask", 2048, NULL, 1, &rx_task_handle, 1);
	}
	xTaskCreatePinnedToCore(MULT_DEVICES::TxTask, "TxTask", 2048, NULL, 1, &tx_task_handle, 1);
	xTaskCreatePinnedToCore(MULT_DEVICES::ManagerTask, "Manager", 1024, NULL, 2, NULL, 1);

	Serial.printf("device_role: %d \n", device_role);
}

void MULT_DEVICES::CloseCommunicate(void)
{
	if(device_role == ROLE_TURNOFF){
		return;
	}

	xSemaphoreGive(task_clear_start);
	// 等待任务关闭
	do{} while(pdPASS != xSemaphoreTake(task_clear_end, portMAX_DELAY));
	rx_task_handle = NULL;
	tx_task_handle = NULL;

	vQueueDelete(tx_queue_handle);
	vSemaphoreDelete(mutex_mesg_uart);
	vSemaphoreDelete(task_clear_start);
	vSemaphoreDelete(task_clear_end);
	tx_queue_handle = NULL;
	task_clear_start = NULL;
	task_clear_end = NULL;
	mutex_mesg_uart = NULL;

	if(master_tx_queue != NULL){
		vQueueDelete(master_tx_queue);
		master_tx_queue = NULL;
	}

	device_role = ROLE_TURNOFF;
}
