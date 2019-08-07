#ifndef _wk2xxx_H
#define _wk2xxx_H

#include <string>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define 	WK2XXX_GENA     0X00
#define 	WK2XXX_GRST     0X01
#define		WK2XXX_GMUT     0X02
#define 	WK2XXX_GIER     0X10
#define 	WK2XXX_GIFR     0X11
#define 	WK2XXX_GPDIR    0X21
#define 	WK2XXX_GPDAT    0X31
#define 	WK2XXX_GPORT    1//	/wkxxxx  Global rigister of PORT
//wkxxxx  slave uarts  rigister address defines

#define 	WK2XXX_SPAGE    0X03
//PAGE0
#define 	WK2XXX_SCR      0X04
#define 	WK2XXX_LCR      0X05
#define 	WK2XXX_FCR      0X06
#define 	WK2XXX_SIER     0X07
#define 	WK2XXX_SIFR     0X08
#define 	WK2XXX_TFCNT    0X09
#define 	WK2XXX_RFCNT    0X0A
#define 	WK2XXX_FSR      0X0B
#define 	WK2XXX_LSR      0X0C
#define 	WK2XXX_FDAT     0X0D
#define 	WK2XXX_FWCR     0X0D
#define 	WK2XXX_RS485    0X0F
//PAGE1
#define 	WK2XXX_BAUD1    0X04
#define 	WK2XXX_BAUD0    0X05
#define 	WK2XXX_PRES     0X06
#define 	WK2XXX_RFTL     0X07
#define 	WK2XXX_TFTL     0X08
#define 	WK2XXX_FWTH     0X09
#define 	WK2XXX_FWTL     0X0A
#define 	WK2XXX_XON1     0X0B
#define 	WK2XXX_XOFF1    0X0C
#define 	WK2XXX_SADR     0X0D
#define 	WK2XXX_SAEN     0X0D
#define 	WK2XXX_RRSDLY   0X0F

//WK串口扩展芯片的寄存器的位定义
//wkxxx register bit defines
// GENA
#define 	WK2XXX_UT4EN	0x08
#define 	WK2XXX_UT3EN	0x04
#define 	WK2XXX_UT2EN	0x02
#define 	WK2XXX_UT1EN	0x01
//GRST
#define 	WK2XXX_UT4SLEEP	0x80
#define 	WK2XXX_UT3SLEEP	0x40
#define 	WK2XXX_UT2SLEEP	0x20
#define 	WK2XXX_UT1SLEEP	0x10
#define 	WK2XXX_UT4RST	0x08
#define 	WK2XXX_UT3RST	0x04
#define 	WK2XXX_UT2RST	0x02
#define 	WK2XXX_UT1RST	0x01
//GIER
#define 	WK2XXX_UT4IE	0x08
#define 	WK2XXX_UT3IE	0x04
#define 	WK2XXX_UT2IE	0x02
#define 	WK2XXX_UT1IE	0x01
//GIFR
#define 	WK2XXX_UT4INT	0x08
#define 	WK2XXX_UT3INT	0x04
#define 	WK2XXX_UT2INT	0x02
#define 	WK2XXX_UT1INT	0x01
//SPAGE
#define 	WK2XXX_SPAGE0	  0x00
#define 	WK2XXX_SPAGE1   0x01
//SCR
#define 	WK2XXX_SLEEPEN	0x04
#define 	WK2XXX_TXEN     0x02
#define 	WK2XXX_RXEN     0x01
//LCR
#define 	WK2XXX_BREAK	  0x20
#define 	WK2XXX_IREN     0x10
#define 	WK2XXX_PAEN     0x08
#define 	WK2XXX_PAM1     0x04
#define 	WK2XXX_PAM0     0x02
#define 	WK2XXX_STPL     0x01
//FCR
//SIER
#define 	WK2XXX_FERR_IEN      0x80
#define 	WK2XXX_CTS_IEN       0x40
#define 	WK2XXX_RTS_IEN       0x20
#define 	WK2XXX_XOFF_IEN      0x10
#define 	WK2XXX_TFEMPTY_IEN   0x08
#define 	WK2XXX_TFTRIG_IEN    0x04
#define 	WK2XXX_RXOUT_IEN     0x02
#define 	WK2XXX_RFTRIG_IEN    0x01
//SIFR
#define 	WK2XXX_FERR_INT      0x80
#define 	WK2XXX_CTS_INT       0x40
#define 	WK2XXX_RTS_INT       0x20
#define 	WK2XXX_XOFF_INT      0x10
#define 	WK2XXX_TFEMPTY_INT   0x08
#define 	WK2XXX_TFTRIG_INT    0x04
#define 	WK2XXX_RXOVT_INT     0x02
#define 	WK2XXX_RFTRIG_INT    0x01


//TFCNT
//RFCNT
//FSR
#define 	WK2XXX_RFOE     0x80
#define 	WK2XXX_RFBI     0x40
#define 	WK2XXX_RFFE     0x20
#define 	WK2XXX_RFPE     0x10
#define 	WK2XXX_RDAT     0x08
#define 	WK2XXX_TDAT     0x04
#define 	WK2XXX_TFULL    0x02
#define 	WK2XXX_TBUSY    0x01
//LSR
#define 	WK2XXX_OE       0x08
#define 	WK2XXX_BI       0x04
#define 	WK2XXX_FE       0x02
#define 	WK2XXX_PE       0x01
//FWCR
//RS485
//常用波特率宏定义

#define 	B9600	        9600
#define 	B19200	        19200
#define	    B57600	        57600
#define		B115200	        115200
#define		B230400	        230400
#define		B691200	        691200

#define     WK2XXX_RX_FIFO_SIZE     16
#define     WK2XXX_TX_FIFO_SIZE     16

#define     RECEIVE_TIMEOUT     5 // ms

#define     ROLE_SLAVER     2
#define     ROLE_MASTER     1
#define     ROLE_TURNOFF    0

#define     THUNDER_MULTI_BAUD  (B691200)

#define     PACKAGE_MIN_LEN             4
#define     PACKAGE_PAYLOAD_MAX_LEN     24
#define     PACKAGE_HEAD_1              0xFE
#define     PACKAGE_HEAD_2              0xF5

#define     PACKAGE_NAME_MAX_LEN        16
// package function
#define     PACKAGE_FUNC_NAME_INT       0x01

typedef struct {
    unsigned char length;
    unsigned char addr;
    unsigned char func;
    unsigned char payload[PACKAGE_PAYLOAD_MAX_LEN];
    unsigned char checksum;
} struct_Mesg_Package;

typedef union{
    unsigned char port_fifo_cnt[4];
    unsigned int ports_fifo_cnt;
} union_Ports_Fifo_Cnt;

typedef struct {
    unsigned char head_info[3];
    unsigned char valid_cnt = 0;
} struct_Head_Info;

typedef struct {
    std::string message;
    int value;
} struct_Int_Message;

class MULT_DEVICES
{
private:
    int device_role = ROLE_TURNOFF;
    TaskHandle_t rx_task_handle;
    TaskHandle_t tx_task_handle;
    SemaphoreHandle_t mutex_mesg_uart;
    SemaphoreHandle_t task_clear_start;
    SemaphoreHandle_t task_clear_end;
    QueueHandle_t master_tx_queue = NULL;
    QueueHandle_t tx_queue_handle;

    struct_Mesg_Package recv_package;
    struct_Head_Info    recv_head_info;
    std::vector<struct_Int_Message> *recv_int_message;

    void Uart_Init(void);
    void Uart_Close(void);
    void Uart_ClearRxBuf(void);
    void Uart_WaitClear(void);
    void uart_sendByte(unsigned char dat);
    int uart_recByte(unsigned char* readValue);
    void Wk2114WriteReg(unsigned char port,unsigned char reg,unsigned char dat);
    unsigned char Wk2114ReadReg(unsigned char port,unsigned char reg);
    
    void Wk2114SetBaud(unsigned char port,int baud);
    void Wk2114Init(unsigned char port);
    void Wk2114Close(unsigned char port);

    inline int CheckPortDateStatus(unsigned char port){
        if(device_role == ROLE_MASTER){ 
            return (Wk2114ReadReg(port,WK2XXX_FSR)&WK2XXX_RDAT) ? 1 : 0; 
        }else{
            return -1;
        }
    }
    inline unsigned char GetPortTxFIFOCnt(unsigned char port){
        if(device_role == ROLE_MASTER){ 
            return (Wk2114ReadReg(port,WK2XXX_TFCNT));
        }else{
            return -1;
        }
    }
    inline unsigned char GetPortRxFIFOCnt(unsigned char port){
        if(device_role == ROLE_MASTER){ 
            return (Wk2114ReadReg(port,WK2XXX_RFCNT));
        }else{
            return -1;
        }
    }
    inline unsigned char CalculateChecksum8(void *s_data, unsigned char length){
        unsigned char check_sum = 0;

        for(unsigned char i=0; i < length; i++){
            check_sum += *((unsigned char *)(s_data) + i);
        }
        return check_sum;
    }

    int GetRxFIFOCnt();
    unsigned char GetInterruptFlag();
    int Wk2114writeFIFO(unsigned char port,unsigned char *p,unsigned char num);
    int Wk2114readFIFO(unsigned char port,unsigned char *rev_da,unsigned char num);
    int Wk2114writeFIFO(unsigned char *p,unsigned char num);
    int Wk2114readFIFO(unsigned char *rev_da,unsigned char num);

    int CheckMultiHost(void);
    int CheckPackage(unsigned char port);// Master
    int CheckPackage();// Slaver
    int AnalyseRxPackage(unsigned char rx_port);
    int SendPackage(unsigned char addr, unsigned char func, unsigned char *payload, unsigned char payload_len);

public:
    MULT_DEVICES();
    ~MULT_DEVICES();
    
    static void MasterRxTask(void *pvParameters);
    static void SlaverRxTask(void *pvParameters);
    static void TxTask(void *pvParameters);
    static void ManagerTask(void *pvParameters);

    inline int GetRole(void){ return device_role; }

    void OpenCommunicate(std::vector<struct_Int_Message> *message_store);
    void CloseCommunicate(void);
    int SendNameVarInt(unsigned char addr, char *name, int var_value);
};

extern MULT_DEVICES *Mult_Devices;

#endif
