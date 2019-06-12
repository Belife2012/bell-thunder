#include "Sensor_IIC.h"

#ifndef _BH1745NUC_H_
#define _BH1745NUC_H_

#define BH1745NUC_DEVICE_ADDRESS_38            (0x38)    // 7bit Addrss
#define BH1745NUC_DEVICE_ADDRESS_39            (0x39)    // 7bit Addrss
#define BH1745NUC_PART_ID_VAL                  (0x0B)
#define BH1745NUC_MANUFACT_ID_VAL              (0xE0)

#define BH1745NUC_SYSTEM_CONTROL               (0x40)
#define BH1745NUC_MODE_CONTROL1                (0x41)
#define BH1745NUC_MODE_CONTROL2                (0x42)
#define BH1745NUC_MODE_CONTROL3                (0x44)
#define BH1745NUC_RED_DATA_LSB                 (0x50)
#define BH1745NUC_MANUFACTURER_ID              (0x92)

#define BH1745NUC_MODE_CONTROL1_MEAS_TIME160MS (0x00)

#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X1    (0)
#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X2    (1)
#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X16   (2)
#define BH1745NUC_MODE_CONTROL2_RGBC_EN        (1 << 4)

#define BH1745NUC_MODE_CONTROL1_VAL            (BH1745NUC_MODE_CONTROL1_MEAS_TIME160MS)
#define BH1745NUC_MODE_CONTROL2_VAL            (BH1745NUC_MODE_CONTROL2_RGBC_EN | BH1745NUC_MODE_CONTROL2_ADC_GAIN_X16)
#define BH1745NUC_MODE_CONTROL3_VAL            (0x02)

// 颜色卡参数
#if 0 // 裸板，没有外壳，LED与SENSOR间距减小
// 亮度值C
#define NO_COLOR_CARD_C         (160)
#define BLACK_CARD_MIN_C        (160)
#define BLACK_CARD_MAX_C        (400)
#define GREEN_CARD_MIN_C        (350)
#define GREEN_CARD_MAX_C        (1000)
#define RED_CARD_MIN_C          (450)
#define RED_CARD_MAX_C          (1500)
#define BROWN_CARD_MIN_C        (400)
#define BROWN_CARD_MAX_C        (1700)
#define BLUE_CARD_MIN_C         (400)
#define BLUE_CARD_MAX_C         (1700)
#define YELLOW_CARD_MIN_C       (700)
#define YELLOW_CARD_MAX_C       (3500)
#define WHITE_CARD_MIN_C        (1000)
#define WHITE_CARD_MAX_C        (6000)
// 黑白卡的RGB值分别由最大值/最小值
#define BLACK_CARD_MIN_RGB         (400)
#define BLACK_CARD_MAX_RGB         (4500)
#define WHITE_CARD_MIN_RGB         (5000)
#define WHITE_CARD_MAX_RGB         (60000)
// HSV 的S值 可以区分颜色的灰度，值越大，彩色越浓，趋向 0 时，就越接近 黑白
#define COLORLESS_S             (0.7)
// 彩色卡可以依据颜色 H值 进行判断
#define COLORLESS_MIN_H         (110)
#define COLORLESS_MAX_H         (150)
#define RED_CARD_MIN_H          (0)
#define RED_CARD_MAX_H          (12)
#define BROWN_CARD_MIN_H        (35)
#define BROWN_CARD_MAX_H        (55)
#define YELLOW_CARD_MIN_H       (62)
#define YELLOW_CARD_MAX_H       (85)
#define GREEN_CARD_MIN_H        (110)
#define GREEN_CARD_MAX_H        (140)
#define BLUE_CARD_MIN_H         (155)
#define BLUE_CARD_MAX_H         (214)
#else // 现有的橘色模具
#if 1 // 270欧姆电阻
  // 亮度值C
  #define NO_COLOR_CARD_C         (100)
  #define BLACK_CARD_MIN_C        (30)
  #define BLACK_CARD_MAX_C        (110)
  #define GREEN_CARD_MIN_C        (60)
  #define GREEN_CARD_MAX_C        (260)
  #define RED_CARD_MIN_C          (100)
  #define RED_CARD_MAX_C          (500)
  #define BROWN_CARD_MIN_C        (100)
  #define BROWN_CARD_MAX_C        (500)
  #define BLUE_CARD_MIN_C         (100)
  #define BLUE_CARD_MAX_C         (430)
  #define YELLOW_CARD_MIN_C       (150)
  #define YELLOW_CARD_MAX_C       (900)
  #define WHITE_CARD_MIN_C        (320)
  #define WHITE_CARD_MAX_C        (1500)
  // 黑白卡的RGB值分别由最大值/最小值
  #define BLACK_CARD_MIN_RGB         (150)
  #define BLACK_CARD_MAX_RGB         (1000)
  #define WHITE_CARD_MIN_RGB         (1800)
  #define WHITE_CARD_MAX_RGB         (15000)
  // HSV 的S值 可以区分颜色的灰度，值越大，彩色越浓，趋向 0 时，就越接近 黑白
  #define COLORLESS_S             (0.62)
  // 彩色卡可以依据颜色 H值 进行判断
  #define COLORLESS_MIN_H         (60)
  #define COLORLESS_MAX_H         (130)
  #define BLACK_MIN_H         (55)
  #define BLACK_MAX_H         (100)
  #define WHITE_MIN_H         (105)
  #define WHITE_MAX_H         (140)
  #define RED_CARD_MIN_H          (0)
  #define RED_CARD_MAX_H          (12)
  #define BROWN_CARD_MIN_H        (25)
  #define BROWN_CARD_MAX_H        (38)
  #define YELLOW_CARD_MIN_H       (47)
  #define YELLOW_CARD_MAX_H       (70)
  #define GREEN_CARD_MIN_H        (105)
  #define GREEN_CARD_MAX_H        (135)
  #define BLUE_CARD_MIN_H         (170)
  #define BLUE_CARD_MAX_H         (220)
#else // 1K欧姆电阻
  // 亮度值C
  #define NO_COLOR_CARD_C         (100)
  #define BLACK_CARD_MIN_C        (10)    // 判断黑卡要先获取环境光的C值，从而再次确认判断为黑卡的C值范围
  #define BLACK_CARD_MAX_C        (70)
  #define GREEN_CARD_MIN_C        (25)
  #define GREEN_CARD_MAX_C        (150)
  #define RED_CARD_MIN_C          (40)
  #define RED_CARD_MAX_C          (250)
  #define BROWN_CARD_MIN_C        (40)
  #define BROWN_CARD_MAX_C        (250)
  #define BLUE_CARD_MIN_C         (30)
  #define BLUE_CARD_MAX_C         (200)
  #define YELLOW_CARD_MIN_C       (70)
  #define YELLOW_CARD_MAX_C       (450)
  #define WHITE_CARD_MIN_C        (100)
  #define WHITE_CARD_MAX_C        (700)
  // 黑白卡的RGB值分别由最大值/最小值
  #define BLACK_CARD_MIN_RGB         (25)
  #define BLACK_CARD_MAX_RGB         (500)
  #define WHITE_CARD_MIN_RGB         (450)
  #define WHITE_CARD_MAX_RGB         (5000)
  // HSV 的S值 可以区分颜色的灰度，值越大，彩色越浓，趋向 0 时，就越接近 黑白
  #define COLORLESS_S             (0.68)
  // 彩色卡可以依据颜色 H值 进行判断
  #define COLORLESS_MIN_H         (40)
  #define COLORLESS_MAX_H         (125)
  #define BLACK_MIN_H         (41)
  #define BLACK_MAX_H         (80)
  #define WHITE_MIN_H         (81)
  #define WHITE_MAX_H         (125)
  #define RED_CARD_MIN_H          (0)
  #define RED_CARD_MAX_H          (12)
  #define BROWN_CARD_MIN_H        (25)
  #define BROWN_CARD_MAX_H        (37)
  #define YELLOW_CARD_MIN_H       (47)
  #define YELLOW_CARD_MAX_H       (65)
  #define GREEN_CARD_MIN_H        (100)
  #define GREEN_CARD_MAX_H        (137)
  #define BLUE_CARD_MIN_H         (160)
  #define BLUE_CARD_MAX_H         (215)

#endif

#endif

typedef enum{
  BLACK_CARD = 0,
  GREEN_CARD,
  RED_CARD,
  BROWN_CARD,
  BLUE_CARD,
  YELLOW_CARD,
  WHITE_CARD,
  NO_CARD
}enum_Color_Card;

class BH1745NUC : public SENSOR_IIC
{
  public:
    float env_backlight_c;
    
    BH1745NUC(int slave_address);                     // 配置I2C地址
    
    byte Setup(void);                                 // 初始化设置
    byte Get_RGBC_Data(unsigned short *data);         // 获取RGBC
    void RGBtoHSV(unsigned short *RGBC, float *HSV);  // 计算HSV

    uint8_t Colour_Recognition(unsigned short *RGBC); // 识别颜色 (预留)
    uint8_t Thunder_Get_Color_Sensor_Data(uint8_t sensorChannel);

  private:
    byte device_detected;// 0为未插入设备，!0为已插入设备

    void Env_Backlight_Filter(unsigned short new_data);
    byte get_rawval(unsigned char *data);                                               // 类内部使用，读取传感器数据
};

#endif // _BH1745NUC_H_
