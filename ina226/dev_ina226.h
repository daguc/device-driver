#ifndef _DEV_INA226_H_
#define _DEV_INA226_H_

// 头文件
#include "main.h"
#include <string.h>
#include <stdbool.h>

// 宏定义
#define INA226_HW_I2C // hardware i2c
//#define INA226_SW_I2C // software i2c
#define INA226_L_ADDR   (0x40 << 1)     // INA226 L device address
#define INA226_R_ADDR   (0x41 << 1)     // INA226 R device address
// register
#define INA226_REG_CONFIG       0x00    // 配置
#define INA226_REG_SHUNTVOLTAGE 0x01    // 分流电压
#define INA226_REG_BUSVOLTAGE   0x02    // 总线电压
#define INA226_REG_POWER        0x03    // 电源功率
#define INA226_REG_CURRENT      0x04    // 电流
#define INA226_REG_CALIBRATION  0x05    // 校准
#define INA226_REG_MASKENABLE   0x06    // 屏蔽
#define INA226_REG_ALERTLIMIT   0x07    // 警报限定
#define INA226_REG_ID           0XFF    // 芯片标识号

// 定义配置数据
#define INA226_SHUNTVOLTAGE_LSB	2.5f	//分流电压: 2.5uV/LSB
#define INA226_VBUS_VOL_LSB	    1.25f   //总线电压: 1.25mV/LSB
#define INA226_CURRENT_LSB 	    1.0f 	//电流: 25mA/LSB
#define INA226_POWER_LSB        (25 * INA226_CURRENT_LSB)

// 数据结构
typedef enum {
    INA226_AVERAGES_1             = 0,// 0b000
    INA226_AVERAGES_4             = 1,// 0b001
    INA226_AVERAGES_16            = 2,// 0b010
    INA226_AVERAGES_64            = 3,// 0b011
    INA226_AVERAGES_128           = 4,// 0b100
    INA226_AVERAGES_256           = 5,// 0b101
    INA226_AVERAGES_512           = 6,// 0b110
    INA226_AVERAGES_1024          = 7,// 0b111
} ina226_averages_t;    

typedef enum {
    INA226_BUS_CONV_TIME_140US    = 0,// 0b000
    INA226_BUS_CONV_TIME_204US    = 1,// 0b001
    INA226_BUS_CONV_TIME_332US    = 2,// 0b010
    INA226_BUS_CONV_TIME_588US    = 3,// 0b011
    INA226_BUS_CONV_TIME_1100US   = 4,// 0b100
    INA226_BUS_CONV_TIME_2116US   = 5,// 0b101
    INA226_BUS_CONV_TIME_4156US   = 6,// 0b110
    INA226_BUS_CONV_TIME_8244US   = 7,// 0b111
} ina226_busConvTime_t;

typedef enum {
    INA226_SHUNT_CONV_TIME_140US   = 0,// 0b000
    INA226_SHUNT_CONV_TIME_204US   = 1,// 0b001
    INA226_SHUNT_CONV_TIME_332US   = 2,// 0b010
    INA226_SHUNT_CONV_TIME_588US   = 3,// 0b011
    INA226_SHUNT_CONV_TIME_1100US  = 4,// 0b100
    INA226_SHUNT_CONV_TIME_2116US  = 5,// 0b101
    INA226_SHUNT_CONV_TIME_4156US  = 6,// 0b110
    INA226_SHUNT_CONV_TIME_8244US  = 7,// 0b111
} ina226_shuntConvTime_t;

typedef enum {
    INA226_MODE_POWER_DOWN      = 0,// 0b000
    INA226_MODE_SHUNT_TRIG      = 1,// 0b001
    INA226_MODE_BUS_TRIG        = 2,// 0b010
    INA226_MODE_SHUNT_BUS_TRIG  = 3,// 0b011
    INA226_MODE_ADC_OFF         = 4,// 0b100
    INA226_MODE_SHUNT_CONT      = 5,// 0b101
    INA226_MODE_BUS_CONT        = 6,// 0b110 
    INA226_MODE_SHUNT_BUS_CONT  = 7,// 0b111
} ina226_mode_t;    

typedef union {
    uint8_t buffer[2];
    uint16_t data;
} ina226_data_t;

typedef struct {
    bool init_surcceed;
    uint16_t id;
    uint16_t vbus_mV;
    uint16_t shunt_uV;
    uint16_t shunt_mA;
    uint16_t power_mW;
    float power_W;
}INA226Typedef;

// 声明
extern void ina226_init(void);
extern void ina226_update(void);

#endif
