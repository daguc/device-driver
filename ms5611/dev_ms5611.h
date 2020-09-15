#ifndef __DEV_MS5611_H_
#define __DEV_MS5611_H_

#include <stdbool.h>

// 宏定义
#define MS5611_CS_ENABLE    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
#define MS5611_CS_DISABLE   HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

#define MS5611_CMD_ADC_READ   0x00      // ADC读命令 
#define MS5611_CMD_RESET      0x1E 	    // MS5611复位
#define MS5611_CMD_PROM_RD    0xA0      // 读原厂校准数据
#define MS5611_CMD_ADC_D1     0x40      // 读气压原始数据
#define MS5611_CMD_ADC_D2     0x50      // 读温度原始数据

#define MS5611_CMD_ADC_256    0x00     // ADC OSR=256   Conversion time 0.6ms  Resolution 0.065mbar
#define MS5611_CMD_ADC_512    0x02     // ADC OSR=512   Conversion time 1.2ms  Resolution 0.042mbar
#define MS5611_CMD_ADC_1024   0x04     // ADC OSR=1024  Conversion time 2.3ms  Resolution 0.027mbar
#define MS5611_CMD_ADC_2048   0x06     // ADC OSR=2056  Conversion time 4.6ms  Resolution 0.018mbar
#define MS5611_CMD_ADC_4096   0x08     // ADC OSR=4096  Conversion time 9.1ms  Resolution 0.012mbar

// 函数声明
extern bool ms5611_init(void);
extern bool ms5611_update(void);
extern inline float ms5611_get_temperature_c(void);
extern inline float ms5611_get_pressure_pa(void);

#endif
//------------------End of File----------------------------
