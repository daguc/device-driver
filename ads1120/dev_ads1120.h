#ifndef _DEV_ADS1120_H_
#define _DEV_ADS1120_H_

// 头文件
#include "main.h"

// 宏定义
#define ADS1120 /* ADS1120是ADS1220的16版本 */

/* Definition of GPIO Port Bits Used for Communication */
#define SPI1_CS0_DISABLE    HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET)
#define SPI1_CS0_ENABLE     HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_RESET)
#define SPI1_CS1_DISABLE    HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET)
#define SPI1_CS1_ENABLE     HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_RESET)
#define ADS1120_READY0_PIN  HAL_GPIO_ReadPin(DRDY0_GPIO_Port, DRDY0_Pin)
#define ADS1120_READY1_PIN  HAL_GPIO_ReadPin(DRDY1_GPIO_Port, DRDY1_Pin)

/* Error Return Values */
#define ADS1120_NO_ERROR            0
#define ADS1120_ERROR				-1
/* Command Definitions */
#define ADS1120_CMD_RDATA    	0x10
#define ADS1120_CMD_RREG     	0x20
#define ADS1120_CMD_WREG     	0x40
#define ADS1120_CMD_SYNC    	0x08
#define ADS1120_CMD_SHUTDOWN    0x02
#define ADS1120_CMD_RESET    	0x06
/* ADS1120 Register Definitions */
#define ADS1120_0_REGISTER   	0x00
#define ADS1120_1_REGISTER     	0x01
#define ADS1120_2_REGISTER     	0x02
#define ADS1120_3_REGISTER    	0x03
/* ADS1120 Register 0 Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//                     MUX [3:0]                 |             GAIN[2:0]             | PGA_BYPASS
*/
/* Define MUX */
#define ADS1120_MUX_0_1   	0x00
#define ADS1120_MUX_0_2   	0x10
#define ADS1120_MUX_0_3   	0x20
#define ADS1120_MUX_1_2   	0x30
#define ADS1120_MUX_1_3   	0x40
#define ADS1120_MUX_2_3   	0x50
#define ADS1120_MUX_1_0   	0x60
#define ADS1120_MUX_3_2   	0x70
#define ADS1120_MUX_0_G		0x80
#define ADS1120_MUX_1_G   	0x90
#define ADS1120_MUX_2_G   	0xa0
#define ADS1120_MUX_3_G   	0xb0
#define ADS1120_MUX_EX_VREF 0xc0
#define ADS1120_MUX_AVDD   	0xd0
#define ADS1120_MUX_DIV2   	0xe0
/* Define GAIN */
#define ADS1120_GAIN_1      0x00
#define ADS1120_GAIN_2      0x02
#define ADS1120_GAIN_4      0x04
#define ADS1120_GAIN_8      0x06
#define ADS1120_GAIN_16     0x08
#define ADS1120_GAIN_32     0x0a
#define ADS1120_GAIN_64     0x0c
#define ADS1120_GAIN_128    0x0e
/* Define PGA_BYPASS */
#define ADS1120_PGA_BYPASS 	0x01
/* ADS1120 Register 1 Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//                DR[2:0]            |      MODE[1:0]        |     CM    |     TS    |    BCS
*/
/* Define DR (data rate) */
#define ADS1120_DR_20		0x00
#define ADS1120_DR_45		0x20
#define ADS1120_DR_90		0x40
#define ADS1120_DR_175		0x60
#define ADS1120_DR_330		0x80
#define ADS1120_DR_600		0xa0
#define ADS1120_DR_1000		0xc0
/* Define MODE of Operation */
#define ADS1120_MODE_NORMAL 0x00
#define ADS1120_MODE_DUTY	0x08
#define ADS1120_MODE_TURBO 	0x10
#define ADS1120_MODE_DCT	0x18
/* Define CM (conversion mode) */
#define ADS1120_CC			0x04
/* Define TS (temperature sensor) */
#define ADS1120_TEMP_SENSOR	0x02
/* Define BCS (burnout current source) */
#define ADS1120_BCS			0x01
/* ADS1120 Register 2 Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//         VREF[1:0]     |        50/60[1:0]     |    PSW    |             IDAC[2:0]
*/
/* Define VREF */
#define ADS1120_VREF_INT	0x00
#define ADS1120_VREF_EX_DED	0x40
#define ADS1120_VREF_EX_AIN	0x80
#define ADS1120_VREF_SUPPLY	0xc0
/* Define 50/60 (filter response) */
#define ADS1120_REJECT_OFF	0x00
#define ADS1120_REJECT_BOTH	0x10
#define ADS1120_REJECT_50	0x20
#define ADS1120_REJECT_60	0x30
/* Define PSW (low side power switch) */
#define ADS1120_PSW_SW		0x08
/* Define IDAC (IDAC current) */
#define ADS1120_IDAC_OFF	0x00
#define ADS1120_IDAC_10		0x01
#define ADS1120_IDAC_50		0x02
#define ADS1120_IDAC_100	0x03
#define ADS1120_IDAC_250	0x04
#define ADS1120_IDAC_500	0x05
#define ADS1120_IDAC_1000	0x06
#define ADS1120_IDAC_2000	0x07
/* ADS1120 Register 3 Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//               I1MUX[2:0]          |               I2MUX[2:0]          |   DRDYM   | RESERVED
*/
/* Define I1MUX (current routing) */
#define ADS1120_IDAC1_OFF	0x00
#define ADS1120_IDAC1_AIN0	0x20
#define ADS1120_IDAC1_AIN1	0x40
#define ADS1120_IDAC1_AIN2	0x60
#define ADS1120_IDAC1_AIN3	0x80
#define ADS1120_IDAC1_REFP0	0xa0
#define ADS1120_IDAC1_REFN0	0xc0
/* Define I2MUX (current routing) */
#define ADS1120_IDAC2_OFF	0x00
#define ADS1120_IDAC2_AIN0	0x04
#define ADS1120_IDAC2_AIN1	0x08
#define ADS1120_IDAC2_AIN2	0x0c
#define ADS1120_IDAC2_AIN3	0x10
#define ADS1120_IDAC2_REFP0	0x14
#define ADS1120_IDAC2_REFN0	0x18
/* define DRDYM (DOUT/DRDY behaviour) */
#define ADS1120_DRDY_MODE	0x02

#pragma pack(push, 4) // 4字节对齐
// 数据结构
typedef struct { 
    long analog_diff_01;                // AN0与AN1电压模拟量差
    double vol_diff_01;                 // AN0与AN1电压差
}Adc1120DataTypedef;
#pragma pack(pop)   // 4字节对齐

/* Low Level ADS1120 Device Functions */
extern void ADS1120_init(void);							    /* Device initialization */
extern void ADS1120_update(void);                           /* Device data update */

#endif /*ADS1120_H_*/
