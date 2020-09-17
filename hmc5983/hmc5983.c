//头文件
#include "hmc5983.h"
#include "spi.h"
#include "timer.h"

//宏定义
#define HMC5983_DEV_ADDR_READ	0x3D		//I2C接口使用
#define HMC5983_DEV_ADDR_WRITE  0x3C 

#define HMC5983_DIR_READ			(1<<7)	//SPI接口使用
#define HMC5983_DIR_WRITE			(0<<7)
#define HMC5983_ADDR_INCREMENT		(1<<6)

#define HMC5983_ADDR_CONF_A				0x00
#define HMC5983_ADDR_CONF_B				0x01
#define HMC5983_ADDR_MODE				0x02
#define HMC5983_ADDR_DATA_OUT_X_MSB		0x03
#define HMC5983_ADDR_DATA_OUT_X_LSB		0x04
#define HMC5983_ADDR_DATA_OUT_Z_MSB		0x05
#define HMC5983_ADDR_DATA_OUT_Z_LSB		0x06
#define HMC5983_ADDR_DATA_OUT_Y_MSB		0x07
#define HMC5983_ADDR_DATA_OUT_Y_LSB		0x08
#define HMC5983_ADDR_STATUS				0x09
#define HMC5983_ADDR_ID_A				0x0a
#define HMC5983_ADDR_ID_B				0x0b
#define HMC5983_ADDR_ID_C				0x0c
#define HMC5983_ADDR_TEMP_MSB			0x31
#define HMC5983_ADDR_TEMP_LSB			0x32

#define HMC5983_ID_A_WHO_AM_I		'H'
#define HMC5983_ID_B_WHO_AM_I		'4'
#define HMC5983_ID_C_WHO_AM_I		'3'

#define HMC5983_MODE_NORMAL			(0 << 0)  /* default */
#define HMC5983_MODE_POSITIVE_BIAS	(1 << 0)  /* positive bias */
#define HMC5983_MODE_NEGATIVE_BIAS	(2 << 0)  /* negative bias */
#define HMC5983_MODE_TEMP_ONLY		(3 << 0)  /* Temperature sensor only. Magnetic sensor will not be enabled during measurement. */

#define HMC5983_RATE_075HZ			(0 << 2)
#define HMC5983_RATE_105HZ			(1 << 2)
#define HMC5983_RATE_3HZ			(2 << 2)
#define HMC5983_RATE_705HZ			(3 << 2)
#define HMC5983_RATE_15HZ			(4 << 2)
#define HMC5983_RATE_30HZ			(5 << 2)
#define HMC5983_RATE_75HZ			(6 << 2)

#define HMC5983_AVERAGING_1		(0 << 5) /* conf a register */
#define HMC5983_AVERAGING_2		(1 << 5)
#define HMC5983_AVERAGING_4		(2 << 5)
#define HMC5983_AVERAGING_8		(3 << 5)

#define HMC5983_RANGE_088GA			(0 << 5)
#define HMC5983_RANGE_103GA			(1 << 5)
#define HMC5983_RANGE_109GA			(2 << 5)
#define HMC5983_RANGE_205GA			(3 << 5)
#define HMC5983_RANGE_4GA			(4 << 5)
#define HMC5983_RANGE_407GA			(5 << 5)
#define HMC5983_RANGE_506GA			(6 << 5)
#define HMC5983_RANGE_801GA			(7 << 5)

#define HMC5983_CONTINOUS_MODE		(0 << 0)
#define HMC5983_SINGLE_MODE			(1 << 0) /* default */

#define HMC5983_ENABLE_TEMP			(1 << 7)

//全局变量
//参数


//变量
float g_hmc5983_range_scale;
float g_hmc5983_range_ga;

//函数声明
void hmc5983_read_regs(uint8_t addr, uint8_t *pData, uint8_t size);
uint8_t hmc5983_read_reg(uint8_t addr);
void hmc5983_write_reg(uint8_t addr, uint8_t data);
void hmc5983_set_range(uint8_t range);

/**
  * @brief	  hmc5983_Init
  * @param	  null
  * @retval   true: 初始化成功，false: 初始化失败
  */
bool hmc5983_init(void)
{
	//读取设备ID
	uint8_t dev_id[3];
	hmc5983_read_regs(HMC5983_ADDR_ID_A, dev_id, 3);	
	if(dev_id[0] != HMC5983_ID_A_WHO_AM_I
	|| dev_id[1] != HMC5983_ID_B_WHO_AM_I
	|| dev_id[2] != HMC5983_ID_C_WHO_AM_I){
		return false;
	}
	
	//配置
	hmc5983_write_reg(HMC5983_ADDR_CONF_A, HMC5983_AVERAGING_8|HMC5983_RATE_75HZ|HMC5983_MODE_NORMAL|HMC5983_ENABLE_TEMP);
	hmc5983_set_range(HMC5983_RANGE_4GA);
	hmc5983_write_reg(HMC5983_ADDR_MODE, HMC5983_CONTINOUS_MODE);
	
	return true;
}

/**
  * @brief	  hmc5983 update
  * @param	  磁场数据（单位毫高斯）
  * @retval   null
  */
void hmc5983_update(vector3f_t* mag)
{
	uint8_t buffer[6];
	int16_t raw[3];
	
	hmc5983_read_regs(HMC5983_ADDR_DATA_OUT_X_MSB, buffer, 6);
	
	raw[0] = (int16_t)((((uint16_t)buffer[0]) << 8) + (uint16_t)buffer[1]);
	raw[2] = (int16_t)((((uint16_t)buffer[2]) << 8) + (uint16_t)buffer[3]);
	raw[1] = (int16_t)((((uint16_t)buffer[4]) << 8) + (uint16_t)buffer[5]);
	
	// 安装坐标转换
	mag->x = -raw[0] * g_hmc5983_range_scale;
	mag->y = -raw[1] * g_hmc5983_range_scale;
	mag->z = raw[2] * g_hmc5983_range_scale;
}

/**
  * @brief	  hmc5983 测量范围设置
  * @param	  测量范围
  * @retval   null
  */
void hmc5983_set_range(uint8_t range)
{
	if (range == HMC5983_RANGE_088GA) {
		g_hmc5983_range_scale = 1000.0f / 1370.0f;
		g_hmc5983_range_ga = 0.88f;

	} else if (range == HMC5983_RANGE_103GA) {
		g_hmc5983_range_scale = 1000.0f / 1090.0f;
		g_hmc5983_range_ga = 1.3f;

	} else if (range == HMC5983_RANGE_109GA) {
		g_hmc5983_range_scale = 1000.0f / 820.0f;
		g_hmc5983_range_ga = 1.9f;

	} else if (range == HMC5983_RANGE_205GA) {
		g_hmc5983_range_scale = 1000.0f / 660.0f;
		g_hmc5983_range_ga = 2.5f;

	} else if (range == HMC5983_RANGE_4GA) {
		g_hmc5983_range_scale = 1000.0f / 440.0f;
		g_hmc5983_range_ga = 4.0f;

	} else if (range == HMC5983_RANGE_407GA) {
		g_hmc5983_range_scale = 1000.0f / 390.0f;
		g_hmc5983_range_ga = 4.7f;

	} else if (range == HMC5983_RANGE_506GA) {
		g_hmc5983_range_scale = 1000.0f / 330.0f;
		g_hmc5983_range_ga = 5.6f;

	} else {
		range = HMC5983_RANGE_801GA;
		g_hmc5983_range_scale = 1000.0f / 230.0f;
		g_hmc5983_range_ga = 8.1f;
	}
	// 发送测量范围设置命令
	hmc5983_write_reg(HMC5983_ADDR_CONF_B, range);
}

/**
  * @brief	  hmc5983 读取寄存器
  * @param	  寄存器地址addr，数据缓冲区指针pData， 数据大小size
  * @retval   null
  */
void hmc5983_read_regs(uint8_t addr, uint8_t *pData, uint8_t size)
{	
	HMC5883_CS_ENABLE;
	delay_us(2);
	SPI1_ReadWriteByte(HMC5983_DIR_READ|HMC5983_ADDR_INCREMENT|addr);  	//设置地址指针
	for(uint16_t i=0; i<size; i++)
		pData[i] = SPI1_ReadWriteByte(0xFF);
	delay_us(2);
	HMC5883_CS_DISABLE;
}

/**
  * @brief	  hmc5983 读单个寄存器
  * @param	  寄存器地址addr，数据data
  * @retval   null
  */
uint8_t hmc5983_read_reg(uint8_t addr)
{
	uint8_t ret;
    
	HMC5883_CS_ENABLE;
	delay_us(2);
	SPI1_ReadWriteByte(HMC5983_DIR_READ|addr);  	//设置地址指针
	ret = SPI1_ReadWriteByte(0xFF);
	delay_us(2);
	HMC5883_CS_DISABLE;
	
	return ret;
}

/**
  * @brief	  hmc5983 写单个寄存器
  * @param	  寄存器地址addr，数据data
  * @retval   null
  */
void hmc5983_write_reg(uint8_t addr, uint8_t data)
{
	HMC5883_CS_ENABLE;
	delay_us(2);
	SPI1_ReadWriteByte(HMC5983_DIR_WRITE|addr);  	//设置地址指针
	SPI1_ReadWriteByte(data);
	delay_us(2);
	HMC5883_CS_DISABLE;
}

//文件结束
