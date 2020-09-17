#include "qmc5883.h"
#include "timer.h"
#include "i2c.h"
//#include "drv_sw_iic.h"

//宏定义
#define QMC5883_ADDR			0x0D//The default I2C address is 0D: 0001101
#define QMC5883_WRITE_ADDR		0x1A
#define QMC5883_READ_ADDR		0x1B

#define QMC5883_REG_DATA		0x00
#define QMC5883_REG_OUT_X_L    	0x00
#define QMC5883_REG_OUT_X_M    	0x01
#define QMC5883_REG_OUT_Y_L    	0x02
#define QMC5883_REG_OUT_Y_M    	0x03
#define QMC5883_REG_OUT_Z_L    	0x04
#define QMC5883_REG_OUT_Z_M    	0x05
 
#define QMC5883_REG_STATUS     	0x06
   #define QMC5883_DRDY_BIT0      //0: no new data, 1: new data is ready
   #define QMC5883_OVL_BIT1       //0: normal,      1: data overflow
   #define QMC5883_DOR_BIT2       //0: normal,      1: data skipped for reading
   
#define QMC5883_REG_TEMP_OUT_L 	0x07
#define QMC5883_REG_TEMP_OUT_H 	0x08
 
#define QMC5883_REG_CTRL1      	0x09
   #define QMC5883_CMD_MODE_STANDBY		0x00  //mode 
   #define QMC5883_CMD_MODE_CON         0x01
   #define QMC5883_CMD_ODR_10HZ         0x00  //Output Data Rate
   #define QMC5883_CMD_ODR_50HZ         0x04
   #define QMC5883_CMD_ODR_100HZ        0x08
   #define QMC5883_CMD_ODR_200HZ        0x0C
   #define QMC5883_CMD_RNG_2G           0x00  //Full Scale
   #define QMC5883_CMD_RNG_8G           0x10    
   #define QMC5883_CMD_OSR_512          0x00  //Over Sample Ratio
   #define QMC5883_CMD_OSR_256          0x40    
   #define QMC5883_CMD_OSR_128          0x80    
   #define QMC5883_CMD_OSR_64           0xC0    
 
#define QMC5883_REG_CTRL2      	0x0A
   #define QMC5883_CMD_INT_ENABLE       0x00 
   #define QMC5883_CMD_INT_DISABLE      0x01
   #define QMC5883_CMD_ROL_PNT_ENABLE   0x40  //pointer roll-over function,only 0x00-0x06 address
   #define QMC5883_CMD_ROL_PNT_DISABLE  0x00 
   #define QMC5883_CMD_SOFT_RST_ENABLE  0x80
   #define QMC5883_CMD_SOFT_RST_DISABLE 0x00 
   
#define QMC5883_REG_SET_RESET  	0x0B
   #define QMC5883_CMD_SET_RESET        0x01 
  
#define QMC5883_REG_PRODUCTID  	0x0D           //chip id :0xFF
#define	QMC5883_PRODUCTID		0xFF
#define	QMC5883_RNG_2G_RANGE	(12000.0)
#define	QMC5883_RNG_8G_RANGE	(3000.0)

//本地函数
static uint8_t qmc5883_read_reg(uint8_t reg_addr, uint8_t *pdata);
static uint8_t qmc5883_read_regs(uint8_t reg_addr, uint8_t *pdata, uint8_t size);
static uint8_t qmc5983_write_reg(uint8_t reg_addr, uint8_t data);

/**
  * @brief	  qmc5883_Init
  * @param	  null
  * @retval   null
  */
bool qmc5883_init(void)
{
	bool res = false ;
	uint8_t reg1_state = 0;
	uint8_t chip_id = 0;
	
	/* 读磁力计ID */
	qmc5883_read_reg(QMC5883_REG_PRODUCTID, &chip_id);
	if (chip_id != QMC5883_PRODUCTID)
    {
		/* 磁力计读ID失败 */
		return false;
	}
	
	/* 使能软件复位 */
	qmc5983_write_reg(QMC5883_REG_CTRL2, QMC5883_CMD_SOFT_RST_ENABLE);
	delay_us(100);
	
	/* RESET Period  */
	qmc5983_write_reg(QMC5883_REG_SET_RESET, QMC5883_CMD_SET_RESET);
	
	/*
		Mode = continuous (mode control)
		ODR = 100Hz (output data rate)
		RNG = +/-8G (full sacle)
		OSR = 512 (over sample ratio)
	*/
	qmc5983_write_reg(QMC5883_REG_CTRL1, QMC5883_CMD_MODE_CON | QMC5883_CMD_ODR_100HZ | QMC5883_CMD_RNG_2G | QMC5883_CMD_OSR_512);	
	
	/* 不使用中断，数据指针自动移动到下一个地址（在寄存器00H-05H之间） */
	qmc5983_write_reg(QMC5883_REG_CTRL2, QMC5883_CMD_INT_DISABLE | QMC5883_CMD_ROL_PNT_ENABLE);	
	
	qmc5883_read_reg(QMC5883_REG_CTRL1, &reg1_state);
	if (reg1_state == (QMC5883_CMD_MODE_CON | QMC5883_CMD_ODR_100HZ | QMC5883_CMD_RNG_2G | QMC5883_CMD_OSR_512))
	{
		res = true;
	}
	
	return res;
}

/**
  * @brief	  qmc5883_read_reg
  * @param	  
  * @retval   
  */
static uint8_t qmc5883_read_reg(uint8_t reg_addr, uint8_t *pdata)
{
	/*
	HAL_StatusTypeDef res = HAL_TIMEOUT;
	
	res = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)QMC5883_READ_ADDR, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, 1, 100);

	return res;
	*/
	
	uint8_t res = I2C1_Read_NBytes((uint8_t)QMC5883_READ_ADDR, (uint8_t)reg_addr, 1, pdata);
	
	return res;
}

/**
  * @brief	  qmc5883_read_regs
  * @param	  
  * @retval   
  */
static uint8_t qmc5883_read_regs(uint8_t reg_addr, uint8_t *pdata, uint8_t size)
{
	/*
	HAL_StatusTypeDef res = HAL_TIMEOUT;
	
	res = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)QMC5883_READ_ADDR, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, size, 100);

	return res;
	*/
	
	uint8_t res = I2C1_Read_NBytes((uint8_t)QMC5883_READ_ADDR, (uint8_t)reg_addr, size, pdata);
	
	return res;
}

/**
  * @brief	  qmc5983_write_reg
  * @param	  
  * @retval   
  */
static uint8_t qmc5983_write_reg(uint8_t reg_addr, uint8_t data)
{
	/*
	HAL_StatusTypeDef res = HAL_TIMEOUT;
	
	res = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)QMC5883_WRITE_ADDR, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

	return res;
	*/
	
	uint8_t res = I2C1_Write_NBytes((uint8_t)QMC5883_WRITE_ADDR, (uint8_t)reg_addr, 1, &data);
	
	return res;
}

/**
  * @brief	  qmc5883_update
  * @param	  vector3f_t(uint: mG)
  * @retval   null
  */
void qmc5883_update(vector3f_t* mag)
{
	uint8_t buffer[6];
	int16_t raw[3];
	
	qmc5883_read_regs(QMC5883_REG_DATA, buffer, 6);
	
	raw[0] = (int16_t)((((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0]);
	raw[2] = (int16_t)((((uint16_t)buffer[3]) << 8) + (uint16_t)buffer[2]);
	raw[1] = (int16_t)((((uint16_t)buffer[5]) << 8) + (uint16_t)buffer[4]);
	
	//安装坐标转换
	mag->x = raw[0] / QMC5883_RNG_2G_RANGE * 1000.0;//unit: mG
	mag->y = -raw[1] / QMC5883_RNG_2G_RANGE * 1000.0;//unit: mG
	mag->z = raw[2] / QMC5883_RNG_2G_RANGE * 1000.0;//unit: mG
}
