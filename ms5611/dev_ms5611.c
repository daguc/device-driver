#include "dev_ms5611.h"
#include <math.h>
#include "cmsis_os.h"
#include "spi.h"
#include "tim.h"

// 气压计状态机
enum{
	MS5611_START_CONVERT_TEMP 		= 0,
	MS5611_CONVERTING_TEMP 			= 1,
	MS5611_START_CONVERT_PRESS 		= 2,
	MS5611_CONVERTING_PRESS 		= 3,	
}ms5611_state;
    
// 延时表  单位 us    不同的采样精度对应不同的延时值
const uint32_t ms5611_delay_us[9] = {
    800,    // MS561101BA_OSR_256  0.9ms 0x00
    800,    // MS561101BA_OSR_256  0.9ms  
    1500,   // MS561101BA_OSR_512  1.2ms 0x02
    1500,   // MS561101BA_OSR_512  1.2ms
    2500,   // MS561101BA_OSR_1024 2.3ms 0x04
    2500,   // MS561101BA_OSR_1024 2.3ms
    5000,   // MS561101BA_OSR_2048 4.6ms 0x06
    5000,   // MS561101BA_OSR_2048 4.6ms
    11000,  // MS561101BA_OSR_4096 9.1ms 0x08
};

// 本地变量
static uint16_t MS5611_PROM_C[8]; //标定值存放
float ms5611_dT;

// 全局变量
bool g_ms5611_init_surcceed = false;
float g_ms5611_temp;// 单位 [温度 0.01度]
float g_ms5611_press;// 单位 [气压 100帕]

//静态函数声明
static unsigned char SPI2_ReadWriteByte(unsigned char TxData);
static bool ms5611_read_prom(void);
static uint16_t ms5611_crc4(uint16_t *data);
static void ms5611_reset(void);
static void ms5611_start_conversion(uint8_t command);
static uint32_t ms5611_read_adc(void);
static void ms5611_cal_temperature(void);
static void ms5611_cal_pressure(void);

/**
  * @brief    SPI2 read/write byte
  * @param    TxData, SPI 发送1字节数据
  * @retval   RxData, SPI 接收1字节数据
  */
static unsigned char SPI2_ReadWriteByte(unsigned char TxData)
{
    unsigned char RxData = 0;
    
    if (HAL_SPI_TransmitReceive(&hspi2, &TxData, &RxData, 1, 100) != HAL_OK) {
        RxData = 0;
    }

    return RxData;
}

/**
  * @brief    ms5611 read PROM:从Prom中读取原厂校准数据
  * @param    null
  * @retval   bool, true or false
  */
static bool ms5611_read_prom(void)
{ 
	uint16_t high_byte = 0,low_byte = 0;
	uint16_t i;
	uint16_t crc4;	
    
	for (i = 0; i < 8; i++) {	
		MS5611_CS_ENABLE;
        delay_us(2);	
		SPI2_ReadWriteByte(MS5611_CMD_PROM_RD + i * 2);
		high_byte = SPI2_ReadWriteByte(0x00);   //读数 	
		low_byte = SPI2_ReadWriteByte(0x00);    //读数 		
		MS5611_PROM_C[i] = (high_byte << 8) | low_byte;
		delay_us(2);
        MS5611_CS_DISABLE;
	}		
	
    // 获取CRC4校验值
    uint16_t crc_read = MS5611_PROM_C[7] & 0x0F;
    // 清除CRC4校验值
    MS5611_PROM_C[7] &= 0xFF00;
    crc4 = ms5611_crc4(MS5611_PROM_C);
    if (crc_read == crc4) {
        return true;
    } else {
        return false;
    }
}

/**
  * @brief    ms5611 crc4
  * @param    uint16_t *data, 16位无符号整形crc4校验
  * @retval   uint16_t, 16位无符号整形校验结果
  */
static uint16_t ms5611_crc4(uint16_t* data)
{
    uint16_t n_rem = 0;
    uint8_t n_bit;

    for (uint8_t cnt = 0; cnt < 16; cnt++) {
        if (cnt & 1) {
            n_rem ^= (uint8_t)((data[cnt >> 1]) & 0x00FF);
        } else {
            n_rem ^= (uint8_t)(data[cnt >> 1] >> 8);
        }

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    return (n_rem >> 12) & 0xF;
}

/**
  * @brief    ms5611 reset:复位ms5611，写入0X1E
  * @param    none
  * @retval   none
  */
static void ms5611_reset(void)
{
	MS5611_CS_ENABLE;
    delay_us(2);
	SPI2_ReadWriteByte(MS5611_CMD_RESET);
	osDelay(100);   //10ms
	delay_us(2);
    MS5611_CS_DISABLE;
}

/**
  * @brief    ms5611 start Conversion
  * @param    uint8_t command, 转换命令
  * @retval   null
  */
static void ms5611_start_conversion(uint8_t command)
{
    // initialize pressure conversion
	MS5611_CS_ENABLE;
    delay_us(2);
	SPI2_ReadWriteByte(command);			 //发送转换指令
	delay_us(2);
    MS5611_CS_DISABLE;
}

/**
  * @brief    ms5611 read adc():读取 ms5611 的转换结果	
  * @param    null
  * @retval   uint32_t result, 转换结果
  */
static uint32_t ms5611_read_adc(void)
{
	uint32_t result = 0;
	uint32_t temp[3] = {0,0,0};
	
	MS5611_CS_ENABLE;
    delay_us(2);
	SPI2_ReadWriteByte(MS5611_CMD_ADC_READ);
	temp[0] = SPI2_ReadWriteByte(0x00);           //读数	
	temp[1] = SPI2_ReadWriteByte(0x00);           //读数 	
	temp[2] = SPI2_ReadWriteByte(0x00);           //读数 
    delay_us(2);
    MS5611_CS_DISABLE;	
	result = (temp[0]<<16) | (temp[1]<<8) | temp[2];
    
	return result;
}

/**
  * @brief    ms5611 Cal Temperature():计算温度值，单位0.01度
  * @param    null
  * @retval   uint32_t, 转换结果
  */
static void ms5611_cal_temperature(void)
{
	//获取原始数据
	uint32_t D2 = ms5611_read_adc();
	//温度校准
	ms5611_dT = (float)((int32_t)D2 - ((int32_t)MS5611_PROM_C[5] << 8));
	g_ms5611_temp = 2000.0f + (ms5611_dT * MS5611_PROM_C[6]) / 8388608.0f;
}

/**
  * @brief    ms5611_Cal_Pressure(): 计算气压值
  * @param    null
  * @retval   null
  */
static void ms5611_cal_pressure(void) 
{
	float MS5611_OFF, MS5611_SENS, MS5611_OFF2, MS5611_SENS2, MS5611_T2;
	float temp;
    
	//获取原始数据
	uint32_t D1 = ms5611_read_adc();
	//气压校准
	MS5611_OFF = MS5611_PROM_C[2] * 65536.0f + MS5611_PROM_C[4] / 128.0f * ms5611_dT;
	MS5611_SENS = MS5611_PROM_C[1] * 32768.0f + MS5611_PROM_C[3]  / 256.0f * ms5611_dT;
	
	if (g_ms5611_temp < 2000.0f) {
		MS5611_T2 = (ms5611_dT * ms5611_dT) / 0x80000000;
		temp = (g_ms5611_temp - 2000.0f) * (g_ms5611_temp - 2000.0f);
		MS5611_OFF2 = temp * 2.5f;
		MS5611_SENS2 = temp * 1.25f;
		if (g_ms5611_temp < -1500.0f) {
			temp = (g_ms5611_temp + 1500.0f) * (g_ms5611_temp + 1500.0f);
			MS5611_OFF2 = MS5611_OFF2 + 7.0f * temp;
			MS5611_SENS2 = MS5611_SENS2 + 5.5f * temp;
		}
	} else {
		MS5611_OFF2  = 0.0f;
		MS5611_SENS2 = 0.0f;
		MS5611_T2    = 0.0f;	
	}
	g_ms5611_temp = g_ms5611_temp - MS5611_T2;
	MS5611_OFF = MS5611_OFF - MS5611_OFF2;
	MS5611_SENS = MS5611_SENS - MS5611_SENS2;

	g_ms5611_press = (D1 * (MS5611_SENS / 2097152.0f) - MS5611_OFF) / 32768.0f;
}

/**
  * @brief    ms5611 init:初始化ms5611
  * @param    null
  * @retval   bool, true or false
  */
bool ms5611_init(void)
{
	ms5611_state = MS5611_START_CONVERT_TEMP;
    ms5611_reset();
	osDelay(1000); //1000ms
	delay_us(2);
	g_ms5611_init_surcceed = ms5611_read_prom();
    
	return g_ms5611_init_surcceed;
}

/**
  * @brief    MS5611_Update(): MS5611运行函数，周期性调用，以更新气压值和温度值 
  * @param    null
  * @retval   bool, true or false
  */
bool ms5611_update(void) 
{	
	static uint32_t current_delay;	   		//转换延时时间 us 
	static uint32_t start_convert_time;   //启动转换时的 时间 us 
	bool conversion_complete = false;

	if (!g_ms5611_init_surcceed) {				//初始化失败直接退出
		return false;
	}
	
	switch(ms5611_state) {
		case MS5611_START_CONVERT_TEMP:  //启动温度转换
			ms5611_start_conversion(MS5611_CMD_ADC_D2 + MS5611_CMD_ADC_1024);
			current_delay = ms5611_delay_us[MS5611_CMD_ADC_1024] ;//转换时间
			start_convert_time = micros(); //计时开始
			ms5611_state = MS5611_CONVERTING_TEMP;//下一个状态
			break;
		case MS5611_CONVERTING_TEMP:  //正在转换中 
			if ((micros()- start_convert_time) > current_delay) { //延时时间到了吗？
				ms5611_cal_temperature(); //取温度	
				ms5611_state = MS5611_START_CONVERT_PRESS;	
			}
			break;
		case MS5611_START_CONVERT_PRESS: //启动气压转换
			ms5611_start_conversion(MS5611_CMD_ADC_D1 + MS5611_CMD_ADC_1024);
			current_delay = ms5611_delay_us[MS5611_CMD_ADC_1024];//转换时间
			start_convert_time = micros();//计时开始
			ms5611_state = MS5611_CONVERTING_PRESS;//下一个状态
			break;
		case MS5611_CONVERTING_PRESS:	 //正在转换气压值
			if ((micros() - start_convert_time) > current_delay) { //延时时间到了吗？
				ms5611_cal_pressure();  //更新 	
				conversion_complete = true;
				ms5611_state = MS5611_START_CONVERT_TEMP; //从头再来	
			}
			break;
		default: 
			ms5611_state = MS5611_START_CONVERT_TEMP;
			break;
	}
	return conversion_complete;
}

inline float ms5611_get_temperature_c(void)
{
	if (g_ms5611_init_surcceed) {
		return (float)(g_ms5611_temp * 0.01f);
	} else {
		return 0.0f;
	}
}

inline float ms5611_get_Pressure_pa(void)
{
	if (g_ms5611_init_surcceed) {
		return (float)g_ms5611_press;
	} else {
		return 0.0f;
	}
}

//------------------End of File----------------------------
