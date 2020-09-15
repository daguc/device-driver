#include "dev_ina226.h"
#include "i2c.h"
//#include "drv_sw_i2c.h"

// local variable
float vShuntMax, vBusMax, rShunt;
float currentLSB, powerLSB;

// global variable
INA226Typedef g_ina226_l;
INA226Typedef g_ina226_r;

// local static functions
static HAL_StatusTypeDef ina226_read_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t* pdata, uint8_t size);
static HAL_StatusTypeDef ina226_write_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t* pdata, uint8_t size);
static void swap_u16(uint16_t* pdata);
static bool ina226_l_config(void);
static bool ina226_l_configure(ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode);
static bool ina226_l_calibrate(float rShuntValue, float iMaxExpected);
static uint16_t ina226_l_get_id(void);
static uint16_t ina226_l_get_shunt_voltage(void);
static uint16_t ina226_l_get_vbus_voltage(void);
static uint16_t ina226_l_get_shunt_current(void);
static uint16_t ina226_l_get_power(void);
static bool ina226_r_config(void);
static bool ina226_r_configure(ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode);
static bool ina226_r_calibrate(float rShuntValue, float iMaxExpected);
static uint16_t ina226_r_get_id(void);
static uint16_t ina226_r_get_shunt_voltage(void);
static uint16_t ina226_r_get_vbus_voltage(void);
static uint16_t ina226_r_get_shunt_current(void);
static uint16_t ina226_r_get_power(void);
static void ina226_global_var_init(void);

/**
  * @brief	  ina226 read regs
  * @param	  uint8_t dev_addr 设备地址
  * @param	  uint8_t reg_addr 寄存器地址
  * @param	  uint8_t *pdata 读数据缓存指针
  * @param	  uint8_t size 数据长度
  * @retval   
  */
static HAL_StatusTypeDef ina226_read_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *pdata, uint8_t size)
{
	HAL_StatusTypeDef res = HAL_TIMEOUT;
#ifdef INA226_HW_I2C
	res = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)dev_addr, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, size, 10);
#elif defined(INA226_SW_I2C)
    if (I2C_ReadBuff(dev_addr, reg_addr, size, pdata)) {
        res = HAL_OK;
    }
#else
    #error "Please select first the target I2C used in your application (dev_ina226.h)"
#endif

	return res;
}

/**
  * @brief	  ina226 write regs
  * @param	  uint8_t dev_addr 设备地址
  * @param	  uint8_t reg_addr 寄存器地址
  * @param	  uint8_t *pdata 写数据缓存指针
  * @param	  uint8_t size 数据长度  
  * @retval   
  */
static HAL_StatusTypeDef ina226_write_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t* pdata, uint8_t size)
{
	HAL_StatusTypeDef res = HAL_TIMEOUT;
#if defined(INA226_HW_I2C)
	res = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)dev_addr, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, size, 10);
#elif defined(INA226_SW_I2C)
    if (I2C_WriteBuff(dev_addr, reg_addr, size, pdata)) {
        res = HAL_OK;
    }
#else
    #error "Please select first the target I2C used in your application (dev_ina226.h)"
#endif

	return res;
}

// swap uint16_t high 8 bit and low 8 bit
static void swap_u16(uint16_t* pdata)
{
    uint16_t temp;
    temp = *pdata;
    *pdata >>= 8;
    *pdata |= (temp << 8);
}

// ina226 l config
static bool ina226_l_config(void)
{
    bool res = false;   

    // Configure INA226
    res = ina226_l_configure(INA226_AVERAGES_64, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    if (!res) { // exception
        return res;
    }

    // Calibrate INA226. Rshunt = 0.02 ohm, Max excepted current = 4A
    res = ina226_l_calibrate(0.02f, 4.0f);
    if (!res) { // exception
        return res;
    }

    return HAL_OK;
}

/**
  * @brief	  ina226 l configure
  * @param	  ina226_averages_t avg 平均滤波窗口宽度
  * @param	  ina226_busConvTime_t busConvTime VBUS电压转换时间
  * @param	  ina226_shuntConvTime_t shuntConvTime SHUNT电压转换时间
  * @param	  ina226_mode_t mode 工作模式：电流、电压连续转换
  * @retval   
  */
static bool ina226_l_configure(ina226_averages_t avg, 
                      ina226_busConvTime_t busConvTime, 
                      ina226_shuntConvTime_t shuntConvTime, 
                      ina226_mode_t mode)
{
    HAL_StatusTypeDef res = HAL_TIMEOUT;
    ina226_data_t config;

    memset(&config, 0, sizeof(ina226_data_t));
    //vBusMax = 36;
    //vShuntMax = 0.08192f;
    config.data |= ((uint16_t)avg << 9 | (uint16_t)busConvTime << 6 | (uint16_t)shuntConvTime << 3 | (uint16_t)mode);
    swap_u16((uint16_t *)config.buffer);
    res = ina226_write_regs(INA226_L_ADDR, INA226_REG_CONFIG, config.buffer, 2);
    if (res != HAL_OK) {
        return false;
    }

    return true;
}

//rShuntValue 电阻值, mOhm
//iMaxExpected 最大电流值 A
static bool ina226_l_calibrate(float rShuntValue, float iMaxExpected)
{
    HAL_StatusTypeDef res = HAL_TIMEOUT;
    ina226_data_t calibrate;
    memset(&calibrate, 0, sizeof(ina226_data_t));
#if 0
    rShunt = rShuntValue;
    float /*iMaxPossible,*/ minimumLSB;
    //iMaxPossible = vShuntMax / rShunt;
    minimumLSB = iMaxExpected / 32767.0f;
    currentLSB = (uint16_t)(minimumLSB * 100000000);
    currentLSB /= 100000000;
    currentLSB /= 0.0001f;
    currentLSB = (uint16_t)(currentLSB);
    currentLSB *= 0.0001f;
#else
    rShunt = rShuntValue * 1000.0f; // Ohm
    currentLSB = INA226_CURRENT_LSB; // 0.025A/LSB
#endif

    //powerLSB = currentLSB * 25;
    calibrate.data = (uint16_t)((5120.0f) / (currentLSB * rShunt));
    swap_u16((uint16_t *)calibrate.buffer);
    res = ina226_write_regs(INA226_L_ADDR, INA226_REG_CALIBRATION, calibrate.buffer, 2);
    if (res != HAL_OK) {
        return false;
    }

    return true;
}

// get id
static uint16_t ina226_l_get_id(void)
{
    uint16_t id;
    uint8_t buffer[2] = {0, 0};
    
    if (ina226_read_regs(INA226_L_ADDR, INA226_REG_ID, buffer, 2) == HAL_OK) {
        id = ((uint16_t)buffer[0] << 8 | (uint16_t)buffer[1]);
    } else {
        id = 0;
        //printf("ina226_L get id error.\r\n");
    }

    return id;
}

// get shunt voltage
static uint16_t ina226_l_get_shunt_voltage(void)
{
    uint16_t data = 0;
    uint8_t buffer[2] = {0, 0};

    if (ina226_read_regs(INA226_L_ADDR, INA226_REG_SHUNTVOLTAGE, buffer, 2) == HAL_OK) {
        data = ((uint16_t)buffer[0] << 8 | (uint16_t)buffer[1]);
        if (data & 0x8000) {
            data = ~(data - 1);    
            //data = (uint16_t)data - 2 * (uint16_t)data;
        }
    } else {
        data = 0;
        //printf("ina226_L get shunt voltage error.\r\n");
    }

    return data;
}

// get bus voltage
static uint16_t ina226_l_get_vbus_voltage(void)
{
    uint16_t data = 0;
    uint8_t buffer[2] = {0, 0};

    if (ina226_read_regs(INA226_L_ADDR, INA226_REG_BUSVOLTAGE, buffer, 2) == HAL_OK) {
        data = ((uint16_t)buffer[0] << 8 | (uint16_t)buffer[1]);
        //vbus_voltage = data * INA226_VBUS_VOL_LSB;
    } else {
        data = 0;
        //printf("ina226_L get vbus voltage error.\r\n");
    }

    return data;
}

// get shunt current
static uint16_t ina226_l_get_shunt_current(void)
{
    uint16_t data = 0;
    uint8_t buffer[2] = {0, 0};

    if (ina226_read_regs(INA226_L_ADDR, INA226_REG_CURRENT, buffer, 2) == HAL_OK) {
        data = ((uint16_t)buffer[0] << 8 | (uint16_t)buffer[1]);
        if (data & 0x8000) {
            data = ~(data - 1);    
            //data = (uint16_t)data - 2 * (uint16_t)data;
        }
        //shunt_current = data * INA226_CURRENT_LSB;
    } else {
        data = 0;
        //printf("ina226_L get shunt current error.\r\n");
    }

    return data;
}

// get shunt current
static uint16_t ina226_l_get_power(void)
{
    uint16_t data = 0;
    uint8_t buffer[2] = {0, 0};

    if (ina226_read_regs(INA226_L_ADDR, INA226_REG_POWER, buffer, 2) == HAL_OK) {
        data = ((uint16_t)buffer[0] << 8 | (uint16_t)buffer[1]);
        //power = data * INA226_PWR_LSB;
    } else {
        data = 0;
        //printf("ina226_L get shunt current error.\r\n");
    }

    return data;
}

// ina226 r config
static bool ina226_r_config(void)
{
    bool res = false;   

    // Configure INA226
    res = ina226_r_configure(INA226_AVERAGES_64, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    if (!res) { // exception
        return res;
    }

    // Calibrate INA226. Rshunt = 0.02 ohm, Max excepted current = 4A
    res = ina226_r_calibrate(0.02f, 4.0f);
    if (!res) { // exception
        return res;
    }

    return HAL_OK;
}

/**
  * @brief	  ina226 r configure
  * @param	  ina226_averages_t avg 平均滤波窗口宽度
  * @param	  ina226_busConvTime_t busConvTime VBUS电压转换时间
  * @param	  ina226_shuntConvTime_t shuntConvTime SHUNT电压转换时间
  * @param	  ina226_mode_t mode 工作模式：电流、电压连续转换
  * @retval   
  */
static bool ina226_r_configure(ina226_averages_t avg, 
                      ina226_busConvTime_t busConvTime, 
                      ina226_shuntConvTime_t shuntConvTime, 
                      ina226_mode_t mode)
{
    HAL_StatusTypeDef res = HAL_TIMEOUT;
    ina226_data_t config;

    memset(&config, 0, sizeof(ina226_data_t));
    //vBusMax = 36;
    //vShuntMax = 0.08192f;
    config.data |= ((uint16_t)avg << 9 | (uint16_t)busConvTime << 6 | (uint16_t)shuntConvTime << 3 | (uint16_t)mode);
    swap_u16((uint16_t *)config.buffer);
    res = ina226_write_regs(INA226_R_ADDR, INA226_REG_CONFIG, config.buffer, 2);
    if (res != HAL_OK) {
        return false;
    }

    return true;
}

//rShuntValue 电阻值, mOhm
//iMaxExpected 最大电流值 A
static bool ina226_r_calibrate(float rShuntValue, float iMaxExpected)
{
    HAL_StatusTypeDef res = HAL_TIMEOUT;
    ina226_data_t calibrate;
    memset(&calibrate, 0, sizeof(ina226_data_t));
#if 0
    rShunt = rShuntValue;
    float /*iMaxPossible,*/ minimumLSB;
    //iMaxPossible = vShuntMax / rShunt;
    minimumLSB = iMaxExpected / 32767.0f;
    currentLSB = (uint16_t)(minimumLSB * 100000000);
    currentLSB /= 100000000;
    currentLSB /= 0.0001f;
    currentLSB = (uint16_t)(currentLSB);
    currentLSB *= 0.0001f;
#else
    rShunt = rShuntValue * 1000.0f; // Ohm
    currentLSB = INA226_CURRENT_LSB; // 0.025A/LSB
#endif

    //powerLSB = currentLSB * 25;
    calibrate.data = (uint16_t)((5120.0f) / (currentLSB * rShunt));
    swap_u16((uint16_t *)calibrate.buffer);
    res = ina226_write_regs(INA226_R_ADDR, INA226_REG_CALIBRATION, calibrate.buffer, 2);
    if (res != HAL_OK) {
        return false;
    }

    return true;
}

// get id
static uint16_t ina226_r_get_id(void)
{
    uint16_t id;
    uint8_t buffer[2] = {0, 0};
    
    if (ina226_read_regs(INA226_R_ADDR, INA226_REG_ID, buffer, 2) == HAL_OK) {
        id = ((uint16_t)buffer[0] << 8 | (uint16_t)buffer[1]);
    } else {
        id = 0;
        //printf("ina226_L get id error.\r\n");
    }

    return id;
}

// get shunt voltage
static uint16_t ina226_r_get_shunt_voltage(void)
{
    uint16_t data = 0;
    uint8_t buffer[2] = {0, 0};

    if (ina226_read_regs(INA226_R_ADDR, INA226_REG_SHUNTVOLTAGE, buffer, 2) == HAL_OK) {
        data = ((uint16_t)buffer[0] << 8 | (uint16_t)buffer[1]);
        if (data & 0x8000) {
            data = ~(data - 1);    
            //data = (uint16_t)data - 2 * (uint16_t)data;
        }
    } else {
        data = 0;
        //printf("ina226_L get shunt voltage error.\r\n");
    }

    return data;
}

// get bus voltage
static uint16_t ina226_r_get_vbus_voltage(void)
{
    uint16_t data = 0;
    uint8_t buffer[2] = {0, 0};

    if (ina226_read_regs(INA226_R_ADDR, INA226_REG_BUSVOLTAGE, buffer, 2) == HAL_OK) {
        data = ((uint16_t)buffer[0] << 8 | (uint16_t)buffer[1]);
        //vbus_voltage = data * INA226_VBUS_VOL_LSB;
    } else {
        data = 0;
        //printf("ina226_L get vbus voltage error.\r\n");
    }

    return data;
}

// get shunt current
static uint16_t ina226_r_get_shunt_current(void)
{
    uint16_t data = 0;
    uint8_t buffer[2] = {0, 0};

    if (ina226_read_regs(INA226_R_ADDR, INA226_REG_CURRENT, buffer, 2) == HAL_OK) {
        data = ((uint16_t)buffer[0] << 8 | (uint16_t)buffer[1]);
        if (data & 0x8000) {
            data = ~(data - 1);    
            //data = (uint16_t)data - 2 * (uint16_t)data;
        }
        //shunt_current = data * INA226_CURRENT_LSB;
    } else {
        data = 0;
        //printf("ina226_L get shunt current error.\r\n");
    }

    return data;
}

// get shunt current
static uint16_t ina226_r_get_power(void)
{
    uint16_t data = 0;
    uint8_t buffer[2] = {0, 0};

    if (ina226_read_regs(INA226_R_ADDR, INA226_REG_POWER, buffer, 2) == HAL_OK) {
        data = ((uint16_t)buffer[0] << 8 | (uint16_t)buffer[1]);
        //power = data * INA226_PWR_LSB;
    } else {
        data = 0;
        //printf("ina226_L get shunt current error.\r\n");
    }

    return data;
}

/**
  * @brief	  ina226 global variable init
  * @param	  null
  * @retval   null
  */
static void ina226_global_var_init(void)
{
    memset(&g_ina226_l, 0, sizeof(INA226Typedef));
    memset(&g_ina226_r, 0, sizeof(INA226Typedef));

    g_ina226_l.init_surcceed = false;
    g_ina226_r.init_surcceed = false;
}

/**
  * @brief	  ina226 init
  * @param	  null
  * @retval   null
  */
void ina226_init(void)
{
    // global init
    ina226_global_var_init();
    // get device id
    g_ina226_l.id = ina226_l_get_id();
    // config
    ina226_l_config();
    g_ina226_l.init_surcceed = true;

    // get device id
    g_ina226_r.id = ina226_r_get_id();
    // config
    ina226_r_config();
    g_ina226_r.init_surcceed = true;
}

/**
  * @brief	  ina226 update
  * @param	  null
  * @retval   null
  */
void ina226_update(void)
{
	g_ina226_l.vbus_mV = (uint16_t)(ina226_l_get_vbus_voltage() * INA226_VBUS_VOL_LSB);		    //mV
	g_ina226_l.shunt_uV = (uint16_t)(ina226_l_get_shunt_voltage() * INA226_SHUNTVOLTAGE_LSB); 	//uV
	g_ina226_l.shunt_mA = (uint16_t)(ina226_l_get_shunt_current() * INA226_CURRENT_LSB);	    //mA
	g_ina226_l.power_mW = (uint16_t)(ina226_l_get_power() * INA226_POWER_LSB);
    //获取功率(W) = 总线电压 * 电流
	g_ina226_l.power_W = (g_ina226_l.vbus_mV * 0.001f) * (g_ina226_l.shunt_mA * 0.001f);

    g_ina226_r.vbus_mV = (uint16_t)(ina226_r_get_vbus_voltage() * INA226_VBUS_VOL_LSB);		    //mV
	g_ina226_r.shunt_uV = (uint16_t)(ina226_r_get_shunt_voltage() * INA226_SHUNTVOLTAGE_LSB); 	//uV
	g_ina226_r.shunt_mA = (uint16_t)(ina226_r_get_shunt_current() * INA226_CURRENT_LSB);	    //mA
	g_ina226_r.power_mW = (uint16_t)(ina226_r_get_power() * INA226_POWER_LSB);
    //获取功率(W) = 总线电压 * 电流
	g_ina226_r.power_W = (g_ina226_r.vbus_mV * 0.001f) * (g_ina226_r.shunt_mA * 0.001f);
}
