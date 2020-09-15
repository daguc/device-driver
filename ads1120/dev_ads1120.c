#include "dev_ADS1120.h"
#include "spi.h"
#include <string.h>
#include "tim.h"

// macro definition
#define ADS1120_VREF    (1.65f) // Vref is 1.65V
#define ADS1120_GAIN    (8)     // Gain is 8
#define ADS1120_LSB	    ((2 * ADS1120_VREF / ADS1120_GAIN) / 65536) // 1 LSB = (2 * Vref / Gain) / 2^16

// global variable
Adc1120DataTypedef g_pt100_l;
Adc1120DataTypedef g_pt100_r;

// 本地函数
/* ADS1120 Higher Level Functions */
long ADS1120ReadData(void);						/* Read the data results */
void ADS1120ReadRegister(int StartAddress, int NumRegs, unsigned * pData);	/* Read the register(s) */
void ADS1120WriteRegister(int StartAddress, int NumRegs, unsigned * pData); /* Write the register(s) */
void ADS1120SendResetCommand(void);				/* Send a device Reset Command */
void ADS1120SendStartCommand(void);				/* Send a Start/SYNC command */
void ADS1120SendShutdownCommand(void);			/* Place the device in powerdown mode */

/* Register Set Value Commands */
void ADS1120Config(void);
int ADS1120SetChannel(int Mux);
int ADS1120SetGain(int Gain);
int ADS1120SetPGABypass(int Bypass);
int ADS1120SetDataRate(int DataRate);
int ADS1120SetClockMode(int ClockMode);
int ADS1120SetPowerDown(int PowerDown);
int ADS1120SetTemperatureMode(int TempMode);
int ADS1120SetBurnOutSource(int BurnOut);
int ADS1120SetVoltageReference(int VoltageRef);
int ADS1120Set50_60Rejection(int Rejection);
int ADS1120SetLowSidePowerSwitch(int PowerSwitch);
int ADS1120SetCurrentDACOutput(int CurrentOutput);
int ADS1120SetIDACRouting(int IDACRoute);
int ADS1120SetDRDYMode(int DRDYMode);

/* Register Get Value Commands */
int ADS1120GetChannel(void);
int ADS1120GetGain(void);
int ADS1120GetPGABypass(void);
int ADS1120GetDataRate(void);
int ADS1120GetClockMode(void);
int ADS1120GetPowerDown(void);
int ADS1120GetTemperatureMode(void);
int ADS1120GetBurnOutSource(void);
int ADS1120GetVoltageReference(void);
int ADS1120Get50_60Rejection(void);
int ADS1120GetLowSidePowerSwitch(void);
int ADS1120GetCurrentDACOutput(void);
int ADS1120GetIDACRouting(int WhichOne);
int ADS1120GetDRDYMode(void);
/* Useful Functions within Main Program for Setting Register Contents
*
*  	These functions show the programming flow based on the header definitions.
*  	The calls are not made within the demo example, but could easily be used by calling the function
*  		defined within the program to complete a fully useful program.
*	Similar function calls were made in the firmware design for the ADS1120EVM.
*  
*  The following function calls use ASCII data sent from a COM port to control settings 
*	on the ADS1120.  The data is reconstructed from ASCII and then combined with the
*	register contents to save as new configuration settings.
*
* 	Function names correspond to datasheet register definitions
*/
void set_MUX(char c);
void set_GAIN(char c);
void set_PGA_BYPASS(char c);
void set_DR(char c);
void set_MODE(char c);
void set_CM(char c);
void set_TS(char c);
void set_BCS(char c);
void set_VREF(char c);
void set_50_60(char c);
void set_PSW(char c);
void set_IDAC(char c);
void set_IMUX(char c, int i);
void set_DRDYM(char c);
void set_ERROR(void);

void ADS1120_global_var_init(void)
{
    // global varibale init
    memset(&g_pt100_l, 0, sizeof(Adc1120DataTypedef));
    memset(&g_pt100_r, 0, sizeof(Adc1120DataTypedef));
}

// ADS1120 CS chip select
void ADS1120AssertCS(int fAssert)
{
    if (fAssert) {
        // pull down cs
        SPI1_CS0_ENABLE;
    } else {
        // pull up cs
        SPI1_CS0_DISABLE;
    }
}

// ADS1120 send byte
void ADS1120SendByte(unsigned char Byte)
{	
    HAL_SPI_Transmit(&hspi1, &Byte, 1, 100);
}

unsigned char ADS1120ReceiveByte(void)
{
    unsigned char result = 0;

    HAL_SPI_Receive(&hspi1, &result, 1, 100);
    
	return result;
}

/*
******************************************************************************
 higher level functions
*/
long ADS1120ReadData(void)
{
    long Data;
    /* assert CS to start transfer */
    ADS1120AssertCS(1);
    /* send the command byte */
    ADS1120SendByte(ADS1120_CMD_RDATA);
    /* get the conversion result */
#ifdef ADS1120
    Data = ADS1120ReceiveByte();
    Data = (Data << 8) | ADS1120ReceiveByte();
    /* sign extend data */
    if (Data & 0x8000)
        Data |= 0xffff0000;
#else
    Data = ADS1120ReceiveByte();
    Data = (Data << 8) | ADS1120ReceiveByte();
    Data = (Data << 8) | ADS1120ReceiveByte();
    /* sign extend data */
    if (Data & 0x800000)
        Data |= 0xff000000; 
#endif
    /* de-assert CS */
    ADS1120AssertCS(0);
    return Data;
}
void ADS1120ReadRegister(int StartAddress, int NumRegs, unsigned * pData)
{
    int i;
	/* assert CS to start transfer */
	ADS1120AssertCS(1);
 	/* send the command byte */
	ADS1120SendByte(ADS1120_CMD_RREG | (((StartAddress<<2) & 0x0c) |((NumRegs-1)&0x03)));
   	/* get the register content */
	for (i=0; i< NumRegs; i++)
	{
		*pData++ = ADS1120ReceiveByte();
	}
   	/* de-assert CS */
	ADS1120AssertCS(0);
	return;
}
void ADS1120WriteRegister(int StartAddress, int NumRegs, unsigned * pData)
{
	int i;
   	/* assert CS to start transfer */
	ADS1120AssertCS(1);
   	/* send the command byte */
	ADS1120SendByte(ADS1120_CMD_WREG | (((StartAddress<<2) & 0x0c) |((NumRegs-1)&0x03)));
    /* send the data bytes */
	for (i=0; i< NumRegs; i++)
	{
		ADS1120SendByte(*pData++);
	}
   	/* de-assert CS */
	ADS1120AssertCS(0);
   	return;
}
void ADS1120SendResetCommand(void)
{
	/* assert CS to start transfer */
	ADS1120AssertCS(1);
   	/* send the command byte */
	ADS1120SendByte(ADS1120_CMD_RESET);
   	/* de-assert CS */
	ADS1120AssertCS(0);
   	return;
}
void ADS1120SendStartCommand(void)
{
	/* assert CS to start transfer */
	ADS1120AssertCS(1);
    /* send the command byte */
	ADS1120SendByte(ADS1120_CMD_SYNC);
   	/* de-assert CS */
	ADS1120AssertCS(0);
    return;
}
void ADS1120SendShutdownCommand(void)
{
	/* assert CS to start transfer */
	ADS1120AssertCS(1);
   	/* send the command byte */
	ADS1120SendByte(ADS1120_CMD_SHUTDOWN);
   	/* de-assert CS */
	ADS1120AssertCS(0);
    return;
}
/*
******************************************************************************
register set value commands
*/
int ADS1120SetChannel(int Mux)
{
    unsigned int cMux = Mux;	   
    /* write the register value containing the new value back to the ADS */
    ADS1120WriteRegister(ADS1120_0_REGISTER, 0x01, &cMux);
    return ADS1120_NO_ERROR;
}
int ADS1120SetGain(int Gain)
{
	unsigned int cGain = Gain;   
	/* write the register value containing the new code back to the ADS */
	ADS1120WriteRegister(ADS1120_0_REGISTER, 0x01, &cGain);
	return ADS1120_NO_ERROR;
}
int ADS1120SetPGABypass(int Bypass)
{
	unsigned int cBypass = Bypass;
	/* write the register value containing the new code back to the ADS */
	ADS1120WriteRegister(ADS1120_0_REGISTER, 0x01, &cBypass);
	return ADS1120_NO_ERROR;
}
int ADS1120SetDataRate(int DataRate)
{
	unsigned int cDataRate = DataRate;  
	/* write the register value containing the new value back to the ADS */
	ADS1120WriteRegister(ADS1120_1_REGISTER, 0x01, &cDataRate);
	return ADS1120_NO_ERROR;
}
int ADS1120SetClockMode(int ClockMode)
{
	unsigned int cClockMode = ClockMode;
   	/* write the register value containing the value back to the ADS */
	ADS1120WriteRegister(ADS1120_1_REGISTER, 0x01, &cClockMode);
	return ADS1120_NO_ERROR;
}
int ADS1120SetPowerDown(int PowerDown)
{
	unsigned int cPowerDown = PowerDown;
   	/* write the register value containing the new value back to the ADS */
	ADS1120WriteRegister(ADS1120_1_REGISTER, 0x01, &cPowerDown);
	return ADS1120_NO_ERROR;
}
int ADS1120SetTemperatureMode(int TempMode)
{
	unsigned int cTempMode = TempMode;
   	/* write the register value containing the new value back to the ADS */
	ADS1120WriteRegister(ADS1120_1_REGISTER, 0x01, &cTempMode);
	return ADS1120_NO_ERROR;
}
int ADS1120SetBurnOutSource(int BurnOut)
{
	unsigned int cBurnOut = BurnOut;
   	/* write the register value containing the new value back to the ADS */
	ADS1120WriteRegister(ADS1120_1_REGISTER, 0x01, &cBurnOut);
	return ADS1120_NO_ERROR;
}
int ADS1120SetVoltageReference(int VoltageRef)
{
	unsigned int cVoltageRef = VoltageRef;
   	/* write the register value containing the new value back to the ADS */
	ADS1120WriteRegister(ADS1120_2_REGISTER, 0x01, &cVoltageRef);
	return ADS1120_NO_ERROR;
}
int ADS1120Set50_60Rejection(int Rejection)
{
	unsigned int cRejection = Rejection;
   	/* write the register value containing the new value back to the ADS */
	ADS1120WriteRegister(ADS1120_2_REGISTER, 0x01, &cRejection);
	return ADS1120_NO_ERROR;
}
int ADS1120SetLowSidePowerSwitch(int PowerSwitch)
{
	unsigned int cPowerSwitch = PowerSwitch;
   	/* write the register value containing the new value back to the ADS */
	ADS1120WriteRegister(ADS1120_2_REGISTER, 0x01, &cPowerSwitch);
	return ADS1120_NO_ERROR;
}
int ADS1120SetCurrentDACOutput(int CurrentOutput)
{
	unsigned int cCurrentOutput = CurrentOutput;
   	/* write the register value containing the new value back to the ADS */
	ADS1120WriteRegister(ADS1120_2_REGISTER, 0x01, &cCurrentOutput);
	return ADS1120_NO_ERROR;
}
int ADS1120SetIDACRouting(int IDACRoute)
{
	unsigned int cIDACRoute = IDACRoute;
	/* write the register value containing the new value back to the ADS */
	ADS1120WriteRegister(ADS1120_3_REGISTER, 0x01, &cIDACRoute);
	return ADS1120_NO_ERROR;
}
int ADS1120SetDRDYMode(int DRDYMode)
{
	unsigned int cDRDYMode = DRDYMode;
   	/* write the register value containing the new gain code back to the ADS */
	ADS1120WriteRegister(ADS1120_3_REGISTER, 0x01, &cDRDYMode);
	return ADS1120_NO_ERROR;
}
/*
******************************************************************************
register get value commands
*/
int ADS1120GetChannel(void)
{
	unsigned Temp;
	/* Parse Mux data from register */
	ADS1120ReadRegister(ADS1120_0_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return (Temp >>4);
}
int ADS1120GetGain(void)
{
	unsigned Temp;
	/* Parse Gain data from register */
	ADS1120ReadRegister(ADS1120_0_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( (Temp & 0x0e) >>1);
}
int ADS1120GetPGABypass(void)
{
	unsigned Temp;
	/* Parse Bypass data from register */
	ADS1120ReadRegister(ADS1120_0_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return (Temp & 0x01);
}
int ADS1120GetDataRate(void)
{
	unsigned Temp;
	/* Parse DataRate data from register */
	ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( Temp >>5 );
}
int ADS1120GetClockMode(void)
{
	unsigned Temp;
	/* Parse ClockMode data from register */
	ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( (Temp & 0x18) >>3 );
}
int ADS1120GetPowerDown(void)
{
	unsigned Temp;
	/* Parse PowerDown data from register */
	ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( (Temp & 0x04) >>2 );
}
int ADS1120GetTemperatureMode(void)
{
	unsigned Temp;
	/* Parse TempMode data from register */
	ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( (Temp & 0x02) >>1 );
}
int ADS1120GetBurnOutSource(void)
{
	unsigned Temp;
	/* Parse BurnOut data from register */
	ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( Temp & 0x01 );
}
int ADS1120GetVoltageReference(void)
{
	unsigned Temp;
	/* Parse VoltageRef data from register */
	ADS1120ReadRegister(ADS1120_2_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( Temp >>6 );
}
int ADS1120Get50_60Rejection(void)
{
	unsigned Temp;
	/* Parse Rejection data from register */
	ADS1120ReadRegister(ADS1120_2_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( (Temp & 0x30) >>4 );
}
int ADS1120GetLowSidePowerSwitch(void)
{
	unsigned Temp;
	/* Parse PowerSwitch data from register */
	ADS1120ReadRegister(ADS1120_2_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( (Temp & 0x08) >>3);
}
int ADS1120GetCurrentDACOutput(void)
{
	unsigned Temp;
	/* Parse IDACOutput data from register */
	ADS1120ReadRegister(ADS1120_2_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( Temp & 0x07 );
}
int ADS1120GetIDACRouting(int WhichOne)
{
	/* Check WhichOne sizing */
	if (WhichOne >1) return ADS1120_ERROR;
	unsigned Temp;
	/* Parse Mux data from register */
	ADS1120ReadRegister(ADS1120_3_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	if (WhichOne) return ( (Temp & 0x1c) >>2);
	else return ( Temp >>5 );
}
int ADS1120GetDRDYMode(void)
{
	unsigned Temp;
	/* Parse DRDYMode data from register */
	ADS1120ReadRegister(ADS1120_3_REGISTER, 0x01, &Temp);
	/* return the parsed data */
	return ( (Temp & 0x02) >>1 );
}
/* Useful Functions within Main Program for Setting Register Contents
*
*  	These functions show the programming flow based on the header definitions.
*  	The calls are not made within the demo example, but could easily be used by calling the function
*  		defined within the program to complete a fully useful program.
*	Similar function calls were made in the firwmare design for the ADS1120EVM.
*  
*  The following function calls use ASCII data sent from a COM port to control settings 
*	on the ADS1120.  The data is recontructed from ASCII and then combined with the
*	register contents to save as new configuration settings.
*
* 	Function names correspond to datasheet register definitions
*/
void set_MUX(char c)
{	
	int mux = (int) c - 48;
	int dERROR;
	unsigned Temp;
	if (mux>=49 && mux<=54) mux -= 39;
	/* The MUX value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_0_REGISTER, 0x01, &Temp);
	Temp &= 0x0f;		
    /* Set multiplexer

    | Value | AINp | AINn |
    | ----- | ---- | ---- |
    | 0x00  | AIN0 | AIN1 |
    | 0X01  | AIN0 | AIN2 |
    | 0X02  | AIN0 | AIN3 |
    | 0X03  | AIN1 | AIN2 |
    | 0X04  | AIN1 | AIN3 |
    | 0X05  | AIN2 | AIN3 |
    | 0X06  | AIN1 | AIN0 |
    | 0X07  | AIN3 | AIN2 |
    | 0X08  | AIN0 | AVSS |
    | 0X09  | AIN1 | AVSS |
    | 0X0A  | AIN2 | AVSS |
    | 0X0B  | AIN3 | AVSS |
    | 0X0C  |  REF/4 MON  |
    | 0X0D  | APWR/4 MON  |
    | 0X0E  |   SHORTED   |
    *//* strip out old settings */
	/* Change Data rate */
	switch(mux) {
		case 0:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_0_1);
			break;
		case 1:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_0_2);
			break;
		case 2:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_0_3);
			break;
		case 3:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_1_2);
			break;
		case 4:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_1_3);
			break;
		case 5:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_2_3);
			break;
		case 6:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_1_0);
			break;
		case 7:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_3_2);
			break;
		case 8:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_0_G);
			break;
		case 9:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_1_G);
			break;
		case 10:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_2_G);
			break;
		case 11:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_3_G);
			break;
		case 12:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_EX_VREF);
			break;
		case 13:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_AVDD);
			break;
		case 14:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_DIV2);
			break;
		case 15:
			dERROR = ADS1120SetChannel(Temp + ADS1120_MUX_DIV2);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;												
	}
	if (dERROR==ADS1120_ERROR)
		set_ERROR();
}
void set_GAIN(char c)
{
	int pga = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* The GAIN value is only part of the register, so we have to read it back
	   and massage the new value into it*/
	ADS1120ReadRegister(ADS1120_0_REGISTER, 0x01, &Temp);
		Temp &= 0xf1;									/* strip out old settings */
	/* Change gain rate */
	switch(pga) {
		case 0:
			dERROR = ADS1120SetGain(Temp + ADS1120_GAIN_1);
			break;
		case 1:
			dERROR = ADS1120SetGain(Temp + ADS1120_GAIN_2);
			break;
		case 2:
			dERROR = ADS1120SetGain(Temp + ADS1120_GAIN_4);
			break;
		case 3:
			dERROR = ADS1120SetGain(Temp + ADS1120_GAIN_8);
			break;
		case 4:
			dERROR = ADS1120SetGain(Temp + ADS1120_GAIN_16);
			break;
		case 5:
			dERROR = ADS1120SetGain(Temp + ADS1120_GAIN_32);
			break;
		case 6:
			dERROR = ADS1120SetGain(Temp + ADS1120_GAIN_64);
			break;
		case 7:
			dERROR = ADS1120SetGain(Temp + ADS1120_GAIN_128);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;	
		}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_PGA_BYPASS(char c)
{
	int buff = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the PGA Bypass value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_0_REGISTER, 0x01, &Temp);
	Temp &= 0xfe;									/* strip out old settings */
	/* Change PGA Bypass */
	switch(buff) {
		case 0:
			dERROR = ADS1120SetPGABypass(Temp);
			break;
		case 1:
			dERROR = ADS1120SetPGABypass(Temp + ADS1120_PGA_BYPASS);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
	}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_DR(char c)
{
	int spd = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the DataRate value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, &Temp);
	Temp &= 0x1f;									/* strip out old settings */
	/* Change Data rate */
	switch(spd) {
		case 0:
			dERROR = ADS1120SetDataRate(Temp + ADS1120_DR_20);
			break;
		case 1:
			dERROR = ADS1120SetDataRate(Temp + ADS1120_DR_45);
			break;
		case 2:
			dERROR = ADS1120SetDataRate(Temp + ADS1120_DR_90);
			break;
		case 3:
			dERROR = ADS1120SetDataRate(Temp + ADS1120_DR_175);
			break;
		case 4:
			dERROR = ADS1120SetDataRate(Temp + ADS1120_DR_330);
			break;
		case 5:
			dERROR = ADS1120SetDataRate(Temp + ADS1120_DR_600);
			break;
		case 6:
			dERROR = ADS1120SetDataRate(Temp + ADS1120_DR_1000);
			break;
		case 7:
			dERROR = ADS1120SetDataRate(Temp + ADS1120_DR_1000);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
	}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_MODE(char c)
{
	int spd = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the MODE value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, &Temp);
	Temp &= 0xe7;									/* strip out old settings */
	/* Change the operating Mode */
	switch(spd) {
		case 0:
			dERROR = ADS1120SetClockMode(Temp + ADS1120_MODE_NORMAL);
			break;
		case 1:
			dERROR = ADS1120SetClockMode(Temp + ADS1120_MODE_DUTY);
			break;
		case 2:
			dERROR = ADS1120SetClockMode(Temp + ADS1120_MODE_TURBO);
			break;
		case 3:
			dERROR = ADS1120SetClockMode(Temp + ADS1120_MODE_DCT);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
	}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_CM(char c)
{
	int pwrdn = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the Conversion Mode value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, &Temp);
	Temp &= 0xfb;									/* strip out old settings */
	/* Change power down mode */
	switch(pwrdn) {
		case 0:
			dERROR = ADS1120SetPowerDown(Temp);
			break;
		case 1:
			dERROR = ADS1120SetPowerDown(Temp + ADS1120_CC);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
	}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_TS(char c)
{
	int tmp = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the Temperature Sensor mode value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, &Temp);
	Temp &= 0xfd;									/* strip out old settings */
	/* Change Temp Diode Setting */
	switch(tmp) {
		case 0:
			dERROR = ADS1120SetTemperatureMode(Temp);
			break;
		case 1:
			dERROR = ADS1120SetTemperatureMode(Temp + ADS1120_TEMP_SENSOR);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
	}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_BCS(char c)
{
	int bo = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the Burnout Current Source value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, &Temp);
	Temp &= 0xfe;									/* strip out old settings */
	/* Change Burnout Current */
	switch(bo) {
		case 0:
			dERROR = ADS1120SetBurnOutSource(Temp);
			break;
		case 1:
			dERROR = ADS1120SetBurnOutSource(Temp + ADS1120_BCS);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
	}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_VREF(char c)
{
	int ref = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the Voltage Reference value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_2_REGISTER, 0x01, &Temp);
	Temp &= 0x3f;									/* strip out old settings */
	/* Change Reference */
	switch(ref) {
		case 0:
			dERROR = ADS1120SetVoltageReference(Temp + ADS1120_VREF_INT);
			break;
		case 1:
			dERROR = ADS1120SetVoltageReference(Temp + ADS1120_VREF_EX_DED);
			break;
		case 2:
			dERROR = ADS1120SetVoltageReference(Temp + ADS1120_VREF_EX_AIN);
			break;
		case 3:
			dERROR = ADS1120SetVoltageReference(Temp + ADS1120_VREF_SUPPLY);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
	}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_50_60(char c)
{
	int flt = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the Digital Filter value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_2_REGISTER, 0x01, &Temp);
	Temp &= 0xcf;									/* strip out old settings */
	/* Change Filter Setting */
	switch(flt) {
		case 0:
			dERROR = ADS1120Set50_60Rejection(Temp + ADS1120_REJECT_OFF);
			break;
		case 1:
			dERROR = ADS1120Set50_60Rejection(Temp + ADS1120_REJECT_BOTH);
			break;
		case 2:
			dERROR = ADS1120Set50_60Rejection(Temp + ADS1120_REJECT_50);
			break;
		case 3:
			dERROR = ADS1120Set50_60Rejection(Temp + ADS1120_REJECT_60);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
	}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_PSW(char c)
{
	int sw = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the Low-side Switch value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_2_REGISTER, 0x01, &Temp);
	Temp &= 0xf7;									/* strip out old settings */
		/* Change low-side switch mode */
	switch(sw) {
		case 0:
			dERROR = ADS1120SetLowSidePowerSwitch(Temp);
			break;
		case 1:
			dERROR = ADS1120SetLowSidePowerSwitch(Temp + ADS1120_PSW_SW);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
	}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_IDAC(char c)
{
	int current = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the IDAC Current value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_2_REGISTER, 0x01, &Temp);
	Temp &= 0xf8;									/* strip out old settings */
	/* Change IDAC Current Output */
	switch(current) {
		case 0:
			dERROR = ADS1120SetCurrentDACOutput(Temp + ADS1120_IDAC_OFF);
			break;
		case 1:
#ifdef ADS1120
			dERROR = ADS1120SetCurrentDACOutput(Temp + ADS1120_IDAC_OFF);
#else
			dERROR = ADS1120SetCurrentDACOutput(Temp + ADS1120_IDAC_10);
#endif
			break;
		case 2:
			dERROR = ADS1120SetCurrentDACOutput(Temp + ADS1120_IDAC_50);
			break;
		case 3:
			dERROR = ADS1120SetCurrentDACOutput(Temp + ADS1120_IDAC_100);
			break;
		case 4:
			dERROR = ADS1120SetCurrentDACOutput(Temp + ADS1120_IDAC_250);
			break;
		case 5:
			dERROR = ADS1120SetCurrentDACOutput(Temp + ADS1120_IDAC_500);
			break;
		case 6:
			dERROR = ADS1120SetCurrentDACOutput(Temp + ADS1120_IDAC_1000);
			break;
		case 7:
			dERROR = ADS1120SetCurrentDACOutput(Temp + ADS1120_IDAC_2000);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
		}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_IMUX(char c, int i)
{
	int mux = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the IDAC Mux value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_3_REGISTER, 0x01, &Temp);
	if (i==1) {
		Temp &= 0xe3;									/* strip out old settings */
		/* Change IDAC2 MUX Output */
		switch(mux) {
			case 0:
				Temp |= ADS1120_IDAC2_OFF;
				break;
			case 1:
				Temp |= ADS1120_IDAC2_AIN0;
				break;
			case 2:
				Temp |= ADS1120_IDAC2_AIN1;
				break;
			case 3:
				Temp |= ADS1120_IDAC2_AIN2;
				break;
			case 4:
				Temp |= ADS1120_IDAC2_AIN3;
				break;
			case 5:
				Temp |= ADS1120_IDAC2_REFP0;
				break;
			case 6:
				Temp |= ADS1120_IDAC2_REFN0;
				break;
			case 7:
				Temp |= ADS1120_IDAC2_REFN0;
				break;
			default:
				dERROR = ADS1120_ERROR;
				break;
		}
	}
	else {
		Temp &= 0x1f;
		/* Change IDAC1 MUX Output */
		switch(mux) {
			case 0:
				Temp |= ADS1120_IDAC1_OFF;
				break;
			case 1:
				Temp |= ADS1120_IDAC1_AIN0;
				break;
			case 2:
				Temp |= ADS1120_IDAC1_AIN1;
				break;
			case 3:
				Temp |= ADS1120_IDAC1_AIN2;
				break;
			case 4:
				Temp |= ADS1120_IDAC1_AIN3;
				break;
			case 5:
				Temp |= ADS1120_IDAC1_REFP0;
				break;
			case 6:
				Temp |= ADS1120_IDAC1_REFN0;
				break;
			case 7:
				Temp |= ADS1120_IDAC1_REFN0;
				break;
			default:
				dERROR = ADS1120_ERROR;
				break;
		}
	}
	if (dERROR==ADS1120_NO_ERROR) 
		dERROR = ADS1120SetIDACRouting(Temp); 
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_DRDYM(char c)
{
	int drdy = (int) c - 48;
	int dERROR;
	unsigned Temp;
	/* the DRDY output mode value is only part of the register, so we have to read it back
	   and massage the new value into it */
	ADS1120ReadRegister(ADS1120_3_REGISTER, 0x01, &Temp);
	Temp &= 0xfd;									/* strip out old settings */
	/* Change DRDY Mode Setting */
	switch(drdy) {
		case 0:
			dERROR = ADS1120SetDRDYMode(Temp);
			break;
		case 1:
			dERROR = ADS1120SetDRDYMode(Temp + ADS1120_DRDY_MODE);
			break;
		default:
			dERROR = ADS1120_ERROR;
			break;
	}
	if (dERROR==ADS1120_ERROR) 
		set_ERROR();
}
void set_ERROR(void)
{
	/* Add some error routine here is desired */
}

//test
unsigned char g_read_reg0 = 0;
unsigned char g_read_reg1 = 0;
unsigned char g_read_reg2 = 0;
unsigned char g_read_reg3 = 0;

/* ADS1120 Initial Configuration */
void ADS1120_init(void)
{
    ADS1120_global_var_init();
    // reset ads1120
    ADS1120SendResetCommand();
    // delay for a minimum of 50 µs + 32 · t(CLK)
    delay_us(100);
    // send start/sync for continous conversion mode
    ADS1120SendStartCommand();
    // set AINp = AIN1, AINn = AIN0
    set_MUX('6');
    // set Gain 8. Possible values are 1, 2, 4, 8, 16, 32, 64, 128.
    set_GAIN('3');
    // set Normal Mode
    set_MODE('0');
    // set Data rate 000.  In normal mode = 20sps
    set_DR('0');
    // set Continous Mode = 1 
    set_CM('1');
    // set external reference(REFP0, REFN0)
    set_VREF('1');
    // set simultaneous 50Hz and 60Hz rejection
    set_50_60('1');
    // set IDAC 500uA
    set_IDAC('5');
    // set INAC1 = AIN2
    set_IMUX('3', 0);
    // set INAC2 = AIN3
    set_IMUX('4', 1);
    delay_us(100);
    // get register 0,1,2,3 config
    ADS1120ReadRegister(ADS1120_0_REGISTER, 0x01, (unsigned *)&g_read_reg0);
    ADS1120ReadRegister(ADS1120_1_REGISTER, 0x01, (unsigned *)&g_read_reg1);
    ADS1120ReadRegister(ADS1120_2_REGISTER, 0x01, (unsigned *)&g_read_reg2);
    ADS1120ReadRegister(ADS1120_3_REGISTER, 0x01, (unsigned *)&g_read_reg3);
}

// ADS1120 update
void ADS1120_update(void)
{
    /* pt100 voltage difference */
    if (ADS1120_READY0_PIN == GPIO_PIN_RESET) {
        g_pt100_l.analog_diff_01 = ADS1120ReadData();
		g_pt100_l.vol_diff_01 = g_pt100_l.analog_diff_01 * ADS1120_LSB;
    }
}

