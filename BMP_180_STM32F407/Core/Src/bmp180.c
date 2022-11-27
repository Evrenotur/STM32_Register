

#include "bmp180.h"
// calibration datas
short 		    AC1 = 0;
short 		    AC2 = 0;
short 			AC3 = 0;
unsigned short 	AC4 = 0;
unsigned short 	AC5 = 0;
unsigned short 	AC6 = 0;
short 			B1 = 0;
short 			B2 = 0;
short 			MB = 0;
short 			MC = 0;
short 			MD = 0;


// calibration calculation
int16_t			UT = 0;
long 			UP = 0;
long 			X1 = 0;
long 			X2 = 0;
long 			X3 = 0;
long 			B3 = 0;
long 			B5 = 0;
unsigned long 	B4 = 0;
long 			B6 = 0;
unsigned long 	B7 = 0;



float TempCalculate;
float Presure;

void BMP180_Init()
{

 if(HAL_I2C_IsDeviceReady(&hi2c1, BMP180_DEVICE_WRITE_REG_ADRR, 1, 100000)!=HAL_OK)
 {
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
 }
 BMP180_Get_Calibration_Value();
}

void BMP180_Get_Calibration_Value()
{


	uint8_t calibDatas[BMP180_CALIBRATION_VALUE_LENGHT] = {0};
	  HAL_I2C_Mem_Read(&hi2c1, BMP180_DEVICE_READ_REG_ADRR, AC1_MSB, 1, calibDatas, BMP180_CALIBRATION_VALUE_LENGHT, 1000);

	AC1 = (( calibDatas[0] << 8) | calibDatas[1]);
	AC2 = (( calibDatas[2] << 8) | calibDatas[3]);
	AC3 = (( calibDatas[4] << 8) | calibDatas[5]);
	AC4 = (( calibDatas[6] << 8) | calibDatas[7]);
	AC5 = (( calibDatas[8] << 8) | calibDatas[9]);
	AC6 = ((calibDatas[10] << 8) | calibDatas[11]);
	B1 =  ((calibDatas[12] << 8) | calibDatas[13]);
	B2 =  ((calibDatas[14] << 8) | calibDatas[15]);
	MB =  ((calibDatas[16] << 8) | calibDatas[17]);
	MC =  ((calibDatas[18] << 8) | calibDatas[19]);
	MD =  ((calibDatas[20] << 8) | calibDatas[21]);
}


float BMP180_Get_Temperature_Value()
{
	BMP180_Calculate_Temperature_Value();
	return TempCalculate/10.0;
}
float BMP180_Get_Pressure_Value()
{
	BMP180_Calculate_Pressure_Value();
	return Presure;
}

float BMP180_Get_Altitude_Value(){
	BMP180_Get_Pressure_Value();
	return 44330*(1-(pow((Presure/(float)Po), 1/5.255)));
}
void BMP180_Calculate_Temperature_Value()
{
	UT =BMP180_Get_Uncompansated_Value();
	X1 = (UT - AC6)*AC5/(pow(2, 15));
	X2 = (MC*pow(2, 11))/(X1 + MD);
	B5 = X1 + X2;
	TempCalculate = (B5 + 8)/pow(2,4);
}


uint16_t BMP180_Get_Uncompansated_Value()
{
	 uint8_t writeData[1];
	 		writeData[0] = 0x2E;

	 	uint8_t UtBuffer[2];
	 	   HAL_I2C_Mem_Write(&hi2c1, BMP180_DEVICE_WRITE_REG_ADRR, UT_WRITEREG , 1, writeData, 1, 100000);
	 	   HAL_Delay(5);
	 	   HAL_I2C_Mem_Read(&hi2c1, BMP180_DEVICE_READ_REG_ADRR, UT_READMSBREG, 1, UtBuffer, 2, 100000);

	 	return (int16_t)((UtBuffer[0]<<8 )| UtBuffer[1]);
}
void BMP180_Calculate_Pressure_Value()
{


	UP=BMP180_Get_Uncompansated_Pressure_Value();
	B6 = B5-4000;
	X1 = (B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
	X2 = AC2*B6/(pow(2,11));
	X3 = X1+X2;
	B3 = (((AC1*4+X3)<<ultrahighresolution)+2)/4;
	X1 = AC3*B6/pow(2,13);
	X2 = (B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
	X3 = ((X1+X2)+2)/pow(2,2);
	B4 = AC4*(unsigned long)(X3+32768)/(pow(2,15));
	B7 = ((unsigned long)UP-B3)*(50000>>ultrahighresolution);
	Presure = (B7<0x80000000) ? (B7*2)/B4 : (B7/B4)*2;
	X1 = (Presure/(pow(2,8)))*(Presure/(pow(2,8)));
	X1 = (X1*3038)/(pow(2,16));
	X2 = (-7357*Presure)/(pow(2,16));
	Presure = Presure + (X1+X2+3791)/(pow(2,4));

}

long BMP180_Get_Uncompansated_Pressure_Value()
{
	uint8_t rData[3] = {0};
	uint8_t wData[1] = {0};
	wData[0] =0x34 |(ultrahighresolution  <<6);
	HAL_I2C_Mem_Write(&hi2c1, BMP180_DEVICE_WRITE_REG_ADRR, UT_WRITEREG , 1, wData, 1, 100000);
    HAL_Delay(26);
    HAL_I2C_Mem_Read(&hi2c1, BMP180_DEVICE_READ_REG_ADRR, UT_READMSBREG, 1, rData, 3, 100000);

    return ((rData[0]<<16 | rData[1]<<8 ) | rData[2]) >> (8-(uint8_t)ultrahighresolution);

}

