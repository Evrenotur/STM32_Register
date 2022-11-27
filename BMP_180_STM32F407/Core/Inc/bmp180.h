/*
 * bmp180.h
 *
 *  Created on: Nov 25, 2022
 *      Author: Evren OTUR
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <math.h>
extern I2C_HandleTypeDef hi2c1;

/*BMP180 Device Address*/
#define BMP180_DEVICE_WRITE_REG_ADRR         0xEE
#define BMP180_DEVICE_READ_REG_ADRR          0xEF

/*BMP180 Calibration Value Length*/
#define BMP180_CALIBRATION_VALUE_LENGHT        22


#define Po 101325



/*BMP180 Register Address*/

typedef enum{

	AC1_MSB = 0xAA,
	AC1_LSB = 0xAB,
	AC2_MSB = 0xAC,
	AC2_LSB = 0xAD,
	AC3_MSB = 0xAE,
	AC3_LSB = 0xAF,
	AC4_MSB = 0xB0,
	AC4_LSB = 0xB1,
	AC5_MSB = 0xB2,
    AC5_LSB = 0xB3,
	AC6_MSB = 0xB4,
    AC6_LSB = 0xB5,

	B1_MSB = 0xB6,
	B1_LSB = 0xB7,
	B2_MSB = 0xB8,
	B2_LSB = 0xB9,

	MB_MSB = 0xBA,
	MB_LSB = 0xBB,

	MC_MSB = 0xBC,
	MC_LSB = 0xBD,

	MD_MSB = 0xBE,
	MD_LSB = 0xBF,

	UT_WRITEREG = 0xF4,
	UT_READMSBREG =0XF6,
	UT_READLSBREG =0XF7,


}BMP180_Addres;




typedef enum{

    ultralowpower =0x0,
	standard      =0x1,
	highresolution=0x2,
	ultrahighresolution=0x3,
}BMP180_OverSampling;



void BMP180_Init();
void BMP180_Get_Calibration_Value();
float BMP180_Get_Temperature_Value();
float BMP180_Get_Pressure_Value();
float BMP180_Get_Altitude_Value();
void BMP180_Calculate_Temperature_Value();
uint16_t BMP180_Get_Uncompansated_Value();
void  BMP180_Calculate_Pressure_Value();
long BMP180_Get_Uncompansated_Pressure_Value();

void BM180_RAW_Value(float *tempvalue,float *pressurevalue,float *altiudevalue);




#endif /* INC_BMP180_H_ */
