/*
 * modbus.h
 *
 *  Created on: Sep 12, 2023
 *      Author: hasan
 */


#ifndef MODBUS_H_
#define MODBUS_H_

/*
#include "main.h"*/
#include "stdbool.h"
#include "stm32f1xx_hal.h"  // stdint ve assert_param icin

/*
#define bitSet(value, bit) ((value) |= (1 << (uint8_t) (bit)))
#define bitClear(value, bit) ((value) &= ~(1 << (uint8_t) bit)))
*/
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

// modbus haberlesmesinde eger register haberlesmesi yapilacak ise
// adres byte bilgisi cevap verilirken 1 byte olarak girileceği icin, ve her register 2 byte oldugu icin
// max 127 adres icin read islemi yapilabilir, ama word type olarak okunursa 64 register bilgisi okunabilir.
#define MAX_BUFFER 127

#define HIGH 1
#define LOW  0
#define ERROR  2

#define BROADCAST_SLAVE_ID 0x00
#define SLAVE_ID_MAX   255
#define BAUDRATE_INDEX 15
#define PARITY_INDEX   4
#define STOPBIT_INDEX  3
#define BYTESIZE_INDEX 3

extern const uint32_t baudrateArray[BAUDRATE_INDEX] ;
extern const uint32_t paritArray[PARITY_INDEX] ;
extern const uint32_t stopbitArray[STOPBIT_INDEX] ;
extern const uint32_t byteSizeArray[BYTESIZE_INDEX] ;
//extern rs485info_ rs485info;// = { 1, 5, 1, 1, 1, 10, 43200, 60, 60, 60, 60, 60, 60, 60, 60 };

typedef struct
{
	uint8_t slaveId        ;
	uint8_t baudrate       ;
	uint8_t parity         ;
	uint8_t stopbits       ;
	uint8_t bytesize       ;

	uint16_t minProtection ;
	uint16_t maxProtection ;

	uint16_t protectionRO1 ; // seconds

} rs485info_;


/**
 * Modbus function codes
 */
enum {
  FC_READ_COILS = 1,
  FC_READ_DISCRETE_INPUT = 2,
  FC_READ_HOLDING_REGISTERS = 3,
  FC_READ_INPUT_REGISTERS = 4,
  FC_WRITE_COIL = 5,
  FC_WRITE_REGISTER = 6,
  FC_WRITE_MULTIPLE_COILS = 15,
  FC_WRITE_MULTIPLE_REGISTERS = 16
};

enum {
  CB_MIN = 0,
  CB_READ_COILS = CB_MIN,
  CB_READ_DISCRETE_INPUTS,
  CB_READ_HOLDING_REGISTERS,
  CB_READ_INPUT_REGISTERS,
  CB_WRITE_COILS,
  CB_WRITE_REGISTERS,
  CB_MAX
};

enum {
  COIL_OFF = 0x0000,
  COIL_ON = 0xff00,
  COIL_ONS = 0x01
};

enum {
  STATUS_OK = 0,
  STATUS_ILLEGAL_FUNCTION,
  STATUS_ILLEGAL_DATA_ADDRESS,
  STATUS_ILLEGAL_DATA_VALUE,
  STATUS_SLAVE_DEVICE_FAILURE,
  STATUS_ACKNOWLEDGE,
  STATUS_SLAVE_DEVICE_BUSY,
  STATUS_NEGATIVE_ACKNOWLEDGE,
  STATUS_MEMORY_PARITY_ERROR,
  STATUS_GATEWAY_PATH_UNAVAILABLE,
  STATUS_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND
};

typedef enum{
	FROM_SERVER=1,
	FROM_CLIENT,
	FROM_RF,
	FROM_RS485,
	FROM_USB,
} readedDatas;
enum {
	RETURN_FUNCTION=0,
	RETURN_MAX,
};
typedef uint8_t (*CallBackFunc)(uint8_t, uint16_t, uint16_t);
typedef uint8_t (*ReturnFunc)(uint8_t *, uint16_t); // data arrayi, data uzunlugu, eger rf veya tcp ise soket numarasi, nereden oldugu

extern CallBackFunc cbVector[CB_MAX];
extern ReturnFunc rVector[RETURN_MAX];

void wordToByte(uint16_t wordData, uint8_t *byteHigh, uint8_t *byteLow);

uint16_t byteToWord( uint8_t byteHigh, uint8_t byteLow);

/**
 * Calculate buffer CRC16
 *
 * @param buf the data buffer.
 * @param length the length of the buffer without CRC.
 *
 * @return the calculated CRC16.
 */
uint16_t calcCRC(uint8_t *buf, int length);

 /**
  * @brief bu fonksiyon cihaza gelen datayi isleyip dogru bir sekilde isleme alamamizi saglayacak
  * @note her data gelmesinde bu fonksion cagrilacaktir
  * @param gelenData gelen datatnin kendisi
  * @param count gelen datatnin byte sayisi
  * @param isUartServerClient gelen datatnin kimden geldigi
  */
void modbusAnalyser(uint8_t *bufIn_, uint16_t lengthIn);

/**
 * Read coil state from input buffer.
 *
 * @param offset the coil offset from first coil in this buffer.
 * @return the coil state from buffer (true / false)
 */
//GPIO_PinState readCoilFromBuffer(int offset);
uint8_t readCoilFromBuffer(int offset);
/**
 * Read register value from input buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @return the register value from buffer.
 */
uint16_t readRegisterFromBuffer(int offset);

/**
 * Write coil state to output buffer.
 *
 * @param offset the coil offset from first coil in this buffer.
 * @param state the coil state to write into buffer (true / false)
 */
void writeCoilToBuffer(int offset, int state);

/**
 * Write register value to output buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @param value the register value to write into buffer.
 */
void writeRegisterToBuffer(int offset, uint16_t value);

/**
 * Write arbitrary string of uint8_t to output buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @param str the string to write into buffer.
 * @param length the string length.
 * @return STATUS_OK in case of success, STATUS_ILLEGAL_DATA_ADDRESS
 *      if data doesn't fit in buffer
 */
uint8_t writeStringToBuffer(int offset, uint8_t *str, uint8_t length);

#endif /* MODBUS_H_ */

