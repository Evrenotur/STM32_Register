/*
 * modbus.c
 *
 *  Created on: Sep 12, 2023
 *      Author: hasan
 */



#include "modbus.h"

CallBackFunc cbVector[CB_MAX];
ReturnFunc rVector[RETURN_MAX];

uint8_t bufOut[MAX_BUFFER];
uint8_t bufIn[MAX_BUFFER];
uint16_t length;

uint16_t address;


const uint32_t baudrateArray[BAUDRATE_INDEX] = {9600, 1200, 2400, 4800, 5800, 9600, 14400, 19200, 28800, 33600, 38400, 57600, 115200, 230400, 256700};// 4.index default 9600
// Asagidakiler hal uart header dosyasinda tanimli degerlerdir.
const uint32_t paritArray[PARITY_INDEX] = {UART_PARITY_NONE, UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD};
const uint32_t stopbitArray[STOPBIT_INDEX] = {UART_STOPBITS_1, UART_STOPBITS_1, UART_STOPBITS_2};
const uint32_t byteSizeArray[BYTESIZE_INDEX] = {UART_WORDLENGTH_8B, UART_WORDLENGTH_8B, UART_WORDLENGTH_9B}; // 7 yi de destekler

extern rs485info_ rs485info;// = { 1, 5, 1, 1, 1, 10, 43200, 60, 60, 60, 60, 60, 60, 60, 60 };

void wordToByte(uint16_t wordData, uint8_t *byteHigh, uint8_t *byteLow)
{
    byteHigh = (wordData >> 8) & 0xFF;
    byteLow  = wordData & 0xFF;
}


uint16_t byteToWord( uint8_t byteHigh, uint8_t byteLow)
{
	return (uint16_t)(byteHigh << 8 | byteLow) ;
}

uint16_t calcCRC(uint8_t *buf, int length) {
    int i, j;
    uint16_t crc = 0xFFFF;
    uint16_t tmp;

    // calculate crc16
    for (i = 0; i < length; i++) {
        crc = crc ^ buf[i];

        for (j = 0; j < 8; j++) {
            tmp = crc & 0x0001;
            crc = crc >> 1;
            if (tmp) {
              crc = crc ^ 0xA001;
            }
        }
    }

    return crc;
}

uint16_t exception(uint8_t *bufOut,  uint8_t *bufIn, uint8_t error)
{
	error = STATUS_ILLEGAL_FUNCTION;

	//lengthOut = 5;
	bufOut[0] = rs485info.slaveId;
	bufOut[1] = bufIn[1] | 0x80;

	bufOut[2] = error;

	if(bufIn[0] == BROADCAST_SLAVE_ID)
		return 0;
	else
		return 5;
}

void modbusAnalyser(uint8_t *bufIn_, uint16_t lengthIn)
{
	uint16_t i, cevapCount = 0;
	memset(bufIn, 0x00, MAX_BUFFER);
	memcpy(bufIn , bufIn_, lengthIn);
	uint8_t slaveAddress = bufIn[0];
	uint8_t fc = bufIn[1];


    int lengthOut;
    uint16_t crc;

    uint16_t available_len;
    uint8_t cb_status;
	uint8_t error = STATUS_OK;

	memset(bufOut, 0x00, MAX_BUFFER);
	if((rs485info.slaveId == slaveAddress) || (BROADCAST_SLAVE_ID == slaveAddress))
	{
		//data bize gelmis cevap verelim
		// fonksiyon kodunu tespit edelim
		//UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size
		address = byteToWord(bufIn[2], bufIn[3]); // first register.
		length  = byteToWord(bufIn[4], bufIn[5]);  // number of registers to act upone or status.

		 // check crc.
		crc = byteToWord(bufIn[lengthIn - 1], bufIn[lengthIn - 2]); // 0xCACD
		if (calcCRC(bufIn, lengthIn - 2) != crc) {
			// standard modbus doesn't respond in case of
			// crc error
			// TODO : arrayleri clear et, cevap vermeye gerek yok
			return; // modbus standararina  uymayan data
		}


		/**
		 * Output length sanity check, and remove trailing noise from message.
		 */
		switch (fc)
		{
			case FC_READ_COILS: // read coils (digital out state)
			case FC_READ_DISCRETE_INPUT: // read input state (digital in)
			case FC_READ_HOLDING_REGISTERS: // read holding registers (analog out state)
			case FC_READ_INPUT_REGISTERS: // read input registers (analog in)
				// sanity check.
				if (length > MAX_BUFFER) {
					error = STATUS_ILLEGAL_DATA_ADDRESS;
					// as long as I am not using gotos at all
					// in case of protocol handling they are usefull for
					// cleaning up when it comes to cleaning up
					// when something goes wrong while processing
					// instead og goto same can be implemented as nested
					// if statements
				   // goto respond;
				}

				// ignore tailing nulls.
				lengthIn = 8;

				break;
			case FC_WRITE_COIL:
				// ignore tailing nulls.
				lengthIn = 8;

				break;
			case FC_WRITE_REGISTER:
				// ignore tailing nulls.
				lengthIn = 8;

				break;
			case FC_WRITE_MULTIPLE_COILS:
				// sanity check.
				if (length > MAX_BUFFER) {
					error = STATUS_ILLEGAL_DATA_ADDRESS;
					// as long as I am not using gotos at all
					// in case of protocol handling they are usefull for
					// cleaning up when it comes to cleaning up
					// when something goes wrong while processing
					// instead og goto same can be implemented as nested
					// if statements
				  //  goto respond;
				}

				// check buffer in size.
				if (lengthIn < (int)(7 + (length + 7) / 8 + 2)) return 0;

				// ignore tailing nulls.
				lengthIn = (int)(7 + (length + 7) / 8 + 2);

				break;
			case FC_WRITE_MULTIPLE_REGISTERS:
				// sanity check.
				if (length > MAX_BUFFER) {
					error = STATUS_ILLEGAL_DATA_ADDRESS;
				   // goto respond;
				}

				// check buffer in size.
				if (lengthIn < (int)(7 + length * 2 + 2)) return 0;

				// ignore tailing nulls.
				lengthIn = (int)(7 + length * 2 + 2);

				break;
			default:
				// unknown command
				// TODO respond with exeption 01 (illegal function)
				error = STATUS_ILLEGAL_FUNCTION;

				lengthOut = exception(bufOut,  bufIn, error);

				goto respond;
		}

		switch (fc)
		{
			case FC_READ_COILS: // read coils (digital out state)
			{
				// build valid empty answer.
				lengthOut = 3 + (length - 1) / 8 + 1 + 2;
				bufOut[2] = (length - 1) / 8 + 1;

				// clear data out.
				memset(bufOut + 3, 0, bufOut[2]);

				// if we have uset callback.
				if (cbVector[CB_READ_COILS]) {
					cb_status = cbVector[CB_READ_COILS](fc, address, length);
				} else {
					cb_status = STATUS_ILLEGAL_FUNCTION;
				}
				break;
			}
			case FC_READ_DISCRETE_INPUT: // read input state (digital in)
			{	// build valid empty answer.
				lengthOut = 3 + (length - 1) / 8 + 1 + 2;
				bufOut[2] = (length - 1) / 8 + 1;

				// clear data out.
				 memset(bufOut + 3, 0, bufOut[2]);

				// if we have uset callback.
				if (cbVector[CB_READ_DISCRETE_INPUTS]) {
					cb_status = cbVector[CB_READ_DISCRETE_INPUTS](fc, address, length);
				} else {
					cb_status = STATUS_ILLEGAL_FUNCTION;
				}
				break;
			}
			case FC_READ_HOLDING_REGISTERS: // read holding registers (analog out state)
			{
				// build valid empty answer.
				lengthOut = 3 + 2 * length + 2;
				bufOut[2] = 2 * length;

				// clear data out.
				memset(bufOut + 3, 0, bufOut[2]);

				// if we have uset callback.
				if (cbVector[CB_READ_HOLDING_REGISTERS]) {
					cb_status = cbVector[CB_READ_HOLDING_REGISTERS](fc, address, length);
				} else {
					cb_status = STATUS_ILLEGAL_FUNCTION;
				}
				break;
			}
			case FC_READ_INPUT_REGISTERS: // read input registers (analog in)
			{	// build valid empty answer.
				lengthOut = 3 + 2 * length + 2;
				bufOut[2] = 2 * length;

				// clear data out.
				memset(bufOut + 3, 0, bufOut[2]);

				// if we have uset callback.
				if (cbVector[CB_READ_INPUT_REGISTERS]) {
					cb_status = cbVector[CB_READ_INPUT_REGISTERS](fc, address, length);
				} else {
					cb_status = STATUS_ILLEGAL_FUNCTION;
				}
				break;
			}
			case FC_WRITE_COIL: // write one coil (digital out)
			{	// build valid empty answer.
				lengthOut = 8;
				memcpy(bufOut + 2, bufIn + 2, 4);

				// if we have uset callback.
				if (cbVector[CB_WRITE_COILS]) {
					cb_status = cbVector[CB_WRITE_COILS](fc, address, 1);
				} else {
					cb_status = STATUS_ILLEGAL_FUNCTION;
				}

				break;
			}
			case FC_WRITE_MULTIPLE_COILS: // write coils (digital out)
			{	// build valid empty answer.
				lengthOut = 8;
				memcpy(bufOut + 2, bufIn + 2, 4);

				// if we have uset callback.
				if (cbVector[CB_WRITE_COILS]) {
					cb_status = cbVector[CB_WRITE_COILS](fc, address, length);
				} else {
					cb_status = STATUS_ILLEGAL_FUNCTION;
				}

				break;
			}
			case FC_WRITE_REGISTER:
			{	// build valid empty answer.
				lengthOut = 8;
				memcpy(bufOut + 2, bufIn + 2, 4);

				// if we have uset callback
				if (cbVector[CB_WRITE_REGISTERS]) {
					cb_status = cbVector[CB_WRITE_REGISTERS](fc, address, 1);
				} else {
					cb_status = STATUS_ILLEGAL_FUNCTION;
				}

				break;
			}
			case FC_WRITE_MULTIPLE_REGISTERS: // write holding registers (analog out)
			{	// build valid empty answer.
				lengthOut = 8;
				memcpy(bufOut + 2, bufIn + 2, 4);

				// if we have uset callback
				if (cbVector[CB_WRITE_REGISTERS]) {
					cb_status = cbVector[CB_WRITE_REGISTERS](fc, address, length);
				} else {
					cb_status = STATUS_ILLEGAL_FUNCTION;
				}

				break;
			}
		}

		/**
		 * Build answer
		 */
		bufOut[0] = slaveAddress;
		bufOut[1] = fc;

		respond:
		if(lengthOut == 0 ) return lengthOut;

		// add crc
		crc = calcCRC(bufOut, lengthOut - 2);
		bufOut[lengthOut - 2] = crc & 0xff;
		bufOut[lengthOut - 1] = crc >> 8;
		if (rVector[RETURN_FUNCTION]) {
			rVector[RETURN_FUNCTION](bufOut, lengthOut);
		}
	}



	return lengthOut;
}


uint8_t readCoilFromBuffer(int offset) {
    if (bufIn[1] == FC_WRITE_COIL) {
    	assert_param(offset == 0);
       return (byteToWord(bufIn[4], bufIn[5]) == COIL_ON ? HIGH:(byteToWord(bufIn[4], bufIn[5]) == COIL_OFF ? LOW:ERROR));
    }

    assert_param(bufIn[1] == FC_WRITE_MULTIPLE_COILS);

    int address = 7 + offset / 8;
    int bit = offset % 8;

    return (bitRead(bufIn[address], bit) == COIL_ONS ? HIGH:LOW);
}
uint16_t readRegisterFromBuffer(int offset) {
    if (bufIn[1] == FC_WRITE_REGISTER) {
    	assert_param(offset == 0);
       return byteToWord(bufIn[4], bufIn[5]);
    }

    assert_param(bufIn[1] == FC_WRITE_MULTIPLE_REGISTERS);

    int address = 7 + offset * 2;

    return byteToWord(bufIn[address], bufIn[address + 1]);
}

void writeCoilToBuffer(int offset, int state) {
    int address = 3 + offset / 8;
    int bit = offset % 8;

    if (state == HIGH) {
        bitSet(bufOut[address], bit);
    } else {
        bitClear(bufOut[address], bit);
    }
}

void writeRegisterToBuffer(int offset, uint16_t value) {
    int address = 3 + offset * 2;

    bufOut[address] = value >> 8;
    bufOut[address + 1] = value & 0xff;
}

uint8_t writeStringToBuffer(int offset, uint8_t *str, uint8_t length) {
    int address = 3 + offset * 2;

    // check string length.
    // MAX_BUFFER-2 because we ned two bytes for crc.
    if ((address + length) >= MAX_BUFFER-2) return STATUS_ILLEGAL_DATA_ADDRESS;

    memcpy(bufOut + address, str, length);
    return STATUS_OK;
}


