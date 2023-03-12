/*
 * Eprom.h
 *
 *  Created on: Mar 12, 2023
 *      Author: Software
 */

#ifndef INC_EPROM_H_
#define INC_EPROM_H_

#include <stm32f1xx_hal.h>

uint16_t Read_Flash(uint32_t  adr);
void Write_Flash( uint32_t address , uint16_t *data);

#endif /* INC_EPROM_H_ */
