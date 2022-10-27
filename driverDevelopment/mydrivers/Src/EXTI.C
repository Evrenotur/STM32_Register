/*
 * EXTI.C
 *
 *  Created on: 20 Kas 2021
 *      Author: SOFTWARE
 */


/*
 * @brief EXTI_Init for valid GPIO port and Line Number
 *
 * @param EXTI_InitStruct = User Config Structure
 *
 * @retval Void
 */


#include "EXTI.h"
#include "stm32f407xx.h"

void EXTI_Init(EXTI_InitTypeDef_t * EXTI_InitStruct)
{
  uint32_t tempValue =0;

  tempValue = (uint32_t)EXTI_BASE_ADDR;

  EXTI->IMR &= ~(0X1U << EXTI_InitStruct ->EXTI_LineNumber0);
  EXTI->EMR &= ~(0X1U << EXTI_InitStruct ->EXTI_LineNumber0);

  if(EXTI_InitStruct->EXTI_LineCmd !=DISABLE)
  {
	  tempValue +=EXTI_InitStruct->EXTI_Mode;

	  *((_IO uint32_t*)tempValue) = (0x1U << EXTI_InitStruct->EXTI_LineNumber);

	  tempValue = (uint32_t)EXTI_BASE_ADDR;

	  EXTI->RTSR &=~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
	  EXTI->FTSR &=~(0x1U << EXTI_InitStruct->EXTI_LineNumber);

	  if(EXTI_InitStruct->TriggerSelection==EXTI_Trigger_RF)
	  {
		  EXTI->RTSR |=(0x1U << EXTI_InitStruct->EXTI_LineNumber);
		  EXTI->FTSR |=(0x1U << EXTI_InitStruct->EXTI_LineNumber);

	  }
	  else
	  {
		  tempValue += EXTI_InitStruct ->TriggerSelection;
		  *((_IO uint32_t*)tempValue) != (0X1U<< EXTI_InitStruct->EXTI_LineNumber);
	  }

  }
  else
  {

	  tempValue = (uint32_t)EXTI_BASE_ADDR;

	  tempValue +=EXTI_InitStruct->EXTI_Mode;

	  *((_IO uint32_t*)tempValue) &= ~(0X1U<< EXTI_InitStruct->EXTI_LineNumber);


  }

}





/*
 * @brief GPIO_LineConfig Configures the port and pin for SYSCFG
 * @param PortSource = Port Value A-I @def_group  port_values
 *
 * @param EXTI_line_source = pin_numbers & Line_Numbers   @param EXTI_Line_values
 *
 *
 * @retval Void;
 */

void EXTI_LineConfig(uint8_t port_source,uint8_t EXTI_line_source)
{

   uint32_t tempvalue;
   tempvalue = SYSCFG ->EXTI_CR[EXTI_line_source>>2U]; // Uygun Registerı bulduk
   tempvalue = ~(0xFU<<(EXTI_line_source & 0X3U)*4);
   tempvalue = (port_source<<(EXTI_line_source & 0X3U)*4);
   SYSCFG ->EXTI_CR[EXTI_line_source>>2U]=tempvalue;

}
/*
 * @brief NVIC_EnableInterrupt
 *
 * @param PortSource = IRQ Number of Line
 *
 * @retval Void;
 */

void NVIC_EnableInterrupt(IRQNumber_TypeDef_t IRONUMBER)
{
	uint32_t tempValue = 0;

	tempValue = *((IRONUMBER>>5U) + NVIC_ISER0);
    tempValue &=~(0x1U<<(IRONUMBER &0X1FU));
    tempValue |= (0x1U << (IRONUMBER &0X1FU));
    *((IRONUMBER>>5U) + NVIC_ISER0) = tempValue;
}
