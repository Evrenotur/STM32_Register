/*
 * GPIO.c
 *
 *  Created on: 5 Ekim 2021
 *      Author: SOFTWARE
 */
#include "GPIO.h"


/*
 * @brief GPIO_Init Configures the port and pin
 * @param GPIOx = GPIO port base Address
 *
 * @param GPIO_InitType_t = Users Config Structure
 *
 *
 * @retval Void
 */
void GPIO_Init(GPIO_TypeDef_t *GPIOx,GPIO_InitType_t*GPIO_ConfigStruct)
{

	uint32_t Position;
	uint32_t fakePosition = 0;
	uint32_t lastPosition = 0;

	for(Position=0;Position <16; Position ++)
	{

		fakePosition = (0X1 <<Position);
		lastPosition = (uint32_t)(GPIO_ConfigStruct->pinNumber) & fakePosition;

		if(fakePosition == lastPosition)
		{
			/* MODE CONFİG */
			uint32_t tempValue = GPIOx->MODER ;
			tempValue &= ~(0x3U << (Position *2 ));
			tempValue |= (GPIO_ConfigStruct->Mode << (Position *2));

			GPIOx->MODER = tempValue;

			if(GPIO_ConfigStruct ->Mode == GPIO_MODE_OUTPUT || GPIO_MODE_ANALOG)
			{
				/* OUTPUT OTYPE CONFİG */

				tempValue = GPIOx->OTYPER;
				tempValue  &= ~(0x1U<< Position);
				tempValue  |= (GPIO_ConfigStruct->Otype <<Position);
				GPIOx->OTYPER = tempValue;


				/* OUTPUT Speed CONFİG */

				tempValue = GPIOx->OSPEEDR;
				tempValue  &= ~(0x3U<< (Position * 2));
				tempValue  |= (GPIO_ConfigStruct->Otype <<Position);
				GPIOx->OSPEEDR = tempValue;

			}


			    /*PUSH PULL CONFİG  */
			tempValue = GPIOx->PUPDR;
			tempValue  &= ~(0x3U<< (Position * 2));
			tempValue  |= (GPIO_ConfigStruct->PuPd << (Position *2 ));
			GPIOx->PUPDR = tempValue;

		}

	}
}
/*
 * @brief GPIO_Write_pin , makes pin High or Low
 * @param GPIOx = GPIO port base Address
 *
 * @param pinNumber = GPIO Pin Numbers 0 - 15
 *
 * @param pinStade  GPIO_Pin_Set Or GPIO_Pin_Reset
 *
 * @retval Void
 */
void GPIO_Write_pin(GPIO_TypeDef_t*GPIOx,uint16_t pinNumber,GPIO_PinStade_t pinstade)
{




	if(pinstade == GPIO_Pin_Set)
	{
		GPIOx->BSRR = pinNumber;
	}
	else
	{
		GPIOx->BSRR = (pinNumber << 16U);
	}
}

/*
 * @brief GPIO_Read_pin , Reads the pin of GPIOx port
 * @param GPIOx = GPIO port base Address
 *
 * @param pinNumber = GPIO Pin Numbers 0 - 15
 *
 * @param pinStade  GPIO_Pin_Set Or GPIO_Pin_Reset
 *
 * @retval GPIO_PinStade_t
*/
GPIO_PinStade_t GPIO_Read_Pin(GPIO_TypeDef_t * GPIOx,uint16_t pinNumber)
{

	GPIO_PinStade_t bit_status = GPIO_Pin_Reset;

	if((GPIOx-> IDR & pinNumber) != GPIO_Pin_Reset)
	{

           bit_status = GPIO_Pin_Set;


	}

      return bit_status;

}

/*
 * @brief GPIO_Lock_pin , Locks the pin of GPIOx port
 * @param GPIOx = GPIO port base Address
 *
 * @param pinNumber = GPIO Pin Numbers 0 - 15
 *
 * @param pinStade  GPIO_Pin_Set Or GPIO_Pin_Reset
 *
 * @retval voidf
*/
void GPIO_LockPin(GPIO_TypeDef_t * GPIOx,uint16_t PinNumber)
{

	uint32_t tempValue = (0X1U << 16U)|PinNumber;   // 1 0000 0000 0000 0000

	GPIOx->LCKR = tempValue;                  // LCKR[16] = '1' LCKR[15:0] = DATA
	GPIOx->LCKR = PinNumber;                  // LCKR[16] = '0' LCKR[15:0] = DATA
	GPIOx->LCKR = tempValue;                  // LCKR[16] = '1' LCKR[15:0] = DATA
	tempValue   = GPIOx->LCKR;                //  Read Lock Register

}
/*
 * @brief GPIO_Toggle_pin , Toggles the pin of GPIOx port
 * @param GPIOx = GPIO port base Address
 *
 * @param pinNumber = GPIO Pin Numbers 0 - 15
 *
 * @param pinStade  GPIO_Pin_Set Or GPIO_Pin_Reset
 *
 * @retval void
*/










void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber)
{

uint32_t tempODRReigster = GPIOx ->ODR;
GPIOx->BSRR =( ((tempODRReigster & pinNumber) << 16U) |(~tempODRReigster & pinNumber)); // 0000 0000 0000 0010 ODR

}







