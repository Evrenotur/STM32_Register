

/* Includes */
#include <stddef.h>
#include "stm32f10x.h"
int flag;
int flag_1;
void GPIO_Config()
{
	RCC->APB2ENR = 0x0000007D;          // GPIO A-B-C-D & AFI0 Clock Bus Enable
	/* GPIO PIN OUT C13 CONFIG*/
	GPIOC->CRH |= 1<<20;               //Pin C13 enable.    Output mode, max speed 50 MHz
	GPIOC->CRH |= 1<<21;               //Pin C13 enable.    Output mode, max speed 50 MHz
	GPIOC->CRH &= ~(1<<22);            // Pin C13 e General purpose output push-pull
	GPIOC->CRH &= ~(1<<23);            // Pin C13 e General purpose output push-pull
}

void EXTI_Config()
{
	RCC->APB2ENR = 0x0000007D;          // GPIO A-B-C-D & AFI0 Clock Bus Enable
	AFIO->EXTICR[0] |= 0x00000000;         //configure EXTI1 line for PA1 and PA0

	NVIC_EnableIRQ (EXTI0_IRQn);        // Enable Interrupt
	NVIC_EnableIRQ (EXTI1_IRQn);        // Enable Interrupt
	NVIC_SetPriority (EXTI0_IRQn, 0);   // Set Priority 0 (Very Priority)
	NVIC_SetPriority (EXTI1_IRQn, 1);   // Set Priority

	/*Interrupt mode için aktif hale getirildi*/
	EXTI->IMR |= 1<<0;                  //Disable the Mask on EXTI 0
	EXTI->IMR |= 1<<1;                  //Disable the Mask on EXTI 1

	EXTI->RTSR |= (1<<0);               // Enable Rising Edge Trigger for PA0
	EXTI->RTSR |= (1<<1);               // Enable Rising Edge Trigger for PA1


}



/*
void  Interrupt_Config()
{
RCC->APB2ENR = 0x0000007D;          // GPIO A-B-C-D & AFI0 Clock Bus Enable
AFIO->EXTICR[0]  &= ~(0XF<<4);      // Bits[7:6:5:4] = (0:0:0:0)  -> configure EXTI1 line for PA1
EXTI->IMR |= 1<<1;                  // Bit[1] = 1  --> Disable the Mask on EXTI 1
EXTI->RTSR |= (1<<1);               // Enable Rising Edge Trigger for PA1
EXTI->FTSR &= ~(1<<1);              // Disable Falling Edge Trigger for PA1
NVIC_SetPriority (EXTI1_IRQn, 1);   // Set Priority


}
*/
void EXTI0_IRQHandler()
{
	if (EXTI->PR & (1<<0))    // If the PA1 triggered the interrupt
	{
		  GPIOC->BSRR = GPIO_BSRR_BR13;
	    flag = 1;
		EXTI->PR |= (1<<0);  // Clear the interrupt flag by writing a 1
	}

}
void EXTI1_IRQHandler()
{

	if (EXTI->PR & (1<<1))    // If the PA1 triggered the interrupt
	{
		  GPIOC->BSRR = GPIO_BSRR_BS13;
	        flag_1 = 1;
		EXTI->PR |= (1<<1);  // Clear the interrupt flag by writing a 1
	}
}

int main(void)
{
	GPIO_Config();
	EXTI_Config();

  /* TODO - Add your application code here */

  /* Infinite loop */
  while (1)
  {




  }
}
