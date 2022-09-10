

/* Includes */

#include "stm32f10x.h"

int count=0;

void delay(long cycles)
{
  while(cycles >0)
  cycles--; // Some stupid delay, it is not in milliseconds or microseconds, but rather in some 'wasted clock cycles'
}

void gpio_ports_enable(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN|RCC_APB2ENR_IOPCEN; //ports A & B & C clock enabled
  //GPIOC->CRH &=0XFF0FFFFF;
/* GPIO PIN OUT C13 CONFIG*/
GPIOC->CRH |= 1<<20;               //Pin C13 enable.    Output mode, max speed 50 MHz
GPIOC->CRH |= 1<<21;               //Pin C13 enable.    Output mode, max speed 50 MHz
GPIOC->CRH &= ~(1<<22);            // Pin C13 e General purpose output push-pull
GPIOC->CRH &= ~(1<<23);            // Pin C13 e General purpose output push-pull

/* GPIO PIN INPUT C15 CONFIG*/
GPIOC->CRH &= ~(1<<28);            // Pin C15 Input mode
GPIOC->CRH &= ~(1<<29);            //Pin C15 Input mode
GPIOC->CRH &= ~(1<<30);            // Input with pull-up / pull-down
GPIOC->CRH |= 1<<31;               // Input with pull-up / pull-down

}

int main(void)
{


gpio_ports_enable();
  while (1)
  {


    if(GPIOC->IDR & (1<<15))    // IDR GPIOC nin 15.register true
    {
    	while(GPIOC->IDR & (1<<15));
    	delay(20000);
    	count ++;
    	GPIOC->BSRR = GPIO_BSSR_BR13;
    }
    else
    {
    	GPIOC->BSRR = GPIO_BSRR_BS13;
    }




  }
}



