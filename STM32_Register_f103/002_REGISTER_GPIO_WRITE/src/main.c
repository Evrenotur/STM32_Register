#include "stm32f10x.h"

void delay(long cycles)
{
  while(cycles >0)
  cycles--; // Some stupid delay, it is not in milliseconds or microseconds, but rather in some 'wasted clock cycles'
}

void gpio_ports_enable(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN|RCC_APB2ENR_IOPCEN; //ports A & B & C clock enabled
  //GPIOC->CRH &=0XFF0FFFFF;

  GPIOC->CRH |= 1<<20;       //Pin C13 enable.    Output mode, max speed 50 MHz
  GPIOC->CRH |= 1<<21;       //Pin C13 enable.  Output mode, max speed 50 MHz
  GPIOC->CRH &= ~(1<<22);    // Pin C13 e General purpose output push-pull
  GPIOC->CRH &= ~(1<<23);    // Pin C13 e General purpose output push-pull

}

int main(void)
{
    gpio_ports_enable();

    for(;;)  //main loop - read "forever", or you may use 'while(1)'
    {
          GPIOC->BSRR = GPIO_BSRR_BR13; // Pinin ters olma sebebi ile reset kullanılmıştır.


    }
}
