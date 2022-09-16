
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
int count=0;
void delay(uint32_t time)
{
	while(time--);


}
void CLK_Config()
{
	RCC->AHB1ENR = 0x00000009;  // RCC AHB1ENR GPIO A  ve D ENABLE
	RCC->CR |= 0x00010000;	// HSEON enable
	while(!(RCC->CR & 0x00020000));	// HSEON Ready Flag wait
	RCC->CR |= 0x00080000;	// CSS Enable
	RCC->PLLCFGR |= 0x00400000;	// PLL e HSE seçtik
	RCC->PLLCFGR |= 0x00000004;	// PLL M = 4
	RCC->PLLCFGR |= 0x00005A00;	// Pll N = 168
	RCC->PLLCFGR |= 0x00000000;	// PLL p = 2
	RCC->CFGR |= 0x00000000;	// AHB Prescaler = 1
	RCC->CFGR |= 0x00080000;	// APB2 Prescaler = 2
	RCC->CFGR |= 0x00001400;	// APB1 Prescaler = 4
	RCC->CIR |= 0x00800000;		// CSS Flag clear

}


void GPIO_Config()
{



	GPIOD->MODER = 0X55000000;  // 12,13,14,15 Pins  OUTPUT MODE

	GPIOD->OTYPER  = 0X00000000;   //12,13,14,15 Pins Push-Pull MODE

	GPIOD->OSPEEDR = 0XFF000000;   // 12,13,14,15 Very high speed MODE

	GPIOD->PUPDR   = 0X00000000;   // 12,13,14,15  No pull-up, pull-down

}


int main(void)
{
	 // SystemClock 168 MHz
	CLK_Config();

   GPIO_Config();
  /* Infinite loop */
  while (1)
  {

	  if(GPIOA->IDR & 0X00000001) // GPIOA 0. Pin0 == HIGH
	  	{
	           while(GPIOA->IDR & 0X00000001);  // GPIOA 0. Pin0 == HIGH
	           delay(1680000);


	  		count ++;
	  	}


	  	if(count%2==0)
	  	{
	  		GPIOD->ODR = 0X00000000;  // GPIOD 12,13,14,15  PIN LOW
	  	}
	  	else
	  	{
	  		GPIOD->ODR = 0X0000F000;  // GPIOD 12,13,14,15  PIN HIGH
	  	}


  }
}


/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
