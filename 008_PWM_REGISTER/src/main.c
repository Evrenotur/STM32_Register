

/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

uint16_t count =0;
void delay(uint32_t time)
{
	while(time--);
}

void RCC_Config()
{
	RCC->CR |= 0x00030000;				    // HSEON and HSEONRDY enable
		while(!(RCC->CR & 0x00020000));		// HSEON Ready Flag wait
		RCC->CR |= 0x00080000;				// CSS Enable
		RCC->CR |= 0x01000000;				// PLL ON
		RCC->PLLCFGR |= 0x00400000;			// PLL e HSE seçtik
		RCC->PLLCFGR |= 0x00000004;			// PLL M = 4
		RCC->PLLCFGR |= 0x00005A00;			// Pll N = 168
		RCC->PLLCFGR |= 0x00000000;			// PLL p = 2
		RCC->CFGR |= 0x00000000;			// AHB Prescaler = 1
		RCC->CFGR |= 0x00080000;			// APB2 Prescaler = 2
		RCC->CFGR |= 0x00001400;			// APB1 Prescaler = 4
		RCC->CIR |= 0x00080000;				// HSERDY Flag clear
		RCC->CIR |= 0x00800000;				// CSS Flag clear

}
void Timer_Config()
{

	   RCC->APB1ENR |= 1<<2;                          // TIM 4 Clock Enable

	   TIM4->CR1    |= 0<<4;                          // TIM 4 Up Counter
	   TIM4->CR1    |= 0<<5;                          // TIM 4 Center Aligned Edge
	   TIM4->CR1    |= 0<<8;                          // TIM 4 Prescaler Clock Div 1
	   /*
	    * Capture/Compare 1 & 2 output selected
	    * Capture/Compare 1 & 2 Pwm1 selected
	    */
	   TIM4->CCMR1  |= 0<<0 | 6<<4 | 0<<8 | 6<<12;


	   /*
	    * Capture/Compare 3 & 4 output selected
	    * Capture/Compare 3 & 4 Pwm1 selected
	    */

	   TIM4->CCMR2  |=  0<<0 | 6<<4 | 0<<8 | 6<<12;

	   /*
	    * Capture/Compare 1 & 2 & 3 & 4 Output Enable
	    *
	    *
	    */
	   TIM4->CCMR2  |=  1<<0 | 1 << 4 | 1 <<8 | 1 << 12;

	   TIM4->PSC   = 83;
	   TIM4->ARR   = 93;

	   TIM4->CCR1 = 24;
	   TIM4->CCR2 = 44;
	   TIM4->CCR3 = 79;
	   TIM4->CCR3 = 99;

	   TIM4->CR1  |= 1<<0;
}
void GPIO_Config()
{

	RCC->AHB1ENR   |= 0X00000008;                         // AHB1 Clock Enable

	GPIOD->MODER   |=0XAA000000;                          // GPIO D 12,13,14,15 Alternate Function

	GPIOD->AFR[1]  |=  2<<16 | 2<<20 |  2<<24 | 2<<28 ;   // GPIO D Pin 12-13-14-15 TIM4







}

int main(void)
{

	RCC_Config();
    GPIO_Config();
	Timer_Config();
  int i = 0;


  /* Infinite loop */
  while (1)
  {
	i++;
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
