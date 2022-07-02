

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

void CLK_Config()
{
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
	RCC->AHB1ENR |= 1<<3;        // RCC AHB1ENR GPIO D  ENABLE

	GPIOD->MODER |= 1<<24;       //GPIO D nin 12 pini output oldu
	GPIOD->MODER &= ~(1<<25);

	GPIOD->MODER |= 1<<26;       //GPIO D nin 13 pini output oldu
	GPIOD->MODER &= ~(1<<27);


	GPIOD->MODER |= 1<<28;       //GPIO D nin 14 pini output oldu
	GPIOD->MODER &= ~(1<<29);

	GPIOD->MODER |= 1<<30;       //GPIO D nin 15 pini output oldu
	GPIOD->MODER &= ~(1<<31);

	GPIOD->OSPEEDR |= 1<<24;     //GPIO D nin 12 pini  Very high speed oldu
	GPIOD->OSPEEDR |= 1<<25;

	GPIOD->OSPEEDR |= 1<<26;     //GPIO D nin 13 pini  Very high speed oldu
    GPIOD->OSPEEDR |= 1<<27;

    GPIOD->OSPEEDR |= 1<<28;     //GPIO D nin 14 pini  Very high speed oldu
    GPIOD->OSPEEDR |= 1<<29;

    GPIOD->OSPEEDR |= 1<<30;     //GPIO D nin 15 pini  Very high speed oldu
    GPIOD->OSPEEDR |= 1<<31;

  //  GPIOD->OSPEEDR |=FF00 0000; Yukarıdaki pin speed ayarlaması ile aynı anlama gelmektedir.

}


int main(void)
{
	 // SystemClock 168 MHz
	CLK_Config();

   GPIO_Config();
  /* Infinite loop */
  while (1)
  {


   GPIOD->ODR |= 1<<12;   //GPIO D nin 12 pini  set oldu
   GPIOD->ODR |= 1<<13;   //GPIO D nin 13 pini  set oldu
   GPIOD->ODR |= 1<<14;   //GPIO D nin 14 pini  set oldu
   GPIOD->ODR |= 1<<15;   //GPIO D nin 15 pini  set oldu

   for(int i=0;i<1680000;i++);

   GPIOD->ODR &= ~(1<<12);   //GPIO D nin 12 pini  reset oldu
   GPIOD->ODR &= ~(1<<13);   //GPIO D nin 13 pini  reset oldu
   GPIOD->ODR &= ~(1<<14);   //GPIO D nin 14 pini  reset oldu
   GPIOD->ODR &= ~(1<<15);   //GPIO D nin 15 pini  reset oldu

   for(int i=0;i<1680000;i++);

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
