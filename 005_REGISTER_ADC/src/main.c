#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
  int adc_value;
void RCC_Config()
{

		RCC->CR |= 0x00030000;	// HSEON and HSEONRDY enable
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
	RCC->AHB1ENR    |=0X00000001;  // RCC GPIOA CLOK Enable
    GPIOA->MODER    |=0X00000003;  // GPIOA  0 Number Pin Analog Mode Active
    GPIOA->OSPEEDR  |=0X00000003;  // Very Hıgh Speed


}
void ADC_Config()
{

  RCC->APB2ENR  |= 0X000000100;   // APB2ENR ADC1 Clock Active

  ADC1->CR1     |= 0X020000000;   // Adc Read Resolution 8 Bit Active

  ADC1->CR2     |= 0X020000001;   // Adc Read  Active

  ADC1->SMPR2   |=0X0000000003;   // Adc Sample 56 cycles

  ADC->CCR      |=0X0001000000;   // Adc Clok Precaler Div 4


}
uint8_t Adc_Read()
{
	uint8_t value;

	ADC1->CR2   |=0X40000000;       // This bit is set by software to start conversion


	while(!(ADC1->SR & 0X00000002)); // Wait until the reading is finished


	value = ADC1->DR;                 // Reading

	return value;

}
int main(void)
{
  int i = 0;





  while (1)
  {
 adc_value = Adc_Read();
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
