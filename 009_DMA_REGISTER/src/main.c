
/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
uint32_t adc;
uint32_t adc_value[2];
void RCC_Config()
{
	RCC->CR |= 0x00010000;	             // HSEON  Enable
	while(!(RCC->CR & 0x00020000));	     // HSEON Ready Flag wait
	RCC->CR |= 0x00080000;	             // CSS Enable
	RCC->CR |= 0X01000000;               // PLL On
	RCC->PLLCFGR |= 0x00400000;	         // PLL e HSE seçtik
	RCC->PLLCFGR |= 0x00000004;	         // PLL M = 4
	RCC->PLLCFGR |= 0x00005A00;	         // Pll N = 168
	RCC->PLLCFGR |= 0x00000000;	         // PLL p = 2
	RCC->CFGR |= 0x00000000;	         // AHB Prescaler = 1
	RCC->CFGR |= 0x00080000;	         // APB2 Prescaler = 2
	RCC->CFGR |= 0x00001400;	         // APB1 Prescaler = 4
	RCC->CIR |= 0x00800000;		         // CSS Flag clear
}

void GPIO_Config()
{
	RCC->AHB1ENR |= 0x00000001;	// GPIOA Clock Enable

	GPIOA->MODER |= 0x00000003;	// Pin 0 Analog
	GPIOA->OSPEEDR |= 0x00000003;	// Pin 0 100MHz;
}

void ADC_Config()
{
	RCC->APB2ENR |= 0x00000100;	// ADC1 Clock enable

    ADC1->CR1 |= 1<<8;           // ADC Scan Mode Enable
    ADC1->CR1 |= 0<<24;          // 12 Bit Adc Resulation

    ADC1->CR2 |= 1<<0;           // ADC Enable
    ADC1->CR2 |= 1<<1;           // Continuous Conversion Mode
    ADC1->CR2 |= 1<<8;           // DMA Mode Enable
    ADC1->CR2 |= 1<<9;           // DDS Enable
    ADC1->CR2 |= 1<<10;           // EOC Mode Enable
    ADC1->CR2 |= 1<<30;           // Start Conversion Enable

    ADC1->SQR1 |= 1<<20;         // 1 Conversion Selected
    ADC1->SQR3 |= 0<<0;



}

void DMA_Config()
{
	RCC->AHB1ENR  |= 1<<22;    // RCC AHB1 DMA Clock Enable


	while((DMA2_Stream4->CR & 0x00000001)==1); // wait until 0
	DMA2_Stream4->PAR  |= (uint32_t) & ADC1->DR; // address of read adc	adc_value[2];
	DMA2_Stream4->M0AR |=  (uint32_t) & adc_value[2];

	DMA2_Stream4->CR   |= 0<<6;   // Peripheral to Memory
	DMA2_Stream4->CR   |= 1<<8;   // Circular Mode Enable
	DMA2_Stream4->CR   |= 0<<9;   // Peripheral Adress Fixed
	DMA2_Stream4->CR   |= 1<<10;   // Memory Adress incremented
	DMA2_Stream4->CR   |= 2<<11;   // Peripheral data size Word (32-bit)
	DMA2_Stream4->CR   |= 2<<12;   // Memory data size (32-bit)
	DMA2_Stream4->CR   |= 3<<16;   // Priority level High Level
	DMA2_Stream4->CR   |= 0<<25;   // Channel 0 Selected
	DMA2_Stream4->NDTR |= 1 ;     // Single Adc Read for One Write

	DMA2_Stream4->FCR  |= 1<<0;
	DMA2_Stream4->CR   |= 1<<0;   // 	DMA2_Stream4 Enable
}

uint8_t Read_ADC()
{
	uint8_t value;

	ADC1->CR2 |= 0x40000000;

	while(!(ADC1->SR & 0x00000002));

	value = ADC1->DR;

	return value;
}

int main(void)
{
  int i = 0;

  RCC_Config();
  GPIO_Config();
  Read_ADC();
  DMA_Config();


  ADC1->CR2  |= ADC_CR2_SWSTART;

  /* Infinite loop */
  while (1)
  {
	  adc=adc_value[0];
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
