/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"



char RX_Buffer[100];
int i=0;

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
	//RCC->AHB1ENR |= 0x00000001;	       // GPIOA Clock Enable
	RCC->AHB1ENR   |= 1<<1;                // GPIOB Clock Enable
    GPIOB->MODER   |= (2<<20) | (2<<22);   // GPIOB 10-11 Alternate Function
    GPIOB->AFR[1]  |= (7<<8) |  (7<<12);   // AF PB10 And PB11 AF7 (Usart3)
}

void Usart_Config()
{
	RCC->APB1ENR   |= 1<<18;                // RCC APB1 Clock Enable

	USART3->BRR    |= 0x1112;              // Baute Rate 9600
	USART3->CR1    |= 1<<2;                // Recive Enable
	USART3->CR1    |= 1<<3;                // Transmit Enable
	USART3->CR1    |= 1<<5;                // RXNE interrupt Enable
	USART3->CR1    |= 1<<10;               // No Parity
	USART3->CR1    |= 0<<12;               // Word Size Start bit, 8 Data bits, n Stop bit
	USART3->CR2    |= 0<<12;               // Stop Bit 1



	USART3->CR1    |= 0<<13;               // Usart Enable

}


void NVIC_Config()
{

	NVIC->ISER[1] |= 1<<7;

}

void USART3_IRQHandler()
{

	volatile int Str;

	Str = USART3->SR;
	RX_Buffer[i] = USART3->DR;
	i++;
}
void Send_Char(char  message)
{

	while(!(USART3->SR |=0x00000080)); // TXE Buffer Dolu İse Bekle

	 USART3->DR  = message;

}

void Send_Message(char *str)
{

    while(*str)
    {

    Send_Char(*str);

    str++;

    }
}
int main(void)
{
	RCC_Config();
	GPIO_Config();
	Usart_Config();
	NVIC_Config();



  /* Infinite loop */
  while (1)
  {
	  Send_Message("Merhaba Evren\n");
	  for(int i=0;i<1000000;i++);
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
