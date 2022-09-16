#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

extern uint32_t SystemCoreClock;

uint32_t Systemclock;

void RCC_Config()
{

   //RCC->CR  &= 0x00000083;
	RCC->CR &= ~(1<<0);  // HSION  stade of mode
	RCC ->CR &= 1<<16;   // HSEON  stade on mode
	while(!(RCC->CR &= 1<<17)); // HSE ON olana kadar bekle

	RCC->CR &=(1<<19); // Her hangi bir clock hatası olduğunda osilatörü kapatır

	 // 0-1-2-3-4 bitlerinin  hepsini 1 olduğu sayı 31 bunu terslersek 0 olur
	RCC->PLLCFGR |= ~(31<<0);

	// PLL M değeri 4
	RCC->PLLCFGR |= (4<<0);

	// PLL N değeri 168
	RCC->PLLCFGR |= (6<<0);

	// PLL P değerinin 2 olması için 0 yazılacak 16. ve 17.bitlere
	RCC->PLLCFGR &= ~(1<<16);
	RCC->PLLCFGR &= ~(1<<17);


}

int main(void)
{
	Systemclock = SystemCoreClock;

	RCC_DeInit(); // HSI Stade ON MOD- PLL Stade  OFF MOD

	SystemCoreClockUpdate(); //System Mod 16 000 000 Mhz


	while(1)
	{





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
