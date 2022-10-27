

#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"


static void Led_config();
static void lockcontrol();

int main(void)
{

    Led_config();
   GPIO_LockPin(GPIOA, GPIO_PIN_0);

   lockcontrol();
   for(;;)
   {
	   if(GPIO_Read_Pin(GPIOA, GPIO_PIN_0)==GPIO_Pin_Set){
	      GPIO_Write_pin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13  | GPIO_PIN_14 , GPIO_Pin_Set);
	      }
	      else
	      {
	      	GPIO_Write_pin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13  | GPIO_PIN_14 , GPIO_Pin_Reset);
	      }
   }
}

static void Led_config()
{

	GPIO_InitType_t GPIO_Init_Struct = {0} ;

	RCC_GPIOD_CLK_ENABLE();
    RCC_GPIOA_CLK_ENABLE();
	GPIO_Init_Struct.pinNumber = GPIO_PIN_12 | GPIO_PIN_13  | GPIO_PIN_14 ;
	GPIO_Init_Struct.Mode      = GPIO_MODE_OUTPUT;
	GPIO_Init_Struct.Speed     = GPIO_SPEED_HIGH;
	GPIO_Init_Struct.Otype     = GPIO_OTYPE_PP;

	GPIO_Init(GPIOD, &GPIO_Init_Struct);

	memset(&GPIO_Init_Struct,0,sizeof(GPIO_Init_Struct));

	GPIO_Init_Struct.pinNumber = GPIO_PIN_0;
	GPIO_Init_Struct.Mode      = GPIO_MODE_INPUT;
	GPIO_Init_Struct.Speed     = GPIO_SPEED_HIGH;


	GPIO_Init(GPIOA, &GPIO_Init_Struct);

}
static void lockcontrol()
{
	GPIO_InitType_t GPIO_Init_Struct = {0} ;

	GPIO_Init_Struct.pinNumber = GPIO_PIN_0;
	GPIO_Init_Struct.Mode      = GPIO_MODE_OUTPUT;
	GPIO_Init_Struct.Speed     = GPIO_SPEED_HIGH;


    GPIO_Init(GPIOA, &GPIO_Init_Struct);



}
