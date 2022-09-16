
/* Includes */


	/************** STEPS TO FOLLOW *****************
	1. Enable ADC and GPIO clock
	2. Set the prescalar in the Clock configuration register (RCC_CFGR)
	3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)
	4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	5. Set the Sampling Time for the channels in ADC_SMPRx
	6. Set the Regular channel sequence length in ADC_SQR1
	7. Set the Respective GPIO PINs in the Analog Mode
	************************************************/





#include <stddef.h>
#include "stm32f10x.h"
extern uint32_t SystemCoreClock;

uint32_t  systemclock;
uint8_t adc_value;

void ADC_Initt()
{
	/************** STEPS TO FOLLOW *****************
	1. Enable ADC and GPIO clock
	2. Set the prescalar in the Clock configuration register (RCC_CFGR)
	3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)
	4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	5. Set the Sampling Time for the channels in ADC_SMPRx
	6. Set the Regular channel sequence length in ADC_SQR1
	7. Set the Respective GPIO PINs in the Analog Mode
	************************************************/

//1. Enable ADC and GPIO clock
	RCC->APB2ENR |= 1<<9;  // enable ADC1 clock
	RCC->APB2ENR |= (1<<2);  // enable GPIOA clock

//2. Set the prescalar in the Clock configuration register (RCC_CFGR)
	RCC->CFGR |= (2<<14);  // Prescaler 6, ADC Clock = 72/6 = 12 MHz

//3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)
	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	// Resolution is 12 bit in F103

//4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	ADC1->CR2 = (1<<1);     // enable continuous conversion mode
	// EOC after each conversion by default
	ADC1->CR2 |= (7<<17);  // External Event selection pointed to SWSTART bit
	ADC1->CR2 &= ~(1<<11);   // Data Alignment RIGHT

//5. Set the Sampling Time for the channels
	ADC1->SMPR2 &= ~((7<<3) | (7<<12));  // Sampling time of 1.5 cycles for channel 1 and channel 4

//6. Set the Regular channel sequence length in ADC_SQR1
	ADC1->SQR1 |= (2<<20);   // SQR1_L =2 for 3 conversions

//7. Set the Respective GPIO PINs in the Analog Mode
	GPIOA->CRL &= ~(0xf<<4);  // analog mode for PA 1
	GPIOA->CRL &= ~(0xf<<16); // analog mode for PA 4


	/**************************************************************************************************/


	// Sampling Freq for Temp Sensor
	ADC1->SMPR1 |= (7<<18);  // Sampling time (71.5 cycles) of 7 us for channel 16.. It should be <17.1 us

	// Set the TSVREFE Bit to wake the sensor
	ADC1->CR2 |= (1<<23);

	// Enable DMA for ADC
	ADC1->CR2 |= (1<<8);

//	// Enable Continuous Request
//	ADC1->CR2 |= (1<<9);

	// Channel Sequence
	ADC1->SQR3 |= (1<<0);  // SEQ1 for Channel 1
	ADC1->SQR3 |= (4<<5);  // SEQ2 for CHannel 4
	ADC1->SQR3 |= (16<<10);  // SEQ3 for CHannel 16
}

void ADC_Enable (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable the ADC by setting ADON bit in CR2
	2. Wait for ADC to stabilize (approx 10us)
	************************************************/
	ADC1->CR2 |= 1<<0;   // ADON =1 enable ADC1

	uint32_t delay = 10000;
	while (delay--);
}

void ADC_Start(void)
{
	/************** STEPS TO FOLLOW *****************
	1. Clear the Status register
	2. Start the Conversion by Setting the SWSTART bit in CR2
	*************************************************/
	ADC1->SR = 0;                      // Clear Status register
	ADC1->CR2 |= (1<<20);              // Conversion on external event enabled
	ADC1->CR2 |= 1<<22;                // Start conversion
}

uint8_t Read_ADC()
{
	//uint8_t value = 0;
	 //ADC1->CR2 |= (7<<17);  // External Event selection pointed to SWSTART bit

	 //EOC Control
	 //while( !(ADC1->SR & (1<<0)));

	int value = ADC1->DR;

	return value;
}


int main(void)
{
	SystemInit();
	ADC_Initt ();
		ADC_Enable ();
		ADC_Start ();

 while(1)
 {
	 systemclock=SystemCoreClock;
     adc_value = Read_ADC();


 }
}
