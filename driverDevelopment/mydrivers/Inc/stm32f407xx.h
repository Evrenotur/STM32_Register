

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <string.h>



/*
 * Microprocessor Defines
 */
#define NVIC_ISER0              ((uint32_t*)(0XE000100UL))




#define _IO volatile
#define SET_BIT(REG,BIT)        ((REG)  |=  (BIT))
#define CLEAR_BIT(REG, BIT)     ((REG)  &= ~(BIT))
#define READ_BIT(REG,BIT)       ((REG)  &   (BIT))
#define UNSINED(x)              (void)x  // temp value için

typedef enum
{
	DISABLE = 0X0U,
	ENABLE = !DISABLE
}FunctionStade_t;

/*
 *
 * IRQ Numbers Of MCU == Vector Table
 *
 */

typedef enum
{
       EXTI0_IRQNUMBER = 6,
	   EXTI1_IRQNUMBER = 7,
	   EXTI2_IRQNUMBER = 8,
	   EXTI3_IRQNUMBER = 9

}IRQNumber_TypeDef_t;




/*
 *
 * Memory Base Address
 *
 */

#define FLASH_BASE_ADDR                (0x08000000UL)                 /*Flash Base Address(1 MB)  */
#define SRAM1_BASE_ADDR                (0x20000000UL)                 /*SRAM1 Base Address (112kb)*/
#define SRAM2_BASE_ADDR                (0x2001C000UL)                 /*SRAM2 Base Address (16kb) */

/*
 *
 * Peripheral Base Addresses
 *
 */

#define PERIPH_BASE_ADDR            (0x40000000UL)                       /*Base Address for All Peripherals*/

#define APB1_BASE_ADDR             PERIPH_BASE_ADDR                      /*APB1 Bus Domain Base  Address  */
#define APB2_BASE_ADDR             (PERIPH_BASE_ADDR  + 0x00010000UL)    /*APB2 Bus Domain Base  Address  */
#define AHB1_BASE_ADDR             (PERIPH_BASE_ADDR  + 0X00020000UL)    /*AHB1 Bus Domain Base  Address  */
#define AHB2_BASE_ADDR             (PERIPH_BASE_ADDR  + 0x10000000UL)    /*AHB2 Bus Domain Base  Address  */



/*
 *
 * APB1 Peripherals Base Addresses
 *
 */

#define TIM2_BASE_ADDR            (APB1_BASE_ADDR + 0X0000UL)           /*Timer 2 Base Address  */
#define TIM3_BASE_ADDR            (APB1_BASE_ADDR + 0X0400UL)           /*Timer 3 Base Address  */
#define TIM4_BASE_ADDR            (APB1_BASE_ADDR + 0X0800UL)           /*Timer 4 Base Address  */
#define TIM5_BASE_ADDR            (APB1_BASE_ADDR + 0X0C00UL)           /*Timer 5 Base Address  */
#define TIM6_BASE_ADDR            (APB1_BASE_ADDR + 0X1000UL)           /*Timer 6 Base Address  */
#define TIM7_BASE_ADDR            (APB1_BASE_ADDR + 0X1400UL)           /*Timer 7 Base Address  */
#define TIM12_BASE_ADDR           (APB1_BASE_ADDR + 0X1800UL)           /*Timer 12 Base Address */
#define TIM13_BASE_ADDR           (APB1_BASE_ADDR + 0X1C00UL)           /*Timer 13 Base Address */
#define TIM14_BASE_ADDR           (APB1_BASE_ADDR + 0X2000UL)           /*Timer 14 Base Address */

#define SPI2_BASE_ADDR            (APB1_BASE_ADDR + 0X3800UL)           /*SPI 2 Base Address    */
#define SPI3_BASE_ADDR            (APB1_BASE_ADDR + 0X3C00UL)           /*SPI 3 Base Address    */

#define USART2_BASE_ADDR          (APB1_BASE_ADDR + 0X4400UL)           /*USART 2 Base Address   */
#define USART3_BASE_ADDR          (APB1_BASE_ADDR + 0X4800UL)           /*USART 3 Base Address   */
#define UART4_BASE_ADDR           (APB1_BASE_ADDR + 0X4C00UL)           /*UART  4 Base Address   */
#define UART5_BASE_ADDR           (APB1_BASE_ADDR + 0X5000UL)           /*UART  5 Base Address   */

#define I2C_1_BASE_ADDR           (APB1_BASE_ADDR + 0X5400UL)           /*I2C   1 Base Address   */
#define I2C_2_BASE_ADDR           (APB1_BASE_ADDR + 0X5800UL)           /*I2C   2 Base Address   */
#define I2C_3_BASE_ADDR           (APB1_BASE_ADDR + 0X5C00UL)           /*I2C   3 Base Address   */
/*
 * APB2 PeripheralS Base Addresses
 */
#define TIM1_BASE_ADDR            (APB2_BASE_ADDR + 0X0000UL)          /*Timer  1 Base Address  */
#define TIM8_BASE_ADDR            (APB2_BASE_ADDR + 0X0400UL)          /*Timer  8 Base Address  */

#define USART1_BASE_ADDR          (APB2_BASE_ADDR + 0X1000UL)          /*USART  1 Base Address  */
#define USART6_BASE_ADDR          (APB2_BASE_ADDR + 0X1400UL)          /*USART  6 Base Address  */

#define SPI1_BASE_ADDR            (APB2_BASE_ADDR + 0X3000UL)          /*SPI    1 Base Address  */
#define SPI4_BASE_ADDR            (APB2_BASE_ADDR + 0X3400UL)          /*SPI    4 Base Address  */

#define SYSCFG_BASE_ADDR          (APB2_BASE_ADDR + 0X3800UL)          /*SYSCFG    Base Address  */
#define EXTI_BASE_ADDR            (APB2_BASE_ADDR + 0x3C00UL)          /*EXTI      Base Address  */

/*
 * AHB1 PeripheralS Base Addresses
 */

#define GPIOA_BASE_ADDR           (AHB1_BASE_ADDR  + 0X0000UL)         /*GPIO A  Base Address   */
#define GPIOB_BASE_ADDR           (AHB1_BASE_ADDR  + 0X0400UL)         /*GPIO B  Base Address   */
#define GPIOC_BASE_ADDR           (AHB1_BASE_ADDR  + 0X0800UL)         /*GPIO C  Base Address   */
#define GPIOD_BASE_ADDR           (AHB1_BASE_ADDR  + 0X0C00UL)         /*GPIO D  Base Address   */
#define GPIOE_BASE_ADDR           (AHB1_BASE_ADDR  + 0X1000UL)         /*GPIO E  Base Address   */
#define GPIOF_BASE_ADDR           (AHB1_BASE_ADDR  + 0X1400UL)         /*GPIO F  Base Address   */
#define GPIOG_BASE_ADDR           (AHB1_BASE_ADDR  + 0X1800UL)         /*GPIO G  Base Address   */
#define GPIOH_BASE_ADDR           (AHB1_BASE_ADDR  + 0X1C00UL)         /*GPIO H  Base Address   */
#define GPIOI_BASE_ADDR           (AHB1_BASE_ADDR  + 0X2000UL)         /*GPIO I  Base Address   */
#define GPIOJ_BASE_ADDR           (AHB1_BASE_ADDR  + 0X2400UL)         /*GPIO J  Base Address   */
#define GPIOK_BASE_ADDR           (AHB1_BASE_ADDR  + 0X2800UL)         /*GPIO K  Base Address   */

#define RCC_BASE_ADDR             (AHB1_BASE_ADDR  + 0X3800UL)         /*RCC     Base Address    */


/*
 * Peripheral  Structure Definitions
 */



typedef struct
{
 _IO 	    uint32_t MODER;          /*!< GPIO port mode register                Address Offset 0x0000 */
 _IO        uint32_t OTYPER;         /*!< GPIO port output type register         Address Offset 0x0004 */
 _IO        uint32_t OSPEEDR;        /*!< GPIO port output speed register        Address Offset 0x0008 */
 _IO      	uint32_t PUPDR;          /*!< GPIO port pull-up/pull-down            Address Offset 0x000C */
 _IO        uint32_t IDR;            /*!< GPIO port input data                   Address Offset 0x0010 */
 _IO      	uint32_t ODR;            /*!< GPIO port output data register         Address Offset 0x0014 */
 _IO      	uint32_t BSRR;           /*!< GPIO port bit set/reset register       Address Offset 0x0018 */
 _IO        uint32_t LCKR;           /*!< GPIO port configuration lock register  Address Offset 0x001C */
 _IO     	uint32_t AFR[2];         /*!< GPIO alternate function register       Address Offset 0x0020 */



}GPIO_TypeDef_t;

typedef struct
{

	_IO uint32_t CR;               /*!< RCC Clock Control register                                          Address Offset 0x0000 */
	_IO uint32_t PLLCFGR;          /*!< RCC PLL Configuration register                                      Address Offset 0x0004 */
	_IO uint32_t CFGR;             /*!< RCC Clock Configuration register                                    Address Offset 0x0008 */
	_IO uint32_t CIR;              /*!< RCC Clock interrupt register                                        Address Offset 0x000C */
	_IO uint32_t AHB1RSTR;         /*!< RCC AHB1 Peripheral reset register                                  Address Offset 0x0010 */
	_IO uint32_t AHB2RSTR;         /*!< RCC AHB2 Peripheral reset register                                  Address Offset 0x0014 */
	_IO uint32_t AHB3RSTR;         /*!< RCC AHB3 Peripheral reset register                                  Address Offset 0x0018 */
	_IO uint32_t RESEVERD0;        /*!< RESEVERD ARE                                                        Address Offset 0x001C */
	_IO uint32_t APB1RSTR;         /*!< RCC APB1 Peripheral reset register                                  Address Offset 0x0020 */
	_IO uint32_t APB2RSTR;         /*!< RCC APB2 Peripheral reset register                                  Address Offset 0x0024 */
	_IO uint32_t RESEVERD1[2];     /*!< RESEVERD ARE                                                        Address Offset 0x0028 */
	_IO uint32_t AHB1ENR;          /*!< RCC AHB1 Peripheral clock enable register                           Address Offset 0x0030 */
	_IO uint32_t AHB2ENR;          /*!< RCC AHB2 Peripheral clock enable register                           Address Offset 0x0034 */
	_IO uint32_t AHB3ENR;          /*!< RCC AHB3 Peripheral clock enable register                           Address Offset 0x0038 */
	_IO uint32_t RESEVERD2;        /*!< RESEVERD ARE                                                        Address Offset 0x003C */
	_IO uint32_t APB1ENR;          /*!< RCC APB1ENR Peripheral clock enable register                        Address Offset 0x0040 */
	_IO uint32_t APB2ENR;          /*!< RCC APB2ENR Peripheral clock enable register                        Address Offset 0x0044 */
	_IO uint32_t RESEVERD3[2];     /*!< RESEVERD ARE                                                        Address Offset 0x0048 */
	_IO uint32_t AHB1LPENR;        /*!< RCC AHPB1ENR Peripheral clock enable in low power mode register     Address Offset 0x0050 */
	_IO uint32_t AHB2LPENR;        /*!< RCC AHPB2ENR Peripheral clock enable in low power mode register     Address Offset 0x0054 */
	_IO uint32_t AHB3LPENR;        /*!< RCC AHPB3ENR Peripheral clock enable in low power mode register     Address Offset 0x0058 */
	_IO uint32_t RESEVERD4;        /*!< RESEVERD ARE                                                        Address Offset 0x005C */
	_IO uint32_t APB1LPENR;        /*!< RCC APB1LPENR Peripheral clock enable in low power mode register    Address Offset 0x0060 */
	_IO uint32_t APB2LPENR;        /*!< RCC APB2LPENR Peripheral clock enable in low power mode register    Address Offset 0x0064 */
	_IO uint32_t RESEVERD5[2];     /*!< RESEVERD ARE                                                        Address Offset 0x0068 */
	_IO uint32_t BDCR;             /*!< RCC Backup domain control register                                  Address Offset 0x0070 */
	_IO uint32_t CSR;              /*!< RCC clock control & status register                                 Address Offset 0x0074 */
	_IO uint32_t RESEVERD6[2];     /*!< RESEVERD ARE                                                        Address Offset 0x0078 */
	_IO uint32_t SSCGR;            /*!< RCC spread spectrum clock generation register                       Address Offset 0x0080 */
	_IO uint32_t PLLI2SCFGR;       /*!< GPIO PLLIS configuration register                                   Address Offset 0x0084 */

}RCC_TypeDef_t;

typedef struct
{
	_IO uint32_t MEMRMP;          /*< SYSCFG memory remap register                     Address offset = 0x00 */
	_IO uint32_t PMC;            /*< SYSCFG peripheral mode configuration register   Address offset  = 0x04 */
	_IO uint32_t EXTI_CR[4];      /*< SYSCFG external interrupt configuration register Address offset = 0x08 */
	_IO uint32_t CMPCR;         /*< Compensation cell control register               Address offset = 0x20 */

}SYSCFG_TypeDef_t;

typedef struct
{
	_IO uint32_t  IMR;           /*< Interrupt mask register             Address offset = 0x00 */
	_IO uint32_t  EMR;           /*< Event mask register                 Address offset = 0x04 */
	_IO uint32_t  RTSR;          /*< Rising trigger selection register   Address offset = 0x08 */
	_IO uint32_t  FTSR;          /*< Falling trigger selection register  Address offset = 0x0C */
	_IO uint32_t  PR;            /*< Pending register                    Address offset = 0x14 */
}EXTI_TypeDef_t;



/* Bases addres definitions of ports */


#define GPIOA                    ( (GPIO_TypeDef_t*)(GPIOA_BASE_ADDR    ))
#define GPIOB                    ( (GPIO_TypeDef_t*)(GPIOB_BASE_ADDR    ))
#define GPIOC                    ( (GPIO_TypeDef_t*)(GPIOC_BASE_ADDR    ))
#define GPIOD                    ( (GPIO_TypeDef_t*)(GPIOD_BASE_ADDR    ))
#define GPIOE                    ( (GPIO_TypeDef_t*)(GPIOE_BASE_ADDR    ))
#define GPIOF                    ( (GPIO_TypeDef_t*)(GPIOF_BASE_ADDR    ))
#define RCC                      ( (RCC_TypeDef_t* )(RCC_BASE_ADDR      ))

#define SYSCFG                   ( (SYSCFG_TypeDef_t *)(SYSCFG_BASE_ADDR))

#define EXTI                     ( (EXTI_TypeDef_t*)(EXTI_BASE_ADDR     ))

/*
 * Bit Definitions
 */
#define RCC_AHB1ENR_GPIOAEN_Pos  (0U)                                    // RCC AHB1ENR Register GPIOAEN Bit Position
#define RCC_AHB1ENR_GPIOAEN_Msk (0X1 << RCC_AHB1ENR_GPIOAEN_Pos )        // RCC AHB1ENR Register GPIOAEN Bit Msk
#define RCC_AHB1ENR_GPIOAEN      RCC_AHB1ENR_GPIOAEN_Msk                 // RCC AHB1ENR Register GPIOAEN

#define RCC_AHB1ENR_GPIOBEN_Pos  (1U)                                    //RCC AHB1ENR  Register GPIOBEN Bit Position
#define RCC_AHB1ENR_GPIOBEN_Msk  (0X1 << RCC_AHB1ENR_GPIOBEN_Pos  )      // RCC AHB1ENR Register GPIOBEN Bit Msk
#define RCC_AHB1ENR_GPIOBEN      RCC_AHB1ENR_GPIOBEN_Msk                 // RCC AHB1ENR Register GPIOBEN

#define RCC_AHB1ENR_GPIOCEN_Pos  (2U)                                    //RCC AHB1ENR  Register GPIOCEN Bit Position
#define RCC_AHB1ENR_GPIOCEN_Msk  (0X1 << RCC_AHB1ENR_GPIOCEN_Pos  )      // RCC AHB1ENR Register GPIOCBEN Bit Msk
#define RCC_AHB1ENR_GPIOCEN      RCC_AHB1ENR_GPIOCEN_Msk                 // RCC AHB1ENR Register GPIOCEN

#define RCC_AHB1ENR_GPIODEN_Pos  (3U)                                    //RCC AHB1ENR  Register GPIODEN Bit Position
#define RCC_AHB1ENR_GPIODEN_Msk  (0X1 << RCC_AHB1ENR_GPIODEN_Pos  )      // RCC AHB1ENR Register GPIODBEN Bit Msk
#define RCC_AHB1ENR_GPIODEN      RCC_AHB1ENR_GPIODEN_Msk                 // RCC AHB1ENR Register GPIODEN


#define RCC_AHB1ENR_GPIOEEN_Pos  (4U)                                    //RCC AHB1ENR  Register GPIOEEN Bit Position
#define RCC_AHB1ENR_GPIOEEN_Msk  (0X1 << RCC_AHB1ENR_GPIOEEN_Pos  )      // RCC AHB1ENR Register GPIOEBEN Bit Msk
#define RCC_AHB1ENR_GPIOEEN      RCC_AHB1ENR_GPIOEEN_Msk                 // RCC AHB1ENR Register GPIOEEN


#define RCC_AHB1ENR_GPIOFEN_Pos  (5U)                                    //RCC AHB1ENR  Register GPIOFEN Bit Position
#define RCC_AHB1ENR_GPIOFEN_Msk  (0X1 << RCC_AHB1ENR_GPIOFEN_Pos  )      // RCC AHB1ENR Register GPIOFBEN Bit Msk
#define RCC_AHB1ENR_GPIOFEN      RCC_AHB1ENR_GPIOFEN_Msk                 // RCC AHB1ENR Register GPIOFEN


#define RCC_APB2ENR_SYSCFGEN_Pos (14U)                                  //RCC  APB2ENR  Register SYSCFGEN Bit Position
#define RCC_APB2ENR_SYSCFGEN_Msk (0x1 << RCC_APB2ENR_SYSCFGEN_Pos )     // RCC APB2ENR  Register SYSCFGEN Bit Position
#define RCC_APB2ENR_SYSCFGEN     RCC_APB2ENR_SYSCFGEN_Msk               // RCC APB2ENR  Register SYSCFGEN Bit Position


#define RCC_APB2ENR_TIM2_Pos (0U)                                     //RCC  APB2ENR  Register TIM1 Bit Position
#define RCC_APB2ENR_TIM2_Msk (0x1 << RCC_APB2ENR_TIM1_Pos )            // RCC APB2ENR  Register TIM1 Bit Position
#define RCC_APB2ENR_TIM2EN    RCC_APB2ENR_TIM2_Msk                     // RCC APB2ENR  Register TIM1 Bit Position

#include "RCC.h"
#include "GPIO.h"
#include "EXTI.h"
#endif /* INC_STM32F407XX_H_ */
