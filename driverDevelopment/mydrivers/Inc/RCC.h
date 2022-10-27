

#ifndef INC_RCC_H_
#define INC_RCC_H_
#include "stm32f407xx.h"
/*
 *
 *  RCC AHB1 Peripherals Clock Control Macro Definitions
 */


#define RCC_GPIOA_CLK_ENABLE()    do{ uint32_t tempvalue;\
                                     SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN);   \
                                     tempvalue = READ_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN ); \
									 UNSINED(tempvalue); \
                                    }while(0)


#define RCC_GPIOB_CLK_ENABLE()  do{ uint32_t tempvalue ; \
                                    SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN);  \
	                                tempvalue = READ_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN); \
	                                UNSINED(tempvalue); \
                                   }while(0)

#define RCC_GPIOC_CLK_ENABLE()  do{ uint32_t tempvalue ; \
                                    SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN);  \
	                                tempvalue = READ_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN); \
	                                UNSINED(tempvalue); \
                                   }while(0)

#define RCC_GPIOD_CLK_ENABLE()  do{ uint32_t tempvalue ; \
                                    SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN);  \
	                                tempvalue = READ_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN); \
	                                UNSINED(tempvalue); \
                                   }while(0)


#define RCC_GPIOE_CLK_ENABLE()  do{ uint32_t tempvalue ; \
                                    SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOEEN);  \
	                                tempvalue = READ_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOEEN); \
	                                UNSINED(tempvalue); \
                                   }while(0)


#define RCC_GPIOF_CLK_ENABLE()  do{ uint32_t tempvalue ; \
                                    SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOFEN);  \
	                                tempvalue = READ_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOFEN); \
	                                UNSINED(tempvalue); \
                                   }while(0)



#define RCC_SYSCFG_CLK_ENABLE()   do{ uint32_t tempvalue ; \
                                    SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN);  \
	                                tempvalue = READ_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN); \
	                                UNSINED(tempvalue); \
                                   }while(0)


#define RCC_TIM1_CLK_ENABLE()   do{ uint32_t tempvalue; \
                                   tempvalue = SET_BIT(RCC->APB2_BASE_ADDR,RCC_APB2ENR_TIM2EN); \
                                   UNSINED(tempvalue); \
                                   }while(0)






#define RCC_GPIOA_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN)
#define RCC_GPIOB_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN)
#define RCC_GPIOC_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN)
#define RCC_GPIOD_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN)
#define RCC_GPIOE_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOEEN)
#define RCC_GPIOF_CLK_DISABLE()     CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOFEN)
#define RCC_APB2ENR_TIM2_DISABLE()  CLEAR_BIT(RCC->APB2_BASE_ADDR,RCC_APB2ENR_TIM2EN)

#endif /* INC_RCC_H_ */
