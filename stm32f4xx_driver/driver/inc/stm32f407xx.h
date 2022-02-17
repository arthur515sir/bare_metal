/*
 * stm32f407xx.h
 *
 *  Created on: Dec 14, 2021
 *      Author: MSI
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>
#include <stddef.h>
/////proccecor macros
#define ISER0_Base_addr  ((volatile  *) 0xE000E100U)
#define ISER1_Base_addr  ((volatile  *) 0xE000E104U)
#define ISER2_Base_addr  ((volatile  *) 0xE000E10CU)
#define __weak __attribute__((weak))




#define ICER0_Base_addr  ((volatile  *) 0XE000E180U)
#define ICER1_Base_addr  ((volatile  *) 0XE000E184U)
#define ICER2_Base_addr  ((volatile  *) 0XE000E18CU)

#define IPR0_Base_addr ((volatile *)0xE000E400U)
#define no_of_bits_implemented 			4
/////


#define Aa 1
#define FLASH_BASE_ADDR 0x08000000U
#define SRAM1_BASE_ADDR 0x20000000U
#define SRAM  SRAM1_BASE_ADDR
#define SRAM2_BASE_ADDR 0x20001C00U
#define ROM_BASE_ADDR 0x1FFF000U

#define APB1_BASE_ADDR 0x40000000U

#define APB2_BASE_ADDR (APB1_BASE_ADDR+0x00010000U)
#define AHB1_BASE_ADDR (APB1_BASE_ADDR+0x00020000U)
#define AHB2_BASE_ADDR (0x50000000U)


//ahb1 bus peripherals
///GPIO Peripherlas
#define GPIOA_BASE_ADDR (AHB1_BASE_ADDR )
#define GPIOB_BASE_ADDR (AHB1_BASE_ADDR+0x400U )
#define GPIOC_BASE_ADDR (AHB1_BASE_ADDR+0x800U )
#define GPIOD_BASE_ADDR (AHB1_BASE_ADDR+0xC00U )
#define GPIOE_BASE_ADDR (AHB1_BASE_ADDR+0x1000U )
#define GPIOF_BASE_ADDR (AHB1_BASE_ADDR+0x1400U)
#define GPIOG_BASE_ADDR (AHB1_BASE_ADDR+0x1800U )
#define GPIOH_BASE_ADDR (AHB1_BASE_ADDR+0x1C00U )
#define GPIOI_BASE_ADDR (AHB1_BASE_ADDR+0x2000U )
#define GPIOJ_BASE_ADDR (AHB1_BASE_ADDR+0x2400U )
#define GPIOK_BASE_ADDR (AHB1_BASE_ADDR+0x2800U )
/// rcc perihperal
#define RCC_BASE_ADDR (AHB1_BASE_ADDR+0x3800)




typedef struct{
	volatile uint8_t SR;
	volatile uint8_t DR;
	volatile uint8_t BRR;
	volatile uint8_t CR1;
	volatile uint8_t CR2;
	volatile uint8_t CR3;
	volatile uint8_t GTPR;


}USART_RegDef_t;





///İ2C PERİPERHAL APB1
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;






}I2C_RegDef_t;


//apb1 perp
#define SPI2_BASE_ADDR (APB1_BASE_ADDR+0x3800U)
#define SPI3_BASE_ADDR (APB1_BASE_ADDR+0x3C00U)




#define USART2_BASE_ADDR (APB1_BASE_ADDR+0x4400U)
#define USART3_BASE_ADDR (APB1_BASE_ADDR+0x4800U)
#define USART6_BASE_ADDR (APB2_BASE_ADDR+0x1400U)
#define USART1_BASE_ADDR (APB2_BASE_ADDR+0x1000U)





#define UART4_BASE_ADDR (APB1_BASE_ADDR+0x4C00U)
#define UART5_BASE_ADDR (APB1_BASE_ADDR+0x5000U)
#define UART8_BASE_ADDR (APB1_BASE_ADDR+0x7C00U)
#define UART7_BASE_ADDR (APB1_BASE_ADDR+0x7800U)



#define I2C1_BASE_ADDR (APB1_BASE_ADDR+0x5400U)
#define I2C2_BASE_ADDR (APB1_BASE_ADDR+0x5800U)
#define I2C3_BASE_ADDR (APB1_BASE_ADDR+0x5C00U)
//APB2 PERP
#define EXTI_BASE_ADDR (APB2_BASE_ADDR+0x3C00U)
///SPI stuct


#define SPI1 ((SPI_RegDef_t *)0x40013000U)
#define SPI2  ((SPI_RegDef_t *)0x40003800U)
#define SPI3  ((SPI_RegDef_t *)0x40003C00U)
/// I2C
#define I2C1 ((I2C_RegDef_t *)0x40005400U)
#define I2C2 ((I2C_RegDef_t *)0x40005800U)
#define I2C3 ((I2C_RegDef_t *)0x40005C00U)

/// I2C CLK ENABLE


/// UART




#define USART1 ((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2 ((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3 ((USART_RegDef_t*)USART3_BASE_ADDR)
#define USART6 ((USART_RegDef_t*)USART6_BASE_ADDR)

#define UART4  ((USART_RegDef_t*)UART4_BASE_ADDR)
#define UART5  ((USART_RegDef_t*)UART5_BASE_ADDR)
#define UART7  ((USART_RegDef_t*)UART7_BASE_ADDR)
#define UART8  ((USART_RegDef_t*)UART8_BASE_ADDR)






typedef struct{

	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;



}SPI_RegDef_t;















///

///GPIO STRUCT
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;

}GPIO_RegDef_t;

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOA_BASE_ADDR)
#define GPIOC (GPIO_RegDef_t *)GPIOA_BASE_ADDR
#define GPIOD (GPIO_RegDef_t *)GPIOA_BASE_ADDR
#define GPIOE (GPIO_RegDef_t *)GPIOA_BASE_ADDR
#define GPIOF (GPIO_RegDef_t *)GPIOA_BASE_ADDR
#define GPIOG (GPIO_RegDef_t *)GPIOA_BASE_ADDR
#define GPIOH (GPIO_RegDef_t *)GPIOA_BASE_ADDR
#define GPIOI (GPIO_RegDef_t *)GPIOA_BASE_ADDR
#define GPIOJ (GPIO_RegDef_t *)GPIOA_BASE_ADDR
#define GPIOK (GPIO_RegDef_t *)GPIOA_BASE_ADDR




///rcc structer
typedef struct{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t Reserved;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t Reserved1;
	volatile uint32_t Reserved2;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t Reserved3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t Reserved4;
	volatile uint32_t Reserved5;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t Reserved6;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t Reserved7;
	volatile uint32_t Reserved8;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t Reserved9;
	volatile uint32_t Reserve10;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;



}RCC_RegDef_t;
#define RCC ((RCC_RegDef_t*)RCC_BASE_ADDR)





/// CLOCK ENABLE MACRO
#define GPIOA_PERI_CLK_EN()		(RCC->AHB1ENR|=(1<<0))
#define GPIOB_PERI_CLK_EN()		(RCC->AHB1ENR|=(1<<1))
#define GPIOC_PERI_CLK_EN()		(RCC->AHB1ENR|=(1<<2))
#define GPIOD_PERI_CLK_EN()		(RCC->AHB1ENR|=(1<<3))
#define GPIOE_PERI_CLK_EN()		(RCC->AHB1ENR|=(1<<4))
#define GPIOF_PERI_CLK_EN()		(RCC->AHB1ENR|=(1<<5))
#define GPIOG_PERI_CLK_EN()		(RCC->AHB1ENR|=(1<<6))
#define GPIOH_PERI_CLK_EN()		(RCC->AHB1ENR|=(1<<7))


///I2C CLK ENAB
#define I2C1_CLK_EN()	(RCC->APB1ENR |=1<<21)
#define I2C2_CLK_EN()	(RCC->APB1ENR |=1<<22)
#define I2C3_CLK_EN()	(RCC->APB1ENR |=1<<23)

///I2C CLK DİSAB
#define I2C1_CLK_DIS()	(RCC->APB1ENR &=~1<<21)
#define I2C2_CLK_DIS()	(RCC->APB1ENR &=~1<<22)
#define I2C3_CLK_DIS()	(RCC->APB1ENR &=~1<<23)
// ı2c clk reset
#define I2C1_CLK_RST()	do{RCC->APB1ENR |=1<<21;RCC->APB1ENR &=~1<<21;}while(0)
#define I2C2_CLK_RST()	do{RCC->APB1ENR |=1<<22;RCC->APB1ENR &=~1<<22;}while(0)
#define I2C3_CLK_RST()	do{RCC->APB1ENR |=1<<23;RCC->APB1ENR &=~1<<23;}while(0)

#define SYSCF_CLK_EN()			(RCC->APB2ENR|=(1<<14))
/// CLOCK DİSABLE MACRO
#define GPIOA_PERI_CLK_DIS()		(RCC->AHB1ENR &=~ (1<<0))
#define GPIOB_PERI_CLK_DIS()		(RCC->AHB1ENR &=~ (1<<1))
#define GPIOC_PERI_CLK_DIS()		(RCC->AHB1ENR &=~ (1<<2))
#define GPIOD_PERI_CLK_DIS()		(RCC->AHB1ENR &=~ (1<<3))
#define GPIOE_PERI_CLK_DIS()		(RCC->AHB1ENR &=~ (1<<4))
#define GPIOF_PERI_CLK_DIS()		(RCC->AHB1ENR &=~ (1<<5))
#define GPIOG_PERI_CLK_DIS()		(RCC->AHB1ENR &=~ (1<<6))
#define GPIOH_PERI_CLK_DIS()		(RCC->AHB1ENR&=~(1<<7))
#define GPIOJ_PERI_CLK_DIS()		(RCC->AHB1ENR&=~(1<<8))

///spı clk enable
#define SPI1_CLK_EN()			(RCC->APB2ENR|=(1<<12))
#define SPI2_CLK_EN()			(RCC->APB1ENR|=(1<<14))
#define SPI3_CLK_EN()			(RCC->APB1ENR|=(1<<15))
///spı clk disable
#define SPI1_CLK_DIS()			(RCC->APB2ENR&=~(1<<12))
#define SPI2_CLK_DIS()			(RCC->APB1ENR&=~(1<<14))
#define SPI3_CLK_DIS()			(RCC->APB1ENR&=~(1<<15))



///uart clk enable
#define UART5_CLK_EN()			(RCC->APB1ENR|=(1<<20))
#define UART4_CLK_EN()			(RCC->APB1ENR|=(1<<19))
#define UART8_CLK_EN()			(RCC->APB1ENR|=(1<<31))
#define UART7_CLK_EN()			(RCC->APB1ENR|=(1<<30))

#define USART3_CLK_EN()			(RCC->APB1ENR|=(1<<18))
#define USART2_CLK_EN()			(RCC->APB1ENR|=(1<<17))
#define USART6_CLK_EN()			(RCC->APB2ENR|=(1<<5))
#define USART1_CLK_EN()			(RCC->APB2ENR|=(1<<4))
///UART CLK DİSABLE
#define UART5_CLK_DIS()			(RCC->APB1ENR &=~(1<<20))
#define UART4_CLK_DIS()			(RCC->APB1ENR &=~(1<<19))
#define UART8_CLK_DIS()			(RCC->APB1ENR &=~(1<<31))
#define UART7_CLK_DIS()			(RCC->APB1ENR &=~(1<<30))

#define USART3_CLK_DIS()			(RCC->APB1ENR &=~(1<<18))
#define USART2_CLK_DIS()			(RCC->APB1ENR &=~(1<<17))
#define USART6_CLK_DIS()			(RCC->APB2ENR &=~(1<<5))
#define USART1_CLK_DIS()			(RCC->APB2ENR &=~(1<<4))
/// UART CLK RESET
#define UART5_CLK_RST()				do{RCC->APB1ENR|=(1<<20);RCC->APB1ENR &=~(1<<20);} while(0)
#define UART4_CLK_RST()				do{RCC->APB1ENR|=(1<<19);RCC->APB1ENR &=~(1<<19);} while(0)
#define UART8_CLK_RST()				do{RCC->APB1ENR|=(1<<31);RCC->APB1ENR &=~(1<<31);} while(0)
#define UART7_CLK_RST()				do{RCC->APB1ENR|=(1<<30);RCC->APB1ENR &=~(1<<30);} while(0)


#define USART3_CLK_RST()				do{RCC->APB1ENR|=(1<<18);RCC->APB1ENR &=~(1<<18);} while(0)
#define USART2_CLK_RST()				do{RCC->APB1ENR|=(1<<17);RCC->APB1ENR &=~(1<<17);} while(0)
#define USART6_CLK_RST()				do{RCC->APB2ENR|=(1<<5);RCC->APB2ENR &=~(1<<5);} while(0)
#define USART1_CLK_RST()				do{RCC->APB2ENR|=(1<<4);RCC->APB2ENR &=~(1<<4);} while(0)



#define ENABLE 	1
#define DISABLE 0

#define GPIOA_REG_RESET()    do{RCC->AHB1ENR|=(1<<0);RCC->AHB1ENR &=~ (1<<0);} while(0);
#define GPIOB_REG_RESET()    do{RCC->AHB1ENR|=(1<<1);RCC->AHB1ENR &=~ (1<<1);} while(0);
#define GPIOC_REG_RESET()    do{RCC->AHB1ENR|=(1<<2);RCC->AHB1ENR &=~ (1<<2);} while(0);
#define GPIOD_REG_RESET()    do{RCC->AHB1ENR|=(1<<3);RCC->AHB1ENR &=~ (1<<3);} while(0);
#define GPIOE_REG_RESET()    do{RCC->AHB1ENR|=(1<<4);RCC->AHB1ENR &=~ (1<<4);} while(0);
#define GPIOF_REG_RESET()    do{RCC->AHB1ENR|=(1<<5);RCC->AHB1ENR &=~ (1<<5);} while(0);
#define GPIOG_REG_RESET()    do{RCC->AHB1ENR|=(1<<6);RCC->AHB1ENR &=~ (1<<6);} while(0);
#define GPIOH_REG_RESET()    do{RCC->AHB1ENR|=(1<<7);RCC->AHB1ENR &=~ (1<<7);} while(0);
#define GPIOJ_REG_RESET()    do{RCC->AHB1ENR|=(1<<8);RCC->AHB1ENR &=~ (1<<8);} while(0);
#define GPIOtoCODE(param)	 ((param==GPIOA)? 0:\
							 (param==GPIOB)? 1:\
							 (param==GPIOC)? 2:\
							 (param==GPIOD)? 3:\
							 (param==GPIOE)? 4:\
							 (param==GPIOF)? 5:\
							 (param==GPIOG)? 6:\
							 (param==GPIOH)? 7:\
							 (param==GPIOH)? 8:0)


#define SPI1_REG_RESET()			do{RCC->APB2ENR|=(1<<12);RCC->APB2ENR&=~(1<<12);}while(0)
#define SPI2_REG_RESET()			do{RCC->APB1ENR|=(1<<14);RCC->APB2ENR&=~(1<<12);}while(0)
#define SPI3_REG_RESET()            do{RCC->APB1ENR|=(1<<15);RCC->APB2ENR&=~(1<<12);}while(0)

#define EXTI_Base_Addr 0x40013C00U


typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;




}EXTI_Reg_Def_t;
#define EXTI    ((EXTI_Reg_Def_t *)EXTI_Base_Addr)


#define sys_cfg_base_addr 0x40013800
typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR1;
	volatile uint32_t EXTICR2;
	volatile uint32_t EXTICR3;
	volatile uint32_t EXTICR4;
	volatile uint32_t CMPCR;


}SysCfg_RegDef_t;
#define syscfg     ((SysCfg_RegDef_t*)sys_cfg_base_addr)

#define IRQ_NO_EXTI0	6
#define IRQ_NO_EXTI1	7
#define IRQ_NO_EXTI2	8
#define IRQ_NO_EXTI3	9
#define IRQ_NO_EXTI4	10

#define IRQ_NO_EXTI9_5	   23
#define IRQ_NO_EXTI15_10   40


//SPI IRQ NUMBERS
#define IRQ_SPI1	35
#define IRQ_SPI2	36
#define IRQ_SPI3	51
// I2C IRQ NUMBERS
#define IRQ_NO_I2C1_EV 31
#define IRQ_NO_I2C1_ER 32






#include "stm32f407_gpio_drivers.h"
#include "stm32f407_spi_drivers.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407_uart_driver.h"


#endif /* INC_STM32F407XX_H_ */







