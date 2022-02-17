/*
 * stm32f407_gpio_drivers.h
 *
 *  Created on: 18 Ara 2021
 *      Author: MSI
 */

#ifndef INC_STM32F407_GPIO_DRIVERS_H_
#define INC_STM32F407_GPIO_DRIVERS_H_
#include "stm32f407xx.h"



#define GPIO_MODER_INPUT 0
#define GPIO_MODER_OUTPUT 1
#define GPIO_MODER_ALTFUNC 2
#define GPIO_MODER_ANALOG 3
#define GPIO_MODER_IT_FT 4
#define GPIO_MODER_IT_RT 5
#define GPIO_MODER_IT_RFT 6

#define GPIO_OTYPER_OUTPP 0
#define GPIO_OTYPER_OUTOD 1

#define GPIO_OSPEEDR_LOW 0
#define GPIO_OSPEEDR_MEDIUM 1
#define GPIO_OSPEEDR_HIGH 2
#define GPIO_OSPEEDR_VHIGH 3


#define GPIO_PUPDR_NOPUPD 0
#define GPIO_PUPDR_PU 1
#define GPIO_PUPDR_PD 2


#define PIN0 0
#define PIN1 1
#define PIN2 2
#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7
#define PIN8 8
#define PIN9 9
#define PIN10 10
#define PIN11 11
#define PIN12 12
#define PIN13 13
#define PIN14 14
#define PIN15 15











typedef struct{
	uint8_t GPIO_Pin_Number;
	uint8_t GPIO_Pin_Mode;
	uint8_t GPIO_Pin_Speed;
	uint8_t GPIO_Pin_Pupdcontrol;
	uint8_t GPIO_Pin_Optype;
	uint8_t GPIO_Pin_Altfuncmode;

}GPIO_Pin_Config_t;

typedef struct
{
	//hold the base adress of gpıo struct
	GPIO_RegDef_t *pGPIOBaseAddr;
	//config struct
	GPIO_Pin_Config_t  GPIO_Pin_Config;


}GPIO_Handle_t;


//periherals funcs
void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIO);
void GPIO_PeriCLKCtrl(GPIO_RegDef_t *pGPIO,uint8_t en_dis);



uint8_t GPIO_ReadFromInput(GPIO_RegDef_t *pGPIO,uint8_t pinNumber);
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIO);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIO,uint8_t pinNumber,uint8_t value);

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIO,uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO,uint8_t pinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);

void GPIO_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDis);
void GPIO_IRQHandling(uint8_t pinNumber);
















#endif /* INC_STM32F407_GPIO_DRIVERS_H_ */
