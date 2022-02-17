/*
 * stm32f407_gpio_drivers.c
 *
 *  Created on: 18 Ara 2021
 *      Author: MSI
 */
#include "stm32f407_gpio_drivers.h"
#include <stdio.h>
void GPIO_init(GPIO_Handle_t *pGPIOHandle){
	GPIO_PeriCLKCtrl(pGPIOHandle->pGPIOBaseAddr, ENABLE);



	uint8_t temp=0;
	if(pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Mode<4){
		temp=(pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Mode<<2*pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
		pGPIOHandle->pGPIOBaseAddr->MODER&=~(0x11<<2*pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
		pGPIOHandle->pGPIOBaseAddr->MODER|=temp;


	}else{
		if(pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Mode==GPIO_MODER_IT_FT){
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
			EXTI->RTSR&=~(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
		}else if(pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Mode==GPIO_MODER_IT_RT){

			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
			EXTI->FTSR&=~(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);

		}else if(pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Mode==GPIO_MODER_IT_RFT){
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
		}
		SYSCF_CLK_EN();

		uint8_t res=pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number/4;
		uint8_t res1=pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number%4;
		uint8_t code=GPIOtoCODE(pGPIOHandle->pGPIOBaseAddr);
		if(res==0){
			syscfg->EXTICR1|=(code<<res1*4);


		}else if (res==1){
			syscfg->EXTICR2|=(code<<res1*4);

		}
		else if (res==2){
			syscfg->EXTICR3|=(code<<res1*4);

		}
		else if (res==3){

			syscfg->EXTICR4|=(code<<res1*4);
		}






		// 2 configure gpıo port syscfg_exticr
		// 3 enable exti  interrupt delivery using ımr


		EXTI->IMR|=(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
		//ınterrupt mode
	}
	temp=0;


	temp|=(pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Speed<<2*pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
	pGPIOHandle->pGPIOBaseAddr->OSPEEDR&=~(0x11<<2*pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
	pGPIOHandle->pGPIOBaseAddr->OSPEEDR|=temp;
	temp=0;


	temp|=(pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Pupdcontrol<<2*pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
	pGPIOHandle->pGPIOBaseAddr->PUPDR&=~(0x11<<pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
	pGPIOHandle->pGPIOBaseAddr->PUPDR|=temp;
	temp=0;


	temp=pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Optype<<pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number;
	pGPIOHandle->pGPIOBaseAddr->OTYPER&=~(0x1<<pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number);
	pGPIOHandle->pGPIOBaseAddr->OTYPER|=temp;
	temp=0;
	if(pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Mode==GPIO_MODER_ALTFUNC){
		if(pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number/8){
			uint8_t temp1;
			temp1=pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number%8;


			temp=pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Altfuncmode<<(4*temp1);
			pGPIOHandle->pGPIOBaseAddr->AFRH&=~(0x1111<<temp1);
			pGPIOHandle->pGPIOBaseAddr->AFRH|=temp1;
			temp=0;

		}else{

			uint8_t temp1;
			temp1=pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Number%8;
			temp=pGPIOHandle->GPIO_Pin_Config.GPIO_Pin_Altfuncmode<<(4*temp1);
			pGPIOHandle->pGPIOBaseAddr->AFRL&=~(0x1111<<temp1);
			pGPIOHandle->pGPIOBaseAddr->AFRL|=temp;
			temp=0;
		}
		//ALT FUNC
	}
}
void GPIO_Deinit(GPIO_RegDef_t *pGPIO){

			if(pGPIO==GPIOA){
				GPIOA_REG_RESET();

			}if(pGPIO==GPIOB){
				GPIOB_REG_RESET();

			}if(pGPIO==GPIOC){
				GPIOC_REG_RESET();

			}if(pGPIO==GPIOD){
				GPIOD_REG_RESET();

			}if(pGPIO==GPIOE){
				GPIOE_REG_RESET();

			}if(pGPIO==GPIOF){
				GPIOF_REG_RESET();

			}






}
void GPIO_PeriCLKCtrl(GPIO_RegDef_t *pGPIO,uint8_t en_dis){
	if(en_dis == ENABLE){
		if(pGPIO==GPIOA){
			GPIOA_PERI_CLK_EN();

		}if(pGPIO==GPIOB){
			GPIOB_PERI_CLK_EN();

		}if(pGPIO==GPIOC){
			GPIOC_PERI_CLK_EN();

		}if(pGPIO==GPIOD){
			GPIOD_PERI_CLK_EN();

		}if(pGPIO==GPIOE){
			GPIOE_PERI_CLK_EN();

		}if(pGPIO==GPIOF){
			GPIOF_PERI_CLK_EN();

		}


	}else if(en_dis==DISABLE){
		if(pGPIO==GPIOA){
			GPIOA_PERI_CLK_DIS();

		}if(pGPIO==GPIOB){
			GPIOB_PERI_CLK_DIS();

		}if(pGPIO==GPIOC){
			GPIOC_PERI_CLK_DIS();

		}if(pGPIO==GPIOD){
			GPIOD_PERI_CLK_DIS();

		}if(pGPIO==GPIOE){
			GPIOE_PERI_CLK_DIS();

		}if(pGPIO==GPIOF){
			GPIOF_PERI_CLK_DIS();

		}



	}

}



uint8_t GPIO_ReadFromInput(GPIO_RegDef_t *pGPIO,uint8_t pinNumber){
	uint8_t value;
	value=(pGPIO->IDR>>pinNumber)&(0x00000001);
	return value;
}
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIO){
	    uint16_t value;
		value=pGPIO->IDR;
		return value;

return 1;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIO,uint8_t pinNumber,uint8_t value){

	if(value==1){

		pGPIO->ODR|=(value<<pinNumber);

	}else if(value==0){
		pGPIO->ODR &=~(1<<pinNumber);

	}else{
		printf("error");
	}



}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIO,uint8_t value){
	pGPIO->ODR=value;



}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO,uint8_t pinNumber){
	pGPIO->ODR=pGPIO->ODR ^ (1<<pinNumber);

}


void GPIO_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDis)
{
	if(EnOrDis==ENABLE){
		if(IRQNumber<=31){
			*ISER0_Base_addr|=(1<<IRQNumber);

		}else if (IRQNumber<64 && IRQNumber>32){
			*ISER1_Base_addr|=(1<<(IRQNumber%32));

		}else if (IRQNumber<96 && IRQNumber>64){
			*ISER2_Base_addr|=(1<<(IRQNumber%64));

		}

	}else{
		if(IRQNumber<=31){
			*ICER0_Base_addr|=(1<<IRQNumber);

		}else if (IRQNumber<64 && IRQNumber>32){
			*ICER1_Base_addr|=(1<<(IRQNumber%32));

		}else if (IRQNumber<96 && IRQNumber>64){
			*ICER2_Base_addr|=(1<<(IRQNumber%64));

		}

	}


}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;
	uint8_t shift_amount=(8*iprx_section)+(8-no_of_bits_implemented);
	*(IPR0_Base_addr+(4*iprx))|=((IRQPriority)<<shift_amount);




}
void GPIO_IRQHandling(uint8_t pinNumber)
{

	if(EXTI->PR&(1<<pinNumber)){
		EXTI->PR|=(1<<pinNumber);

	}



}


