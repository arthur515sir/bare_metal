/*
 * stm32f407_spi_drivers.c
 *
 *  Created on: 28 Ara 2021
 *      Author: MSI
 */
#include "stm32f407_spi_drivers.h"

static void spi_rxe_IThandle(SPI_Handle_t *pSPIHandle);
static void spi_txe_IThandle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_IThandle(SPI_Handle_t *pSPIHandle);

void SPI_PeriCLKCtrl(SPI_RegDef_t *pSPI,uint8_t en_dis)
{
	if(en_dis == ENABLE){
		if(pSPI==SPI1){
			SPI1_CLK_EN();

		}if(pSPI==SPI2){
			SPI2_CLK_EN();

		}if(pSPI==SPI3){
			SPI3_CLK_EN();

		}

	}else if(en_dis==DISABLE){
		if(pSPI==SPI1){
			SPI1_CLK_DIS();

		}if(pSPI==SPI2){
			SPI2_CLK_DIS();

		}if(pSPI==SPI3){
			SPI3_CLK_DIS();

				}
	}

}



void SPI_init(SPI_Handle_t *pSPIHandle){
	///İNİT SPI
	uint32_t tempreg=0;

	tempreg|=(pSPIHandle->spi_config.SPI_DeviceMode<<2);
	if(pSPIHandle->spi_config.SPI_BusConfig==SPI_BUSCFG_FD){

		tempreg&=~(1<<15);
		//bidi mode should be cleared
	}else if (pSPIHandle->spi_config.SPI_BusConfig==SPI_BUSCFG_HD){
		tempreg|=(1<<15);
		//bidi mode should be set

	}else if (pSPIHandle->spi_config.SPI_BusConfig==SPI_BUSCFG_SIMPLEX_RXONLY){
		//biid mode cleared
		tempreg&=~(1<<15);
		tempreg|=(1<<10);
	//rxonly bit must set


	}

	tempreg|=(pSPIHandle->spi_config.SPI_CPHA<<0);
	tempreg|=(pSPIHandle->spi_config.SPI_CPOL<<1);
	tempreg|=(pSPIHandle->spi_config.SPI_DFF<<11);
	tempreg|=(pSPIHandle->spi_config.SPI_Sclkspeed<<3);
	pSPIHandle->pSPIx->CR1=tempreg;



}
void SPI_Deinit(SPI_RegDef_t *pSPI){
	if(pSPI==SPI1){
		SPI1_REG_RESET();

	}if(pSPI==SPI2){
		SPI2_REG_RESET();

	}if(pSPI==SPI3){
		SPI3_REG_RESET();

		}





}
void SPI_SendData(SPI_RegDef_t *pSPI,uint8_t *pTxBuffer,uint32_t len){
	while(len>0){
		while(!(pSPI->SR&(1<<1)));

		if(pSPI->CR1&(1<<11)){
			pSPI->DR=*((uint16_t *)pTxBuffer);
			len--;
			len--;
			(uint16_t *)pTxBuffer++;
		}else{
			pSPI->DR=*(pTxBuffer);
			len--;
			pTxBuffer++;


		}





	}



}
void SPI_ReceiveData(SPI_RegDef_t *pSPI,uint8_t *pRxBuffer,uint32_t len){
	while(len>0){
		//wait until rxen is set
			while(!(pSPI->SR&(1<<0)));

			if(pSPI->CR1&(1<<11)){
				//load the data from dr to buffer
				*((uint16_t *)pRxBuffer)=pSPI->DR;
				len--;
				len--;
				(uint16_t *)pRxBuffer++;
			}else{
				*(pRxBuffer)=pSPI->DR;
				len--;
				pRxBuffer++;

			}

		}

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);

void SPI_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDis){

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
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1,temp2;
	temp1=pSPIHandle->pSPIx->SR&(1<<1);
	temp2=pSPIHandle->pSPIx->CR2&(1<<7);
	if(temp1&&temp2){
		spi_txe_IThandle(pSPIHandle);

	}

	//CHECK FOR RXEN
	temp1=pSPIHandle->pSPIx->SR&(1<<0);
	temp2=pSPIHandle->pSPIx->CR2&(1<<6);
	if(temp1&&temp2){
		spi_rxe_IThandle(pSPIHandle);

	}
	//check for ovr flag

		temp1=pSPIHandle->pSPIx->SR&(1<<6);
		temp2=pSPIHandle->pSPIx->CR2&(1<<5);
		if(temp1&&temp2){
			spi_ovr_err_IThandle(pSPIHandle);

		}



}
void SPI_PRPCLK_EN(SPI_RegDef_t *pSPI,uint8_t enordis){
	if(enordis==1){

		pSPI->CR1|=(enordis<<6);

	}else{
		pSPI->CR1&=~(enordis<<6);

	}



}void SPI2_SSI_Config(SPI_RegDef_t *pSPI,uint8_t enordis){

	if(enordis==1){

			pSPI->CR1|=(enordis<<8);

		}else{
			pSPI->CR1&=~(enordis<<8);

		}


}
void SPI2_SSOE_Config(SPI_RegDef_t *pSPI,uint8_t enordis)
{

	if(enordis==1){

			pSPI->CR2|=(enordis<<2);

		}else{
			pSPI->CR2&=~(enordis<<2);

		}


}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t len){
	uint8_t status=pSPIHandle->TxState;
	if(status!=SPI_BUSY_IN_TX){
		pSPIHandle->pTxBuffer=pTxBuffer;
		pSPIHandle->TxLen=len;
		pSPIHandle->TxState=SPI_BUSY_IN_TX;
		pSPIHandle->pSPIx->CR2|=(1<<7);



	}
	return status;

}
uint8_t  SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t len){

	uint8_t status=pSPIHandle->RxState;
	if(status!=SPI_BUSY_IN_RX){
		pSPIHandle->pRxBuffer=pRxBuffer;
		pSPIHandle->RxLen=len;
		pSPIHandle->RxState=SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->CR2|=(1<<6);//RXNNEIE bit at cr2



	}
	return status;

}



static void spi_rxe_IThandle(SPI_Handle_t *pSPIHandle){

	if(pSPIHandle->pSPIx->CR1&(1<<11)){
			*((uint16_t *)pSPIHandle->pTxBuffer)=pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer--;
			pSPIHandle->pRxBuffer--;
		}else{
			*(pSPIHandle->pTxBuffer)=pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			(uint16_t *)pSPIHandle->pRxBuffer--;


		}
		if(!pSPIHandle->RxLen){
			//txlen 0 ise ınterrupt bitmiştir
			// interrupt kapatmak ilk iş
			void SPI_Close_Reception(pSPIHandle);

			SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}


}

static void spi_txe_IThandle(SPI_Handle_t *pSPIHandle){

	if(pSPIHandle->pSPIx->CR1&(1<<11)){
		pSPIHandle->pSPIx->DR=*((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}else{
		pSPIHandle->pSPIx->DR=*(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;


	}
	if(!pSPIHandle->TxLen){
		//txlen 0 ise ınterrupt bitmiştir
		// interrupt kapatmak ilk iş
		SPI_Close_Transmission(pSPIHandle);

		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}





}
static void spi_ovr_err_IThandle(SPI_Handle_t *pSPIHandle){

	//clear ovr flag
	if(pSPIHandle->TxState!=SPI_BUSY_IN_TX){
		Clear_OVR_Flag(pSPIHandle->pSPIx);
	}
	SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_OVR_ERROR);




}

void SPI_Close_Transmission(SPI_Handle_t *pSPIHandle){
	        pSPIHandle->pSPIx->CR2&=~(1<<7);
			//2. iş bufferları boşaltmak
			pSPIHandle->pTxBuffer=NULL;
			pSPIHandle->TxLen=0;
			pSPIHandle->TxState=SPI_READY;

}
void SPI_Close_Reception(SPI_Handle_t *pSPIHandle){
				pSPIHandle->pSPIx->CR2&=~(1<<6);
				//2. iş bufferları boşaltmak
				pSPIHandle->pRxBuffer=NULL;
				pSPIHandle->RxLen=0;
				pSPIHandle->RxState=SPI_READY;



}

void Clear_OVR_Flag(SPI_RegDef_t *pSPI){

		uint8_t temp;
		temp=pSPI->DR;
		temp=pSPI->SR;


}

__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle,uint8_t spi_event)
{

	// weak implemantation


}






