/*
 * stm32f404_spi_drivers.h
 *
 *  Created on: 28 Ara 2021
 *      Author: MSI
 */

#ifndef INC_STM32F407_SPI_DRIVERS_H_
#define INC_STM32F407_SPI_DRIVERS_H_
#include "stm32f407xx.h"
#include <stdint.h>

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_Sclkspeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;
typedef struct{


	SPI_RegDef_t    *pSPIx;
	SPI_Config_t    spi_config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t RxState;
	uint8_t TxState;



}SPI_Handle_t;

//PSSBİLY SPI APP EVENT
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERROR   3
#define SPI_EVENT_CRC_ERR   4

#define SPI_READY     0
#define SPI_BUSY_IN_RX     1
#define SPI_BUSY_IN_TX     2








void SPI_PeriCLKCtrl(SPI_RegDef_t *pSPI,uint8_t en_dis);
void SPI_init(SPI_Handle_t *pSPIHandle);
void SPI_Deinit(SPI_RegDef_t *pSPI);

void SPI_SendData(SPI_RegDef_t *pSPI,uint8_t *pTxBuffer,uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPI,uint8_t *pRxBuffer,uint32_t len);

uint8_t  SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t len);
uint8_t  SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t len);

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);

void SPI_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDis);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);



void Clear_OVR_Flag(SPI_RegDef_t *pSPI);
void SPI_Close_Transmission(SPI_Handle_t *pSPIHandle);
void SPI_Close_Reception(SPI_Handle_t *pSPIHandle);

void SPI_PRPCLK_EN(SPI_RegDef_t *pSPI,uint8_t enordis);
void SPI2_SSI_Config(SPI_RegDef_t *pSPI,uint8_t enordis);
void SPI2_SSOE_Config(SPI_RegDef_t *pSPI,uint8_t enordis);


void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle,uint8_t spi_event);


#define SPI_MODE_MASTER   1
#define SPI_MODE_SLAVE    0


#define SPI_BUSCFG_FD				1
#define SPI_BUSCFG_HD				2
#define SPI_BUSCFG_SIMPLEX_RXONLY   3


#define SPI_CLKCFG_SPEED_DIV2		0
#define SPI_CLKCFG_SPEED_DIV4		1
#define SPI_CLKCFG_SPEED_DIV8		2
#define SPI_CLKCFG_SPEED_DIV16		3
#define SPI_CLKCFG_SPEED_DIV32		4
#define SPI_CLKCFG_SPEED_DIV64		5
#define SPI_CLKCFG_SPEED_DIV128		6
#define SPI_CLKCFG_SPEED_DIV256		7

#define DFF_8BIT		0
#define DFF_16BIT		1

#define CPOL_0			0
#define CPOL_1			1

#define CPHA_FIRST			0
#define CPHA_SECOND			1

#define SSM_EN			1
#define SSM_DIS			0


#endif /* INC_STM32F407_SPI_DRIVERS_H_ */
