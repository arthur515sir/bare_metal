/*
 * stm32f407_uart_driver.h
 *
 *  Created on: 31 Oca 2022
 *      Author: MSI
 */

#ifndef INC_STM32F407_UART_DRIVER_H_
#define INC_STM32F407_UART_DRIVER_H_
#include "stm32f407xx.h"
#include <stdint.h>

typedef struct{
	uint8_t USART_Mode;
	uint32_t USART_baudrate;
	uint8_t	USART_NoOfStopBits;
	uint8_t USART_Wordlenght;
	uint8_t USART_ParityControl;
	uint8_t	USART_HWFlowControl;


}USART_Config_t;



typedef struct{

	USART_Config_t USART_Config;
	USART_RegDef_t *pUSARTx;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;

}USART_Handle_t;
/// USART MODE
#define USART_MODE_ONLY_TX 		0
#define USART_MODE_ONLY_RX 		1
#define USART_MODE_ONLY_RXTX 	2
// USART BAUDRATE
#define USART_STD_BAUD_1200	 		1200
#define USART_STD_BAUD_2400	 		2400
#define USART_STD_BAUD_9600	 		9600
#define USART_STD_BAUD_19200		19200
#define USART_STD_BAUD_38400	 	38400
#define USART_STD_BAUD_57600 		57600
#define USART_STD_BAUD_115200	 	115200
#define USART_STD_BAUD_230400	 	230400
#define USART_STD_BAUD_460800	 	460800
#define USART_STD_BAUD_921600		921600
#define USART_STD_BAUD_2M			2000000
#define USART_STD_BAUD_3M			3000000
/// usart parity
#define USART_PARITY_EN_ODD			0
#define USART_PARITY_EN_EVEN		1
#define USART_PARITY_EN_DISABLE		2
// WORDLENGHT
#define USART_WORDLEN_8			0
#define USART_WORDLEN_9			1
// USART STOPBİTS
#define USART_STOPBITS_0_5			1
#define USART_STOPBITS_1			0
#define USART_STOPBITS_1_5			3
#define USART_STOPBITS_2			2


// USART HW FLOWCOTROL
#define USART_HW_FLOW_CTRL_NONE			0
#define USART_HW_FLOW_CTRL_RTS			1
#define USART_HW_FLOW_CTRL_CTS			2
#define USART_HW_FLOW_CTRL_RTS_CTS		3
///
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0



#define USART_ERREVENT_ORE      8
#define USART_ERREVENT_NE		7
#define USART_ERREVENT_FE				6
#define USART_EVENT_ORE			5
#define USART_EVENT_RX_CMPLT    1
#define USART_EVENT_TX_CMPLT 	2
#define USART_EVENT_CTS			3
#define USART_EVENT_IDLE		4
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);


void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

void USART_SendData(USART_Handle_t *pUSARTHangle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHangle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);


void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);

uint32_t RCC_GetPLLOutputClock(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value();













#endif /* INC_STM32F407_UART_DRIVER_H_ */
