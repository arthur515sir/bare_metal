/*
 * stm32f407_uart_driver.c
 *
 *  Created on: 31 Oca 2022
 *      Author: MSI
 */

#include "stm32f407_uart_driver.h"

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
			if(pUSARTx==USART1){

				USART1_CLK_EN();
			}if(pUSARTx==USART2){
				USART2_CLK_EN();

			}if(pUSARTx==USART3){
				USART3_CLK_EN();

			}if(pUSARTx==USART6){
				USART6_CLK_EN();

			}if(pUSARTx==UART4){
				UART4_CLK_EN() ;

			}if(pUSARTx==UART5){
				UART5_CLK_EN();

			}if(pUSARTx==UART7){
				UART7_CLK_EN();

			}if(pUSARTx==UART8){
				UART8_CLK_EN();
			}


		}else if(EnorDi==DISABLE){
			if(pUSARTx==USART1){

				USART1_CLK_DIS();
			}if(pUSARTx==USART2){
				USART2_CLK_DIS();

			}if(pUSARTx==USART3){
				USART3_CLK_DIS();

			}if(pUSARTx==USART6){
				USART6_CLK_DIS();

			}if(pUSARTx==UART4){
				UART4_CLK_DIS() ;

			}if(pUSARTx==UART5){
				UART5_CLK_DIS();

			}if(pUSARTx==UART7){
				UART7_CLK_DIS();

			}if(pUSARTx==UART8){
				UART8_CLK_DIS();
			}


		}





}


void USART_Init(USART_Handle_t *pUSARTHandle){
	//Temporary variable
		uint32_t tempreg=0;

	/******************************** Configuration of CR1******************************************/

		//Implement the code to enable the Clock for given USART peripheral
		USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

		//Enable USART Tx and Rx engines according to the USART_Mode configuration item
		if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
		{
			//Implement the code to enable the Receiver bit field
			tempreg|= (1 << 2);
		}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
		{
			//Implement the code to enable the Transmitter bit field
			tempreg |= ( 1 << 3 );

		}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RXTX)
		{
			//Implement the code to enable the both Transmitter and Receiver bit fields
			tempreg |= ( ( 1 << 2) | ( 1 << 3) );
		}

	    //Implement the code to configure the Word length configuration item
		tempreg |= pUSARTHandle->USART_Config.USART_Wordlenght << 12 ;


	    //Configuration of parity control bit fields
		if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
		{
			//Implement the code to enale the parity control
			tempreg |= ( 1 << 10);

			//Implement the code to enable EVEN parity
			//Not required because by default EVEN parity will be selected once you enable the parity control

		}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
		{
			//Implement the code to enable the parity control
		    tempreg |= ( 1 << 10);

		    //Implement the code to enable ODD parity
		    tempreg |= ( 1 << 10);

		}

	   //Program the CR1 register
		pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

		tempreg=0;

		//Implement the code to configure the number of stop bits inserted during USART frame transmission
		tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << 12;

		//Program the CR2 register
		pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

		tempreg=0;

		//Configuration of USART hardware flow control
		if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
		{
			//Implement the code to enable CTS flow control
			tempreg |= ( 1 << 9);


		}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
		{
			//Implement the code to enable RTS flow control
			tempreg |= (1<<8);

		}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS_CTS)
		{
			//Implement the code to enable both CTS and RTS Flow control
			tempreg |= ( 1 << 8)|( 1 << 9);
		}


		pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

		//Implement the code to configure the baud rate
		//We will cover this in the lecture. No action required here


}
void USART_DeInit(USART_RegDef_t *pUSARTx){


}

void USART_SendData(USART_Handle_t *pUSARTHangle,uint8_t *pTxBuffer, uint32_t Len){
	uint16_t *pdata;
	   //Loop over until "Len" number of bytes are transferred
		for(uint32_t i = 0 ; i < Len; i++)
		{
			//Implement the code to wait until TXE flag is set in the SR
			while(! pUSARTHangle->pUSARTx->SR>>7&(0x1));


	         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
			if(pUSARTHangle->USART_Config.USART_Wordlenght== USART_WORDLEN_9)
			{
				//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
				pdata = (uint16_t*) pTxBuffer;
				pUSARTHangle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

				//check for USART_ParityControl
				if(pUSARTHangle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
				{
					//No parity is used in this transfer. so, 9bits of user data will be sent
					//Implement the code to increment pTxBuffer twice
					pTxBuffer++;
					pTxBuffer++;
				}
				else
				{
					//Parity bit is used in this transfer . so , 8bits of user data will be sent
					//The 9th bit will be replaced by parity bit by the hardware
					pTxBuffer++;
				}
			}
			else
			{
				//This is 8bit data transfer
				pUSARTHangle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

				//Implement the code to increment the buffer address
				pTxBuffer++;
			}
		}

		//Implement the code to wait till TC flag is set in the SR
		while( ! (pUSARTHangle->pUSARTx->SR & (1<<6) ));


}
void USART_ReceiveData(USART_Handle_t *pUSARTHangle, uint8_t *pRxBuffer, uint32_t Len){
	 //Loop over until "Len" number of bytes are transferred
		for(uint32_t i = 0 ; i < Len; i++)
		{
			//Implement the code to wait until RXNE flag is set in the SR
			while(!(pUSARTHangle->pUSARTx->SR&(1<<5))){


			}

			//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
			if(pUSARTHangle->USART_Config.USART_Wordlenght == USART_WORDLEN_9)
			{
				//We are going to receive 9bit data in a frame

				//check are we using USART_ParityControl control or not
				if(pUSARTHangle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
				{
					//No parity is used. so, all 9bits will be of user data

					//read only first 9 bits. so, mask the DR with 0x01FF
					(*((uint16_t*) pRxBuffer)) = (pUSARTHangle->pUSARTx->DR  & (uint16_t)0x1FF);

					//Now increment the pRxBuffer two times
					pRxBuffer++;
					pRxBuffer++;
				}
				else
				{
					//Parity is used, so, 8bits will be of user data and 1 bit is parity
					 *pRxBuffer = (pUSARTHangle->pUSARTx->DR  & (uint8_t)0xFF);

					 //Increment the pRxBuffer
					//TODO
				}
			}
			else
			{
				//We are going to receive 8bit data in a frame

				//check are we using USART_ParityControl control or not
				if(pUSARTHangle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
				{
					//No parity is used , so all 8bits will be of user data

					//read 8 bits from DR
					 *pRxBuffer = pUSARTHangle->pUSARTx->DR;
				}

				else
				{
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity

					//read only 7 bits , hence mask the DR with 0X7F
					 *pRxBuffer = pUSARTHangle->pUSARTx->DR & (uint8_t)0x7F;

				}

				//increment the pRxBuffer
				pRxBuffer++;
			}
		}



}
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
		uint8_t txstate = pUSARTHandle->TxBusyState;

		if(txstate != USART_BUSY_IN_TX)
		{
			pUSARTHandle->TxLen = Len;
			pUSARTHandle->pTxBuffer = pTxBuffer;
			pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

			//Implement the code to enable interrupt for TXE
			pUSARTHandle->pUSARTx->CR1|=(1<<5);


			//Implement the code to enable interrupt for TC
			pUSARTHandle->pUSARTx->CR1|=(1<<6);



		}

		return txstate;


}
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t rxstate = pUSARTHandle->RxBusyState;

		if(rxstate != USART_BUSY_IN_RX)
		{
			pUSARTHandle->RxLen = Len;
			pUSARTHandle->pRxBuffer = pRxBuffer;
			pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

			//Implement the code to enable interrupt for RXNE
			pUSARTHandle->pUSARTx->CR1|=(1<<5);

		}

		return rxstate;


}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi==ENABLE){
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;
	uint8_t shift_amount=(8*iprx_section)+(8-no_of_bits_implemented);
	*(IPR0_Base_addr+(4*iprx))|=((IRQPriority)<<shift_amount);



}
void USART_IRQHandling(USART_Handle_t *pHandle){

	uint32_t temp1 , temp2, temp3;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pHandle->pUSARTx->SR & ( 1 << 6);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pHandle->pUSARTx->CR1 & ( 1 << 6);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pHandle-> TxLen )
			{
				//Implement the code to clear the TC flag
				pHandle->pUSARTx->SR &= ~( 1 << 6);

				//Implement the code to clear the TCIE control bit
				pHandle->pUSARTx->CR1 &=~ (1<<6);
				//Reset the application state
				pHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pHandle->pTxBuffer=NULL;

				//Reset the length to zero
				pHandle->TxLen=0;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pHandle->pUSARTx->SR & ( 1 << 7);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = temp1 = pHandle->pUSARTx->CR1 & ( 1 << 7);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pHandle->USART_Config.USART_Wordlenght == USART_WORDLEN_9)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					uint16_t *pdata;
					 pdata = (uint16_t*) pHandle->pTxBuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pHandle->pTxBuffer++;
						pHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pHandle->TxLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pHandle->pUSARTx->DR = (*pHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pHandle->pTxBuffer++;

					//Implement the code to decrement the length
					pHandle->TxLen-=1;
				}

			}
			if (pHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pHandle->pUSARTx->CR1 &= ~( 1 << 7);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pHandle->pUSARTx->SR & ( 1 << 5);
	temp2 = pHandle->pUSARTx->CR1 & ( 1 << 7);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		//this interrupt is because of txe
		if(pHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			//TXE is set so send data
			if(pHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pHandle->USART_Config.USART_Wordlenght == USART_WORDLEN_9)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pHandle->pRxBuffer) = (pHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pHandle->pRxBuffer++;
						pHandle->pRxBuffer++;

						//Implement the code to decrement the length
						pHandle->RxLen-=1;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *pHandle->pRxBuffer = (pHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 pHandle->pRxBuffer++;

						 //Implement the code to decrement the length
						 pHandle->RxLen-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pHandle->pRxBuffer = (uint8_t) (pHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pHandle->pRxBuffer = (uint8_t) (pHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pHandle->pRxBuffer++;

					//Implement the code to decrement the length
				}


			}//if of >0

			if(! pHandle->RxLen)
			{
				//disable the rxne
				pHandle->pUSARTx->CR1 &= ~( 1 << 5 );
				pHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1=pHandle->pUSARTx->SR&(1<<9);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pHandle->pUSARTx->CR3 & ( 1 << 9);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pHandle->pUSARTx->CR3 & ( 1 << 10);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		pHandle->pUSARTx->SR&=~(1<<9);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1=pHandle->pUSARTx->SR&(1<<4);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pHandle->pUSARTx->CR3 & ( 1 << 4);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pHandle->pUSARTx->SR & (1<<3);

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pHandle->pUSARTx->CR1 & (1<<5);


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		pHandle->pUSARTx->SR&=~(1<<3);
		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pHandle,USART_EVENT_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pHandle->pUSARTx->CR3 & ( 1 << 0) ;

	if(temp2 )
	{
		temp1 = pHandle->pUSARTx->SR;
		if(temp1 & ( 1 << 1))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pHandle,USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << 1) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pHandle,USART_ERREVENT_NE);
		}

		if(temp1 & ( 1 << 3) )
		{
			USART_ApplicationEventCallback(pHandle,USART_ERREVENT_ORE);
		}
	}



}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){



}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv){



}
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){
	//Variable to hold the APB clock
		uint32_t PCLKx;

		uint32_t usartdiv;

		//variables to hold Mantissa and Fraction values
		uint32_t M_part,F_part;

	  uint32_t tempreg=0;

	  //Get the value of APB bus clock in to the variable PCLKx
	  if(pUSARTx == USART1 || pUSARTx == USART6)
	  {
		   //USART1 and USART6 are hanging on APB2 bus
		   PCLKx = RCC_GetPCLK2Value();
	  }else
	  {
		   PCLKx = RCC_GetPCLK1Value();
	  }

	  //Check for OVER8 configuration bit
	  if(pUSARTx->CR1 & (1 << 15))
	  {
		   //OVER8 = 1 , over sampling by 8
		   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	  }else
	  {
		   //over sampling by 16
		  usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	  }

	  //Calculate the Mantissa part
	  M_part = usartdiv/100;

	  //Place the Mantissa part in appropriate bit position . refer USART_BRR
	  tempreg |= M_part << 4;

	  //Extract the fraction part
	  F_part = (usartdiv - (M_part * 100));

	  //Calculate the final fractional
	  if(pUSARTx->CR1 & ( 1 << 15))
	   {
		  //OVER8 = 1 , over sampling by 8
		  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

	   }else
	   {
		   //over sampling by 16
		   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	   }

	  //Place the fractional part in appropriate bit position . refer USART_BRR
	  tempreg |= F_part;

	  //copy the value of tempreg in to BRR register
	  pUSARTx->BRR = tempreg;




}
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};
uint32_t RCC_GetPCLK2Value(){
	uint32_t pclk1,SystemClk;

		uint8_t clksrc,temp,ahbp,apb1p;

		clksrc = ((RCC->CFGR >> 2) & 0x3);

		if(clksrc == 0 )
		{
			SystemClk = 16000000;
		}else if(clksrc == 1)
		{
			SystemClk = 8000000;
		}else if (clksrc == 2)
		{
			SystemClk = RCC_GetPLLOutputClock();
		}

		//for ahb
		temp = ((RCC->CFGR >> 4 ) & 0xF);

		if(temp < 8)
		{
			ahbp = 1;
		}else
		{
			ahbp = AHB_PreScaler[temp-8];
		}



		//apb1
		temp = ((RCC->CFGR >> 10 ) & 0x7);

		if(temp < 4)
		{
			apb1p = 1;
		}else
		{
			apb1p = APB1_PreScaler[temp-4];
		}

		pclk1 =  (SystemClk / ahbp) /apb1p;

		return pclk1;
}
uint32_t RCC_GetPCLK1Value(void){
	uint32_t SystemClock=0,tmp,pclk2;
		uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

		uint8_t ahbp,apb2p;

		if(clk_src == 0)
		{
			SystemClock = 16000000;
		}else
		{
			SystemClock = 8000000;
		}
		tmp = (RCC->CFGR >> 4 ) & 0xF;

		if(tmp < 0x08)
		{
			ahbp = 1;
		}else
		{
	       ahbp = AHB_PreScaler[tmp-8];
		}

		tmp = (RCC->CFGR >> 13 ) & 0x7;
		if(tmp < 0x04)
		{
			apb2p = 1;
		}else
		{
			apb2p = APB1_PreScaler[tmp-4];
		}

		pclk2 = (SystemClock / ahbp )/ apb2p;

		return pclk2;


}
uint32_t RCC_GetPLLOutputClock(void)
{

	return 0;
}

