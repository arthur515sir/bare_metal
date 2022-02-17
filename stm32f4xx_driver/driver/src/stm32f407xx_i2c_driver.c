/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 18 Oca 2022
 *      Author: MSI
 */

#include "stm32f407xx_i2c_driver.h"
#include <stdint.h>


static void I2C_Generate_Start_Con(I2C_RegDef_t *pI2C);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2C,uint8_t slaveaddr);
static void I2C_ExecuteAddressPhase_read(I2C_RegDef_t *pI2C,uint8_t slaveaddr);

static void Clear_AddrFlag(I2C_Handle_t *pI2CHandle);
static void Generate_stop_con(I2C_RegDef_t *pI2C);




void I2C_PeriCLKCtrl(I2C_RegDef_t *pI2C,uint8_t en_dis){
	if(en_dis == ENABLE){
			if(pI2C==I2C1){
				I2C1_CLK_EN();

			}if(pI2C==I2C2){
				I2C2_CLK_EN();

			}if(pI2C==I2C3){
				I2C3_CLK_EN();

			}

		}else if(en_dis==DISABLE){
			if(pI2C==I2C1){
				I2C1_CLK_DIS();

			}if(pI2C==I2C2){
				I2C2_CLK_DIS();

			}if(pI2C==I2C3){
				I2C3_CLK_DIS();

					}
		}



}

void I2C_Deinit(I2C_RegDef_t *pI2C){
	if(pI2C==I2C1){
			I2C1_CLK_RST();

		}if(pI2C==I2C2){
			I2C2_CLK_RST();

		}if(pI2C==I2C3){
			I2C3_CLK_RST();

			}

}
void I2C_init(I2C_Handle_t *pI2CHandle){
	uint32_t temp=0;
	temp|=(pI2CHandle->I2C_Config.I2C_ACKControl<<10);
	pI2CHandle->pI2C->CR1|=temp;
	temp=0;
	temp=RCC_GetPCLKValue()/1000000U;
	pI2CHandle->pI2C->CR2|=(temp & 0x3F);
	temp=0;
	temp=(pI2CHandle->I2C_Config.I2C_DeviceAdress<<1);
	temp|=(1<<14);
	pI2CHandle->pI2C->OAR1|=temp;
	//ccr calculation
	uint16_t ccr_value=0;
	temp=0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed  <= I2C_SCL_SPEED_SM){
		//Mode is standart mode
		ccr_value=(RCC_GetPCLKValue()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		temp=(ccr_value&0xFFF);
	}else{
		// mode is fast mode
		temp|=(1<<15);
		temp|=(pI2CHandle->I2C_Config.I2C_FMDutyCycle<<14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle==I2C_DUTY_CYC_2){
			///duty cycle 2 calculation
			ccr_value=(RCC_GetPCLKValue()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else{
			///duty cycle 16 9 calculation
			ccr_value=(RCC_GetPCLKValue()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		temp=(ccr_value&0xFFF);


	}
	pI2CHandle->pI2C->CCR|=temp;
	///trıse


	uint8_t trise=0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		trise=1+(RCC_GetPCLKValue()/1000000U);
		// MODE İS STANDART MODE

	}else
	{
		trise=((RCC_GetPCLKValue()*300)/1000000000U)+1;

		// mode is fast mode
	}
	pI2CHandle->pI2C->TRISE=(trise & 0x3F);

}
uint32_t AHB_Prescaler[8]={2,4,8,16,32,64,128,256};
uint32_t APB1_Prescaler[4]={2,4,8,16};

uint32_t RCC_GetPCLKValue(void){
	uint32_t pclk1=0;
	uint32_t CLKSRC=0;
	uint32_t systemclk=0;
	CLKSRC=((RCC->CFGR>>2)&0x3);

	if(CLKSRC==0){
		systemclk=16000000;

	}else if(CLKSRC==1){
		systemclk=8000000;

	}
	uint32_t pres=0;
	uint32_t temp=0;
	temp=((RCC->CFGR>>4)&0xF);
	if(temp<8){
		pres=1;
	}else{
		pres=AHB_Prescaler[temp-8];

	}
	//for apb1
	uint32_t apb1pres=0;
	temp=((RCC->CFGR>>10)&0x8);
		if(temp<4){
			apb1pres=1;
		}else{
			apb1pres=APB1_Prescaler[temp-4];

		}

		pclk1=(systemclk/(pres))/( apb1pres);
	return pclk1;



}
void I2C_Master_Send_Data(I2C_Handle_t *pI2CHandle,uint8_t *pTXBuffer,uint8_t len,uint8_t SlaveAddr){
	I2C_Generate_Start_Con(pI2CHandle->pI2C);

	// wait untill sb is cleared
	while(!((pI2CHandle->pI2C->SR1&0x1)==1));


	I2C_ExecuteAddressPhase(pI2CHandle->pI2C,0xF);
	while(((pI2CHandle->pI2C->SR1 >> 1)&0x1) ==1);
	Clear_AddrFlag(pI2CHandle);

	while(len>0)
	{
		while(((pI2CHandle->pI2C->SR1>>7)&0x1)==1);
		pI2CHandle->pI2C->DR=*pTXBuffer;
		len--;
	}

	while(((pI2CHandle->pI2C->SR1>>7)&0x1)==1);// wait until txe set

	while(((pI2CHandle->pI2C->SR1>>2)&0x1)==1); /// wait untill btf set


	Generate_stop_con(pI2CHandle->pI2C);



}
void I2C_Master_Receive_Data(I2C_Handle_t *pI2CHandle,uint8_t *pRXBuffer,uint8_t len,uint8_t SlaveAddr){

	I2C_Generate_Start_Con(pI2CHandle->pI2C);
	//wait untill sb cleared
	while(!((pI2CHandle->pI2C->SR1&0x1)==1));
	I2C_ExecuteAddressPhase_read(pI2CHandle->pI2C,0xF);

	while(((pI2CHandle->pI2C->SR1 >> 1)&0x1) ==1);




	if(len==1){
		// disable ack for send nack at end


		I2C_Manage_Ack(pI2CHandle->pI2C,DISABLE);
		//generating stop con


		/// clearing addr
		Clear_AddrFlag(pI2CHandle);
		while(((pI2CHandle->pI2C->SR1>>6)&0x1)==1);// wait until rxne set
		Generate_stop_con(pI2CHandle->pI2C);

		*pRXBuffer=pI2CHandle->pI2C->DR;




	}else if(len>1){
		// clear addr flag
		Clear_AddrFlag(pI2CHandle);
		// read data untill len 0
		uint32_t i;
		for(i=len;i>0;i--){
			// wait untill rxne beconmes 1
			while(((pI2CHandle->pI2C->SR1>>6)&0x1)==1);// wait until rxne set
			if(i==2){
				// son 2 data packeti demek

				// ack bit clear
				I2C_Manage_Ack(pI2CHandle->pI2C,DISABLE);
				// generate stop condition
				 Generate_stop_con(pI2CHandle->pI2C);

			}
			// read the data from redigter in to buffer
			*pRXBuffer=pI2CHandle->pI2C->DR;
			// increment buffer adress
			pRXBuffer++;


		}



	}


// reenable ack
I2C_Manage_Ack(pI2CHandle->pI2C,ENABLE);


}




static void I2C_Generate_Start_Con(I2C_RegDef_t *pI2C){

	pI2C->CR1|=(1<<8);// generates start with start bit in cr1



}
void I2C_Manage_Ack(I2C_RegDef_t *pI2C,uint8_t enable_disable){
	if(enable_disable==ENABLE){
		pI2C->CR1|=1<<10;
	}else if(enable_disable==DISABLE){
		pI2C->CR1&=~(1<<10);
	}


}

static void I2C_ExecuteAddressPhase_read(I2C_RegDef_t *pI2C,uint8_t slaveaddr){

	slaveaddr=(slaveaddr<<1);
	slaveaddr|=1;
	pI2C->DR=slaveaddr;
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2C,uint8_t slaveaddr){
	slaveaddr=(slaveaddr<<1);/// makin space for r/nw bit
	slaveaddr&=~(1);  /// adding r/nw bit as 0
	pI2C->DR=slaveaddr;

}
static void Clear_AddrFlag(I2C_Handle_t *pI2CHandle){
	uint8_t dummy;
	if(pI2CHandle->pI2C->SR2 & (1<<0)){

		//device is in master mood

		if(pI2CHandle->RxTxState==I2C_BUSY_RX_IN){
			if(pI2CHandle->RxLen==1){
				// disable the ack
				I2C_Manage_Ack(pI2CHandle->pI2C, DISABLE);
				// clear addr flag
				dummy=pI2CHandle->pI2C->SR1;
				dummy=pI2CHandle->pI2C->SR2;
				(void)dummy;
			}


		}else{

			dummy=pI2CHandle->pI2C->SR1;
			dummy=pI2CHandle->pI2C->SR2;
			(void)dummy;

		}
	}else{
		dummy=pI2CHandle->pI2C->SR1;
		dummy=pI2CHandle->pI2C->SR2;
		(void)dummy;
		// device is in slave mode


	}



}
static void Generate_stop_con(I2C_RegDef_t *pI2C){
	pI2C->CR1|=(1<<9);

}



uint8_t I2C_Master_Send_DataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTXBuffer,uint8_t len,uint8_t SlaveAddr,uint8_t Sr){
	uint8_t busystate = pI2CHandle->RxTxState;

		if( (busystate != I2C_BUSY_TX_IN) && (busystate != I2C_BUSY_RX_IN))
		{
			pI2CHandle->pTxBuffer = pTXBuffer;
			pI2CHandle->TxLen = len;
			pI2CHandle->RxTxState = busystate;
			pI2CHandle->DeviceAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_Generate_Start_Con(pI2CHandle->pI2C);


			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2C->CR2 |= ( 1 << 10);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2C->CR2 |= ( 1 << 9);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2C->CR2 |= ( 1 << 8);


		}

		return busystate;


}
uint8_t I2C_Master_Receive_DataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRXBuffer,uint8_t len,uint8_t SlaveAddr,uint8_t Sr){

	uint8_t busystate = pI2CHandle->RxTxState;

	if( (busystate != I2C_BUSY_TX_IN) && (busystate != I2C_BUSY_RX_IN))
	{
		pI2CHandle->pRxBuffer = pRXBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->RxTxState = busystate;
		pI2CHandle->RxSize = len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DeviceAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
			I2C_Generate_Start_Con(pI2CHandle->pI2C);

		//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2C->CR2 |= ( 1 << 10);


		//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2C->CR2 |= ( 1 << 9);


		//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2C->CR2 |= ( 1 << 8);

	}

	return busystate;


}
void I2C_EV_IRQ_Handling(I2C_Handle_t *pI2CHandle){
	uint8_t temp1,temp2,temp3;
	temp1=(pI2CHandle->pI2C->CR2&(1<<9));
	temp2=(pI2CHandle->pI2C->CR2&(1<<8));
	temp3=(pI2CHandle->pI2C->SR1&(1<<0));// sb bit ddeğeri
	(void)temp2;// silebilriz sanırım
	if(temp1&&temp3)
	{
		// sr bit set inerruptı

		if(pI2CHandle->RxTxState==I2C_BUSY_TX_IN)
		{
			I2C_ExecuteAddressPhase(pI2CHandle->pI2C, pI2CHandle->DeviceAddr);

		}else if(pI2CHandle->RxTxState==I2C_BUSY_RX_IN){
			I2C_ExecuteAddressPhase_read(pI2CHandle->pI2C, pI2CHandle->DeviceAddr);

		}




	}


	temp3=(pI2CHandle->pI2C->SR1&(1<<1));// addr bit control
	if(temp1&&temp3)
	{
			// addr bit set inerruptı
		pI2CHandle->pI2C->SR1&=~(1<<1);// Clear addr flag

	}





	temp3=(pI2CHandle->pI2C->SR1&(1<<2));/// btf control
	if(temp1&&temp3)
	{
		// btf bit set inerruptı
		if(pI2CHandle->RxTxState==I2C_BUSY_TX_IN){
			// check fot txe set
			if(pI2CHandle->pI2C->SR1 & (1<<7)){
				//btf txe bth set
				//close com condition
				// generate stop reset all members d
				if(pI2CHandle->TxLen==0){
					if(pI2CHandle->Sr== ENABLE){

										Generate_stop_con(pI2CHandle->pI2C);
									}

									I2C_CloseSendData(pI2CHandle);
									// notifying user
									I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CMP);

				}


			}

		}else if(pI2CHandle->RxTxState==I2C_BUSY_RX_IN){



		}



	}


	temp3=(pI2CHandle->pI2C->SR1&(1<<4));/// stopf control
	if(temp1&&temp3)
	{

				//clearing stop bit read sr1 & write to cr1
				// already readed
				// stop bit set inerruptı
		pI2CHandle->pI2C->CR1|=(0x00000);
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP);

	}

	uint8_t itebuf;
	itebuf=pI2CHandle->pI2C->CR1&(1<<10);

	temp3=pI2CHandle->pI2C->SR1&(1<<7);// txe event
	if(itebuf&&temp1&temp3)
	{
		if(pI2CHandle->pI2C->SR2 & (0x1)){/// slave master ayrımı

			if(pI2CHandle->RxTxState==I2C_BUSY_TX_IN){
						if(pI2CHandle->TxLen>0){
							/// dr boş oraya bilgi akıtıcaz
							// load data to dr
							// decrement the txlen
							// increment the txbuffer
							pI2CHandle->pI2C->DR=*(pI2CHandle->pTxBuffer);
							pI2CHandle->TxLen--;
							pI2CHandle->pTxBuffer++;
						}
					}
		}else{
			if(pI2CHandle->pI2C->SR1&(1<<2)){
				// transciever mode
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_REQ);
			}


		}
		//txe bit set

	}
	temp3=pI2CHandle->pI2C->SR1&(1<<6);
	if(itebuf&&temp1&&temp3)
	{

		if(pI2CHandle->pI2C->SR2&(1<<0)){
		// master mod
			if(pI2CHandle->RxTxState==I2C_BUSY_RX_IN){
			// busy in rx bakıcaz sonuçta gönderme biti


			// recieve data benzeri
				if(pI2CHandle->RxLen==1){

					*(pI2CHandle->pRxBuffer)=pI2CHandle->pI2C->DR;
					pI2CHandle->RxSize--;

			}


				if(pI2CHandle->RxLen>1){

					if(pI2CHandle->RxLen==2){
						I2C_Manage_Ack(pI2CHandle->pI2C,DISABLE);
					}
				*(pI2CHandle->pRxBuffer)=pI2CHandle->pI2C->DR;
				pI2CHandle->pRxBuffer++;
				pI2CHandle->RxSize--;

				}else if (pI2CHandle->RxLen==0){


				//close communication and notify user
				// generate close condition
				// close ı2c rx
				Generate_stop_con(pI2CHandle->pI2C);
				I2C_Close_RecieveData(pI2CHandle);// burda kapatırken bütün ınterttuptları kapatıcaz
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_RX_CMP);


			}
		}
			//rxne bit set

	}else{
		if(!pI2CHandle->pI2C->SR1&(1<<2)){
			// transciever mode
			I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_RCV);
					}


	}

	}

}
void I2C_ER_IRQ_Handling(I2C_Handle_t *pI2CHandle){
	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2C->CR2) & ( 1 << 8);

/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2C->SR1) & ( 1<< 8);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2C->SR1 &= ~( 1 << 8);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2C->SR1) & ( 1 << 9 );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2C->SR1 &= ~( 1 << 9);
		//Implement the code to notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2C->SR1) & ( 1 << 10);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2C->SR1 &= ~( 1 << 10);

		//Implement the code to notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_AF);

	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2C->SR1) & ( 1 << 11);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2C->SR1 &= ~( 1 << 11);

		//Implement the code to notify the application about the error
		 I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_OVR);

	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2C->SR1) & ( 1 << 14);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2C->SR1 &= ~( 1 << 14);

		//Implement the code to notify the application about the error
		 I2C_ApplicationEventCallback(pI2CHandle,I2C_ER_TIMEOUT);

	}



}

void I2C_Close_RecieveData(I2C_Handle_t *pI2CHandle){
// must disable all ırq
	pI2CHandle->pI2C->CR2&=~(1<<10);
	pI2CHandle->pI2C->CR2&=~(1<<9);
	pI2CHandle->RxLen=0;
	pI2CHandle->pRxBuffer=NULL;
	pI2CHandle->RxSize=0;
	pI2CHandle->RxTxState=I2C_READY;
	if(pI2CHandle->I2C_Config.I2C_ACKControl==ENABLE){
		I2C_Manage_Ack(pI2CHandle->pI2C,ENABLE);
	}



}
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){

	pI2CHandle->pI2C->CR2&=~(1<<10);
	pI2CHandle->pI2C->CR2&=~(1<<9);
	pI2CHandle->RxTxState=I2C_READY;
	pI2CHandle->pTxBuffer=NULL;
	pI2CHandle->TxLen=0;


}
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;
	uint8_t shift_amount=(8*iprx_section)+(8-no_of_bits_implemented);
	*(IPR0_Base_addr+(4*iprx))|=((IRQPriority)<<shift_amount);



}

void I2C_IRQITConfig(uint8_t IRQNumber,uint8_t EnOrDis){
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


void I2C_Slave_Send_Data(I2C_RegDef_t *pI2C,uint8_t data){

	pI2C->DR=data;

}
uint8_t I2C_Slave_Receive_Data(I2C_RegDef_t *pI2C){


	return (uint8_t) pI2C->DR;
}











