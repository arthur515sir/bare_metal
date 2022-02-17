/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 18 Oca 2022
 *      Author: MSI
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_
#include "stm32f407xx.h"

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAdress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;



}I2C_Config_t;

typedef struct{

	I2C_RegDef_t *pI2C;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint32_t RxTxState;
	uint32_t DeviceAddr;
	uint32_t RxSize;
	uint32_t Sr;
}I2C_Handle_t;


///speed config macros
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM 400000


#define I2C_ACK_EN 1
#define I2C_ACK_DIS 0

#define I2C_DUTY_CYC_2    0
#define I2C_DUTY_CYC_16_9     1

#define I2C_READY 0
#define I2C_BUSY_TX_IN 1
#define I2C_BUSY_RX_IN  2
// I2C event
#define I2C_EV_RX_CMP     1
#define I2C_EV_TX_CMP     0
#define I2C_EV_STOP       2
////
#define I2C_ER_TIMEOUT 		7
#define I2C_ER_AF			5
#define I2C_ER_ARLO			4
#define I2C_ER_OVR			6
#define I2C_ER_BERR         3
#define I2C_EV_DATA_RCV		9
#define I2C_EV_DATA_REQ		8
//******************************************//



void I2C_PeriCLKCtrl(I2C_RegDef_t *pI2C,uint8_t en_dis);
void I2C_init(I2C_Handle_t *pI2CHandle);
void I2C_Deinit(I2C_RegDef_t *pI2C);


void I2C_Master_Send_Data(I2C_Handle_t *pI2CHandle,uint8_t *pTXBuffer,uint8_t len,uint8_t SlaveAddr);
void I2C_Master_Receive_Data(I2C_Handle_t *pI2CHandle,uint8_t *pRXBuffer,uint8_t len,uint8_t SlaveAddr);



uint8_t I2C_Master_Send_DataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTXBuffer,uint8_t len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_Master_Receive_DataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRXBuffer,uint8_t len,uint8_t SlaveAddr,uint8_t Sr);



void I2C_Slave_Send_Data(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_Slave_Receive_Data(I2C_RegDef_t *pI2C);





void I2C_EV_IRQ_Handling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQ_Handling(I2C_Handle_t *pI2CHandle);

void I2C_Close_RecieveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);




void I2C_PRPCLK_EN(I2C_RegDef_t *pI2C,uint8_t enordis);

void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle,uint8_t I2C_event);


uint32_t RCC_GetPCLKValue(void);
void I2C_Manage_Ack(I2C_RegDef_t *pI2C,uint8_t enable_disable);


//******************************************//

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
