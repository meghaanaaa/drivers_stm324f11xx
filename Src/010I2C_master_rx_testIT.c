/*
 * 010I2C_master_rx_testIT.c
 *
 *  Created on: Jul 29, 2024
 *      Author: megar
 */


/*
 * 009I2C_MasterRxdataCmd.c
 *
 *  Created on: Jul 23, 2024
 *      Author: megar
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f411x.h"

#define MY_ADDR 0x61
I2C_Handle_t I2C1Handle;

void delay(void){
	for(uint32_t i=0;i<2500000;i++);
}

void GPIO_ButtonInit(void)
{
	GPIO_handle_t GPIOBtn;
	GPIOBtn.pGPIOX=GPIOA;

	GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	GPIO_Init(&GPIOBtn);
}

void I2C_GPIOInit(void)
{
	GPIO_handle_t I2CPins;
	I2CPins.pGPIOX=GPIOB;

	I2CPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode=4;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//SCL PIN
	I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_8;
	GPIO_Init(&I2CPins);

	//SDA PIN
	I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}

void I2C1_Init(void)
{


	I2C1Handle.pI2Cx=I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_EN;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}
int main()
{

	uint8_t SlaveAddr = 0x68;
	uint8_t cmdLen=0x51;
	uint8_t cmdData=0x52;
	uint8_t RcvLen,RcvData[32];


	//initialize the input onboard user button
	GPIO_ButtonInit();

	//initialize the I2C peripheral gpio
	I2C_GPIOInit();

	//initialize the I2C peripheral
	I2C1_Init();

	//I2C IRQ CONFIGURATION
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//enable acking
	I2C1->CR1 |=(1<<I2C_CR1_ACK);

	while(1)
	{

		delay();
		//wait for button to be pushed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		//to avoid switch debouncing
		delay();

		//send the first command to recevie the length of data slave is sending
		while(I2C_MasterSendDataIT(&I2C1Handle,&cmdLen,1,SlaveAddr,I2C_ENABLE_SR) != I2C_READY);

		//receive the length info from the slave
		while(I2C_MasterReceiveDataIT(&I2C1Handle,&RcvLen,1,SlaveAddr,I2C_ENABLE_SR)!= I2C_READY);

		//send the second command to receive the data from the slave is sending
		while(I2C_MasterSendDataIT(&I2C1Handle,&cmdData,1,SlaveAddr,I2C_ENABLE_SR)!= I2C_READY);

		//store the received data in the buffer
		while(I2C_MasterReceiveDataIT(&I2C1Handle,RcvData,RcvLen,SlaveAddr,I2C_DISABLE_SR)!= I2C_READY);

		RcvData[RcvLen+1]='\0';

		//delay();

		//printf("%s\n",RcvData);


	}



	return 0;
}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	if(AppEv == I2C_EV_TX_CMPLT)
	 {
		 printf("Tx is completed\n");
	 }else if (AppEv == I2C_EV_RX_CMPLT)
	 {
		 printf("Rx is completed\n");
		 //rxComplt = SET;
	 }else if (AppEv == I2C_ERROR_AF)
	 {
		 printf("Error : Ack failure\n");
		 //in master ack failure happens when slave fails to send ack for the byte
		 //sent from the master.
		 I2C_CloseSendData(pI2CHandle);

		 //generate the stop condition to release the bus
		 I2C_GenerateStopCondition(I2C1);

		 //Hang in infinite loop
		 while(1);
	 }
}
