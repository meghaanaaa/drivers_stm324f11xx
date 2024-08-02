/*
 * 008I2C_masterTxTesting.c
 *
 *  Created on: Jul 18, 2024
 *      Author: megar
 *      PB8-->SCL
 *      PB7-->SDA
 */


#include "stm32f411x.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#define MY_ADDR 0x61

I2C_Handle_t I2C1Handle;
uint8_t SendData[]="TEST DATA";

void delay(void){
	for(uint32_t i=0;i<2500000;i++);
}

void I2C1_GPIOInits()
{
	GPIO_handle_t I2CPins;
	I2CPins.pGPIOX=GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode=4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_8;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}

void GPIO_ButtonInit(void){

	//button initialization
	GPIO_handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOX = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx=I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_EN;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);


}
int main (void)
{
	uint8_t SlaveAddr = 0x68;
	//I2C PIN initialization
	I2C1_GPIOInits();
	//I2C Peripheral initialization
	I2C1_Inits();

	//push button initialization
	GPIO_ButtonInit();

	//enable the peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	while(1)
	{
		//wait for button press
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		delay();

		//send data
		I2C_MasterSendData(&I2C1Handle,SendData,strlen((char*)SendData),SlaveAddr);
	}


}

