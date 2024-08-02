/*
 * 006SpiCmdHandling.c
 *
 *  Created on: Jul 10, 2024
 *      Author: megar
 */


/*
 * 005SpiArduinoTxOly.c
 *
 *  Created on: Jul 8, 2024
 *      Author: megar
 *      PB14-->MISO
 *      PB15-->MOSI
 *      PB9-->NSS
 *      PB10-->SCK
 *      PB12-->button
 */


#include "stm32f411x.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

//arduino led pin
#define LED_PIN 9

uint8_t SPI_Verifyresponse(uint8_t ackbyte);

void delay(void){
	for(uint32_t i=0;i<2500000;i++);
}

void SPI2_GPIOInits()
{
	GPIO_handle_t SPIPins;
	SPIPins.pGPIOX=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_10;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_9;
	GPIO_Init(&SPIPins);


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

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2_Handle;
	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI2_Handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV8;//2MHz clock
	SPI2_Handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI2_Handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2_Handle.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI2_Handle.SPIConfig.SPI_SSM=SPI_SSM_DI;//Hardware Slave select management

	SPI_Init(&SPI2_Handle);
}
int main(void)
{
	uint8_t dummyWrite = 0xff;
	uint8_t dummyRead;

	GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Inits();
	//Eabling SPI SSOE BIT
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1){

		//wait for the button to be pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		//delay t avoid switch debouncing
		delay();

		//enable spi2 perpheral
		SPI_peripheralcontrol(SPI2,ENABLE);

		//1. CMD_LED_CTRL <pin no(1)>  <value(1)>
		uint8_t commandCode=COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command code
		SPI_SendData(SPI2,&commandCode,1);

		//delay();
		//do a dummy read to clear RXNE
		SPI_ReceiveData(SPI2,&dummyRead,1);

		//2.send 1 byte of dummy byte to fetch response from slave
		SPI_SendData(SPI2,&dummyWrite,1);

		//delay();
		//receive the ack byte data
		SPI_ReceiveData(SPI2,&ackbyte,1);

		//check if you have received ACK or NACK
		if(SPI_Verifyresponse(ackbyte)){
			//send other arguments i.e., <pin no(1)>  <value(1)>
			args[0]=LED_PIN;
			args[1]=LED_OFF;
			SPI_SendData(SPI2,args,2);

			//delay();
			// dummy read
			//SPI_ReceiveData(SPI2,args,2);
		}
		//end of  COMMAND_LEDCTRL

		//2.CMD_SENSOR_READ  <analog pin number(1)>

		//wait for the button to be pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		//delay t avoid switch debouncing
		delay();

		commandCode=COMMAND_SENSOR_READ;

		//send command code
		SPI_SendData(SPI2,&commandCode,1);

		//do a dummy read to clear RXNE
		SPI_ReceiveData(SPI2,&dummyRead,1);

		//2.send 1 byte of dummy byte to fetch response from slave
		SPI_SendData(SPI2,&dummyWrite,1);

		//receive the ack byte data
		SPI_ReceiveData(SPI2,&ackbyte,1);

		//check if you have received ACK or NACK
		if(SPI_Verifyresponse(ackbyte)){
			//send other arguments i.e., <pin no(1)>  <value(1)>
			args[0]=ANALOG_PIN0;
			SPI_SendData(SPI2,args,1);

			//do a dummy read to clear RXNE
			SPI_ReceiveData(SPI2,&dummyRead,1);

			//insert some delay for arduino to respond
			delay();

			//2.send 1 byte of dummy byte to fetch response from slave
			SPI_SendData(SPI2,&dummyWrite,1);

			//receive the data
			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);

			printf("%d \n",analog_read);

			//confirm that the spi is not busy
			while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG)== FLAG_RESET);
			SPI_peripheralcontrol(SPI2,DISABLE);

		}



	}

	return 0;
}

uint8_t SPI_Verifyresponse(uint8_t ackbyte)
{
	if(ackbyte ==0xF5)
	{
		//ack
		return 1;
	}
		return 0;

}
