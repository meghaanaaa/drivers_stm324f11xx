/*
 * 004Spi_Tx_test.c
 *
 *  Created on: Jul 5, 2024
 *      Author: megar
 *      PB14-->MISO
 *      PB15-->MOSI
 *      PB9-->NSS
 *      PB10-->SCK
 */
#include "stm32f411x.h"
#include <string.h>
#include <stdint.h>
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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	///SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_9;
	//GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2_Handle;
	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI2_Handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV2;
	SPI2_Handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI2_Handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2_Handle.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI2_Handle.SPIConfig.SPI_SSM=SPI_SSM_EN;

	SPI_Init(&SPI2_Handle);
}
int main(void)
{
	SPI2_GPIOInits();
	SPI2_Inits();
	//this makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2,ENABLE);
	//enable spi2 perpheral
	SPI_peripheralcontrol(SPI2,ENABLE);
	char user_data[]="Hello World";
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
	//confirm that the spi is not busy
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG)== FLAG_RESET);
	SPI_peripheralcontrol(SPI2,DISABLE);
	while(1);
	return 0;
}
