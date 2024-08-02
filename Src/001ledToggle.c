/*
 * 001ledToggle.c
 *
 *  Created on: Jun 19, 2024
 *      Author: megar
 */

#include "stm32f411x.h"
#include <stdint.h>

void delay(void){
	for(uint32_t i=0;i<25;i++);
}

int main(void)
{
	GPIO_handle_t GpioLed;
	GpioLed.pGPIOX=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);
	while(1){
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		delay();

	}
	return 0;
}
