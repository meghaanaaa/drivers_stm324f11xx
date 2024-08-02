/*
 * 002LedButton.c
 *
 *  Created on: Jun 19, 2024
 *      Author: megar
 */

#include "stm32f411x.h"
#include <stdint.h>

void delay(void){
	for(uint32_t i=0;i<250000;i++);
}

int main(void)
{
	GPIO_handle_t GpioLed;
	GpioLed.pGPIOX=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	GPIO_handle_t GpioButton;
	GPIO_PeriClockControl(GPIOB,ENABLE);
	GpioButton.pGPIOX=GPIOB;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_Init(&GpioButton);


	while(1){
		uint8_t buttonState;
		buttonState=GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_12);
		if(buttonState==0){
			//GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,GPIO_PIN_SET);
			delay();		//wait until switch debouncing stops
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		}
//		else{
//			GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,GPIO_PIN_RESET);
//		}


	}
	return 0;
}
