/*
 * 003button_interrupt.c
 *
 *  Created on: Jun 21, 2024
 *      Author: megar
 */

#include "stm32f411x.h"
#include <stdint.h>
#include <string.h>
void delay(void){
	for(uint32_t i=0;i<250000;i++);
}

int main(void)
{
	GPIO_handle_t GpioLed;
	memset(&GpioLed,0,sizeof(GpioLed));
	GpioLed.pGPIOX=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	GPIO_handle_t GpioButton;
	memset (&GpioButton,0,sizeof(GpioButton));
	//GPIO_PeriClockControl(GPIOD,ENABLE);
	GpioButton.pGPIOX=GPIOD;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GpioButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;

	GPIO_Init(&GpioButton);

	//ENABLE IRQ
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);

	while(1);

	return 0;
}
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_7);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}

