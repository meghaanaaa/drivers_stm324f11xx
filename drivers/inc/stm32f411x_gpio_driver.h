/*
 * stm32f411x_gpio_driver.h
 *
 *  Created on: Jun 17, 2024
 *      Author: megar
 */

#ifndef INC_STM32F411X_GPIO_DRIVER_H_
#define INC_STM32F411X_GPIO_DRIVER_H_

#include "stm32f411x.h"



typedef struct{
	uint8_t GPIO_PinNumber;			/*!< possible values from @GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;	/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOX;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_handle_t;


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN     2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4           //INPUT FALLING EDGE
#define GPIO_MODE_IT_RT		5			//INPUT RISING EDGE
#define GPIO_MODE_IT_RFT	6			//INPUT RISING AND FALLING EDGE


/*
 * GPIO pin possible Outputs
 */
#define GPIO_OP_TYPE_PP		0		//PUSHPULL TYPE OUTPUT
#define GPIO_OP_TYPE_OD		1		//OPEN DRAIN TYPE OUTPUT

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3

/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 *  EXTI PORTS DEF
 *  0000: PA[x] pin
0001: PB[x] pin
0010: PC[x] pin
0011: PD[x] pin
0100: PE[x] pin
0101: Reserved
0110: Reserved
0111
 */
#define PA	0000
#define PB	0001
#define PC	0010
#define PD	0011
#define PE	0100
#define PH	0111




/************************************************APIs supported by this driver************************************************/

//initializing and de-initializing
void GPIO_Init(GPIO_handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//periphral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

//data read write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

//IRQ configuration and isr handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F411X_GPIO_DRIVER_H_ */
