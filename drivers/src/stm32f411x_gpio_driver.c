/*
 * stm32f411x_gpio_driver.c
 *
 *  Created on: Jun 17, 2024
 *      Author: megar
 */

#include "stm32f411x_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - to initialize
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_Init(GPIO_handle_t *pGPIOHandle){

	/*configure the mode of gpio pin*/
	uint32_t temp=0;

	//Enable the pheripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOX,ENABLE);

	//if the pin mode is not an interrupt mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){

		temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOX->MODER &=~(3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOX->MODER |=temp;
		temp=0;
	}

	else{
		//interrupt modes
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT){
			//1.configure the FTSR (Falling trigger Selection Register)
			EXTI->EXTI_FTSR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding rtsr bit
			EXTI->EXTI_RTSR &=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_IT_RT){
			//1.configure the RTSR (Rising trigger Selection Register)
			EXTI->EXTI_RTSR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding ftrs bit
			EXTI->EXTI_FTSR &=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_IT_RFT){
			//1.configure both the FTSR and RTSR
			//configure the FTSR (Falling trigger Selection Register)
			EXTI->EXTI_FTSR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//configure the RTSR (Rising trigger Selection Register)
			EXTI->EXTI_RTSR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2. configure the gpio port selecton in SYSCFG_EXTICR
		uint8_t ExtiReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t shifts=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portCode=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOX);
		SYSCFG_PCLK_EN();

		//clear
		SYSCFG->EXTICR[ExtiReg]&=~(0xF<<(shifts*4));
		//set
		SYSCFG->EXTICR[ExtiReg]|=portCode<<(shifts*4);

		//3.enable the interrupt delivery using IMR
		EXTI->EXTI_IMR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//temp=0;
	//configure the speed
	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOX->OSPEEDR &=~(3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOX->OSPEEDR |=temp;

	//configure the pull up pull down settings
	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOX->PUPDR &=~(3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOX->PUPDR |=temp;
	//temp=0;
	//configure the optype
	temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOX->OTYPER &=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOX->OTYPER |=temp;

	//configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN){
		uint8_t temp1;
		uint8_t temp2;
		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOX->AFR[temp1] &= ~(0xF<<(4*temp2));
		pGPIOHandle->pGPIOX->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));

		}


}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}


/*********************************************************************
 * @fn      		  - GPIO_PeriClockContro
 *
 * @brief             - to enable or disable clock for GPIO ports
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - specify ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - to read input from pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pin number
 * @param[in]         -
 *
 * @return            - 0 or 1
 *
 * @Note              - none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR>>PinNumber)&0x00000001);
	return value;

}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - to read input from port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - 0b0000000000000000 - 0x1111111111111111
 *
 * @Note              - none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - to write to a pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pin number
 * @param[in]         - value to be written(0/1)
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |=(1<<PinNumber);
	}
	else{
		pGPIOx->ODR &=~(1<<PinNumber);
	}

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - to write to a port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - value to be written(8 bit value)
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value)
{
	pGPIOx->ODR |=(Value);

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^=(1<<PinNumber);

}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - to configure interrupt of interrupt
 *
 * @param[in]         - IRQNumber
 * @param[in]         - enable or disable interrupt
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
//IRQ configuration and isr handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - to configure priority of interrupt
 *
 * @param[in]         - IRQNumber
 * @param[in]         - IRQPriority
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//find our ipr register
	uint8_t iprx=IRQNumber/4;
	uint8_t iprxSection=IRQNumber%4;
	uint8_t shiftAmt=(8*iprxSection) + (8-NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + (iprx*4)) |=IRQPriority<<shiftAmt;

}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - to configure HANDEL INERRUPTS
 *
 * @param[in]         - PinNumber
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - GPIO_IRQHandling comes into the picture now, this API is used in
 * 						ISR which you override in your c file. it takes one argument
 * 						as pinNumber which is the pin from which you expect an interrupt
 * 						to occur.
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register coressponding to the pin number
	//if the interrupt is pended clear it
	if(EXTI->EXTI_PR & (1<<PinNumber))
	{
		//clear the pending interrupt by writing 1
		EXTI->EXTI_PR |=1<<PinNumber;
	}

}
