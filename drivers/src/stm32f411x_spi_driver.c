/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Jul 4, 2024
 *      Author: megar
 */

#include "stm32f411x_spi_driver.h"



static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//configuring SPICR1 register
	uint32_t tempreg=0;

	//configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode<<SPI_CR1_MSTR;

	//2.configure bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//BIDIMODE should be cleared
		tempreg &=~(1<<SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//BIDIMODE should be set
		tempreg |=(1<<SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//BIDIMODE should be cleared
		tempreg &=~(1<<SPI_CR1_BIDI_MODE);
		//RXONLY bit should be set
		tempreg |=(1<<SPI_CR1_RX_ONLY);
	}

	//3. configure the spi serial clockspeed
	tempreg |=pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;

	//4.configure dff
	tempreg |=pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF;

	//5. configure CPOL
	tempreg |=pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL;

	//6. configure CPHA
	tempreg |=pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1=tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){

	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
	else if (pSPIx == SPI5)
	{
		SPI5_REG_RESET();
	}

}

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - this is a blocking call or polling type API

 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len){

	while(Len>0)
	{
		//wait until the TXE flag is 1(1=empty)
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==(uint8_t)FLAG_RESET);

		//check if it is of 16 or 8 bit from dff
		if(pSPIx->CR1 &(1<<SPI_CR1_DFF))//data format is of 16 bit
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else//data format is of 8 bit
		{
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}


	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len){

	while(Len>0)
	{
		//wait until the RXE flag is full (not empty=1)
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)== (uint8_t)FLAG_RESET);

		//check if it is of 16 or 8 bit from dff
		if(pSPIx->CR1 &(1<<SPI_CR1_DFF))//data format is of 16 bit
		{
			//load data from dr to rxbuffer
			*((uint16_t*)pRxBuffer)= pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}
		else//data format is of 8 bit
		{
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}


	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){

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
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){

	//find our ipr register
	uint8_t iprx=IRQNumber/4;
	uint8_t iprxSection=IRQNumber%4;
	uint8_t shiftAmt=(8*iprxSection) + (8-NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + (iprx*4)) |=IRQPriority<<shiftAmt;

}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

	uint8_t temp1, temp2;

	//first check the txe flag
	temp1= pSPIHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2= pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);
	if( temp1 && temp2 )
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//Check for RXNE flag
	temp1= pSPIHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2= pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);
	if( temp1 && temp2 )
	{
		//handle TXE
		spi_rxe_interrupt_handle(pSPIHandle);
	}

	//check for ovr flag
	temp1= pSPIHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2= pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);
	if( temp1 && temp2 )
	{
		//handle TXE
		spi_ovr_interrupt_handle(pSPIHandle);
	}

}
/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             -
 *
 * @param[in]         - pSPIx
 * @param[in]         - EnOrDi
 * @param[in]         -
 *
 * @return            - NONE
 *
 * @Note              -

 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *TxBuffer,uint32_t Len){

	uint8_t state= pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		//1.save Txbuffer address and length info in some global variables
		pSPIHandle->pTxBuffer=TxBuffer;
		pSPIHandle->TxLen=Len;

		//2.Mark the SPI state as busy in transmission so that
		//no other code can take over SPI peripheral until transmission is over
		pSPIHandle->TxState=SPI_BUSY_IN_TX;

		//3.Enable the TXEIE bit whenever TXE flag is set in thE SR
		pSPIHandle->pSPIx->CR2 |=(1<<SPI_CR2_TXEIE);

		//4, Data Transmission will be handled by the ISR CODE
	}

	return state;

}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         - pSPIx
 * @param[in]         - EnOrDi
 * @param[in]         -
 *
 * @return            - NONE
 *
 * @Note              -

 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *RxBuffer,uint32_t Len){

	uint8_t state= pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){
		//1.save rxbuffer address and length info in some global variables
		pSPIHandle->pRxBuffer=RxBuffer;
		pSPIHandle->RxLen=Len;

		//2.Mark the SPI state as busy in reception so that
		//no other code can take over SPI peripheral until transmission is over
		pSPIHandle->RxState=SPI_BUSY_IN_RX;

		//3.Enable the TXEIE bit whenever TXE flag is set in thE SR
		pSPIHandle->pSPIx->CR2 |=(1<<SPI_CR2_RXNEIE);

		//4, Data Transmission will be handled by the ISR CODE
	}

	return state;
}

/*********************************************************************
 * @fn      		  - SPI_peripheralcontrol
 *
 * @brief             -
 *
 * @param[in]         - pSPIx
 * @param[in]         - EnOrDi
 * @param[in]         -
 *
 * @return            - NONE
 *
 * @Note              -

 */

void SPI_peripheralcontrol(SPI_RegDef_t *pSPIx , uint8_t EnOrDi){

	if(EnOrDi==ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         - pSPIx
 * @param[in]         - EnOrDi
 * @param[in]         -
 *
 * @return            - NONE
 *
 * @Note              -

 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx , uint8_t EnOrDi){
	if(EnOrDi==ENABLE)
	{
		pSPIx->CR1 |=(1<<SPI_CR1_SSI);
	}
	else{
		pSPIx->CR1 &=~(1<<SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         - pSPIx
 * @param[in]         - EnOrDi
 * @param[in]         -
 *
 * @return            - NONE
 *
 * @Note              -

 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx , uint8_t EnOrDi){

	if(EnOrDi==ENABLE)
		{
			pSPIx->CR2 |=(1<<SPI_CR2_SSOE);
		}
		else{
			pSPIx->CR2 &=~(1<<SPI_CR2_SSOE);
		}
}

//helper function implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	//check if it is of 16 or 8 bit from dff
	if(pSPIHandle->pSPIx->CR1 &(1<<SPI_CR1_DFF))//data format is of 16 bit
	{
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else//data format is of 8 bit
	{
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(! pSPIHandle->TxLen)
	{
		//TXLen is zero, close the transmission and inform the application that the transmission is over
		//TX is over
		// This prevents interrupt setting of txe flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}


}
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//check if it is of 16 or 8 bit from dff
	if(pSPIHandle->pSPIx->CR1 &(1<<SPI_CR1_DFF))//data format is of 16 bit
	{
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR ;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else//data format is of 8 bit
	{
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen)
	{
		//RXLen is zero, close the Reception and inform the application that the reception is over
		//RX is over
		// This prevents interrupt setting of txe flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;
	//Clear the OVR Flag in SR
	//this is done by read access to SPI_DR followed by read access to SPI_SR register
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp= pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){

	uint8_t temp;
	temp= pSPIx->DR;
	temp =pSPIx->SR;
	(void)temp;

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer= NULL;
	pSPIHandle->TxLen=0;
	pSPIHandle->TxState=SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer= NULL;
	pSPIHandle->RxLen=0;
	pSPIHandle->RxState=SPI_READY;

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent)
{

	//this is a weak implementation and application might override this function
}
