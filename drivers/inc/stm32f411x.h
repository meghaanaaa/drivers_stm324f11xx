/*
 * stm32411x.h
 *
 *  Created on: Jun 14, 2024
 *      Author: megar
 */

#ifndef STM32F411X_H_
#define STM32F411X_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/****************************************Processor specific details************************
 * ARM CORTEX M4 NVIC REGISTER ADDRESS
 */

#define NVIC_ISER0 				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 				((__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0				((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0XE000E18C)

/*
 * ARM CORTEX M4 Priority register base address
 */
#define NVIC_PR_BASEADDR		((__vo uint32_t*)0XE000E400)

//NO OF PRIORITY BITS IMPLEMENTED IN ST
#define NO_PR_BITS_IMPLEMENTED	4

//// BASE ADDRESSES OF FLASH AND RAM ////

#define FLASH_BASEADDR					0x08000000UL
#define SRAM_BASEADDR					0X20000000UL
#define ROM_BASEADDR					0x1FFF0000UL

//// BASE ADDRESS OF AHB AND APB ////

#define PERIPH_BASEADDR					0X40000000UL
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0X40010000UL
#define AHB1PERIPH_BASEADDR				0X40020000UL
#define AHB2PERIPH_BASEADDR				0x50000000UL

////BASE ADDRESS OF PERIPHERALS HAGING ON AHB1////

#define GPIOA_BASEADDR					0X40020000UL
#define	GPIOB_BASEADDR					0x40020400UL
#define GPIOC_BASEADDR					0x40020800UL
#define GPIOD_BASEADDR					0x40020C00UL
#define GPIOE_BASEADDR					0x40021000UL
#define GPIOH_BASEADDR					0x40021C00UL
#define RCC_BASEADDR 					0x40023800UL


///BASE ADDRESS OF PERIPHERALS HAGING ON APB1////

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR+0X5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR+0X5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR+0X5C00)
#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR+0X3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR+0X3C00)
#define USART2_BASEADDR					(APB1PERIPH_BASEADDR+0X4400)

///BASE ADDRESS OF PERIPHERALS HAGING ON APB2////

#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR+0X3C00)
#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR+0X3000)
#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR+0X3400)
#define SPI5_BASEADDR					(APB2PERIPH_BASEADDR+0X5000)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR+0X3800)
#define USART1_BASEADDR					(APB2PERIPH_BASEADDR+0X1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR+0X1400)

//***********************peripheral specific definition structures********************//

typedef struct{
	__vo uint32_t MODER;		//Defining the mode of the gpio pin(i/p, o/p, AF, analog)
	__vo uint32_t OTYPER;		//output push pull or open drain
	__vo uint32_t OSPEEDR;		//can configure it at low, medium or high output speed
	__vo uint32_t PUPDR;		//enabling pull up or pull down registers
	__vo uint32_t IDR;			//read only bits reading the digital input
	__vo uint32_t ODR;			//can be read and written by the software
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];		//to use it for alternate function has AFH and AFL

}GPIO_RegDef_t;

/*
 * SPI register defination
 */

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;


typedef struct{
	__vo uint32_t CR1; 			//I2 C Control register
	__vo uint32_t CR2;			//I2 C Control register 2
	__vo uint32_t OAR1;			//I2 C Own address register 1
	__vo uint32_t OAR2;			//I2 C Own address register 2
	__vo uint32_t DR;			//2 C Data register
	__vo uint32_t SR1;			//I2 C Status register 1
	__vo uint32_t SR2;			//I2 C Status register 2
	__vo uint32_t CCR;			//I2 C Clock control register (
	__vo uint32_t TRISE;		//I2 C T RISE register
	__vo uint32_t FLTR;			//I 2 C FLTR register

}I2C_RegDef_t;


/*
 * External interrupt control register
 */
typedef struct{
	__vo uint32_t EXTI_IMR;			//Interrupt mask register
	__vo uint32_t EXTI_EMR;			//Event mask register
	__vo uint32_t EXTI_RTSR;		//Rising trigger selection register
	__vo uint32_t EXTI_FTSR;		//Falling trigger selection register
	__vo uint32_t EXTI_SWIER;		//Software interrupt event register
	__vo uint32_t EXTI_PR;			//Pending register

}EXTI_RegDef_t;

/*
 * External interrupt control register
 */
typedef struct{
	__vo uint32_t MEMRMP;		//Interrupt mask register
	__vo uint32_t PMC;			//Event mask register
	__vo uint32_t EXTICR[4];		//Rising trigger selection register
	uint32_t Reserved[2];
	__vo uint32_t SYSCFG_CMPCR;			//Compensation cell control register

}SYSCFG_RegDef_t;

typedef struct{
	__vo uint32_t CR;			//RCC clock control register
	__vo uint32_t PLLCFGR;		//RCC PLL configuration register
	__vo uint32_t CFGR;			//RCC clock configuration register
	__vo uint32_t CIR;			//RCC clock interrupt register
	__vo uint32_t AHB1RSTR;		//RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;		//RCC AHB2 peripheral reset register
	__vo uint32_t Reserved0;			//
	__vo uint32_t Reserved1;
	__vo uint32_t APB1RSTR;		//RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;		//RCC APB2 peripheral reset register
	__vo uint32_t Reserved2;
	__vo uint32_t Reserved3;
	__vo uint32_t AHB1ENR;		//RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;		//RCC AHB2 peripheral clock enable register
	__vo uint32_t Reserved4;
	__vo uint32_t Reserved5;
	__vo uint32_t APB1ENR;		//RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;		//RCC APB2 peripheral clock enable register
	__vo uint32_t Reserved6;
	__vo uint32_t Reserved7;
	__vo uint32_t AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t Reserved8;
	__vo uint32_t Reserved9;
	__vo uint32_t APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;	//RCC APB2 peripheral clock enable in low power mode register
	__vo uint32_t Reserved10;
	__vo uint32_t Reserved11;
	__vo uint32_t BDCR;			//RCC Backup domain control register
	__vo uint32_t CSR;			//RCC clock control & status register
	__vo uint32_t Reserved12;
	__vo uint32_t Reserved13;
	__vo uint32_t SSCGR;		//RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register
	__vo uint32_t Reserved14;
	__vo uint32_t DCKCFGR ;		//RCC Dedicated Clocks Configuration Register
}RCC_RegDef_t;


//// PERIPHERAL DEFINITIONS(peripheral base addresses type casted to xxx_RegDef_t)////
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5		((SPI_RegDef_t*)SPI5_BASEADDR)

#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)


#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

////clock enable for GPIOx peripherals////

#define GPIOA_PCLK_EN()			((RCC->AHB1ENR)|=(1<<0))
#define GPIOB_PCLK_EN()			((RCC->AHB1ENR)|=(1<<1))
#define GPIOC_PCLK_EN()			((RCC->AHB1ENR)|=(1<<2))
#define GPIOD_PCLK_EN()			((RCC->AHB1ENR)|=(1<<3))
#define GPIOE_PCLK_EN()			((RCC->AHB1ENR)|=(1<<4))
#define GPIOH_PCLK_EN()			((RCC->AHB1ENR)|=(1<<7))

////clock enable for I2C peripheral////

#define I2C1_PCLK_EN()				((RCC->APB1ENR)|=(1<<21))
#define I2C2_PCLK_EN()				((RCC->APB1ENR)|=(1<<22))
#define I2C3_PCLK_EN()				((RCC->APB1ENR)|=(1<<23))

////clock enable for SPI peripheral////

#define SPI1_PCLK_EN()				((RCC->APB2ENR)|=(1<<12))
#define SPI2_PCLK_EN()				((RCC->APB1ENR)|=(1<<14))
#define SPI3_PCLK_EN()				((RCC->APB1ENR)|=(1<<15))
#define SPI4_PCLK_EN()				((RCC->APB2ENR)|=(1<<13))
#define SPI5_PCLK_EN()				((RCC->APB2ENR)|=(1<<20))

/////usart clock enable////

#define USART1_PCLK_EN()				((RCC->APB2ENR)|=(1<<4))
#define USART2_PCLK_EN()				((RCC->APB1ENR)|=(1<<17))
#define USART6_PCLK_EN()				((RCC->APB2ENR)|=(1<<5))

////SYSCFG clock enable////

#define SYSCFG_PCLK_EN()				((RCC->APB2ENR)|=(1<<14))

////clock disable for GPIOx peripherals////

#define GPIOA_PCLK_DI()			((RCC->AHB1ENR)&=~(1<<0))
#define GPIOB_PCLK_DI()			((RCC->AHB1ENR)&=~(1<<2))
#define GPIOC_PCLK_DI()			((RCC->AHB1ENR)&=~(1<<3))
#define GPIOD_PCLK_DI()			((RCC->AHB1ENR)&=~(1<<3))
#define GPIOE_PCLK_DI()			((RCC->AHB1ENR)&=~(1<<4))
#define GPIOH_PCLK_DI()			((RCC->AHB1ENR)&=~(1<<7))

////clock disable for I2C peripheral////

#define I2C1_PCLK_DI()				((RCC->APB1ENR)&=~(1<<21))
#define I2C2_PCLK_DI()				((RCC->APB1ENR)&=~(1<<22))
#define I2C3_PCLK_DI()				((RCC->APB1ENR)&=~(1<<23))

////clock disable for SPI peripheral////

#define SPI1_PCLK_DI()				((RCC->APB2ENR)&=~(1<<12))
#define SPI2_PCLK_DI()				((RCC->APB1ENR)&=~(1<<14))
#define SPI3_PCLK_DI()				((RCC->APB1ENR)&=~(1<<15))
#define SPI4_PCLK_DI()				((RCC->APB2ENR)&=~(1<<13))
#define SPI5_PCLK_DI()				((RCC->APB2ENR)&=~(1<<20))

/////usart clock disable////

#define USART1_PCLK_DI()				((RCC->APB2ENR)&=~(1<<4))
#define USART2_PCLK_DI()				((RCC->APB1ENR)&=~(1<<17))
#define USART6_PCLK_DI()				((RCC->APB2ENR)&=~(1<<5))

////SYSCFG clock disable////

#define SYSCFG_PCLK_DI()				((RCC->APB2ENR)&=~(1<<14))

////GPIO REST MACROS////
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &=~(1<<0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &=~(1<<1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &=~(1<<2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &=~(1<<3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &=~(1<<4)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<7)); (RCC->AHB1RSTR &=~(1<<7)); }while(0)

//gpio codes

#define GPIO_BASEADDR_TO_CODE(x)	(	(x==GPIOA)? 0 : \
										(x==GPIOB)? 1 : \
										(x==GPIOC)? 2 : \
										(x==GPIOD)? 3 : \
										(x==GPIOE)? 4 : \
										(x==GPIOH)? 7 :0 	)

/*
 * SPI reset macros
 */
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |=(1<<12)); (RCC->APB2RSTR &=~(1<<12)); }while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<14)); (RCC->APB1RSTR &=~(1<<14)); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<15)); (RCC->APB1RSTR &=~(1<<15)); }while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2RSTR |=(1<<13)); (RCC->APB2RSTR &=~(1<<13)); }while(0)
#define SPI5_REG_RESET()		do{ (RCC->APB2RSTR |=(1<<20)); (RCC->APB2RSTR &=~(1<<20)); }while(0)


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85
#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_I2C2_EV      33
#define IRQ_NO_I2C2_ER      34
#define IRQ_NO_I2C3_EV      72
#define IRQ_NO_I2C3_ER      73

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0 	  0
#define NVIC_IRQ_PRI15    15

//generic macros

#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			0
#define FLAG_SET			1

/********************************************************************************************
 * Bit position definitions of spi peripheral CR1
 ********************************************************************************************/
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15

/********************************************************************************************
 * Bit position definitions of spi peripheral CR2
 ********************************************************************************************/
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/********************************************************************************************
 * Bit position definitions of spi peripheral SR
 ********************************************************************************************/
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRC_ERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


#include "stm32f411x_gpio_driver.h"
#include "stm32f411x_spi_driver.h"
#include "stm32f411x_i2c_driver.h"

#endif /* STM32411X_H_ */
