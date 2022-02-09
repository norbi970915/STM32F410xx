/*
 * stm32f4xx.h
 *
 *  Created on: Feb 4, 2022
 *      Author: Norbi
 */

#ifndef INC_STM32F410XX_H_
#define INC_STM32F410XX_H_

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

#define __vo volatile


/*
 *  ARM cortex M4 processor NVIC ISERx register Addressess
 */

#define NVIC_ISER0		((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1		((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2		((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3		((__vo uint32_t *)0xE000E10C)

/*
 *  ARM cortex M4 processor NVIC ICERx register Addressess
 */

#define NVIC_ICER0		((__vo uint32_t *)0XE000E180)
#define NVIC_ICER1		((__vo uint32_t *)0XE000E184)
#define NVIC_ICER2		((__vo uint32_t *)0XE000E188)
#define NVIC_ICER3		((__vo uint32_t *)0XE000E18C)

/*
 *  ARM cortex M4 processor NVIC priority register base address
 */
#define NVIC_PR_BASEADDR		((__vo uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPELMENTED	4


/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00U)
#define	RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800U)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR			0x40005400
#define I2C2_BASEADDR			0x40005800

#define SPI2_BASEADDR			0x40003800

#define USART2_BASEADDR			0x40004400

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define SPI1_BASEADDR			0x40013000
#define SPI5_BASEADDR			0x40015000

#define USART1_BASEADDR			0x40011000
#define USART6_BASEADDR			0x40011400

#define EXTI_BASEADDR			0x40013C00

#define SYSCFG_BASEADDR			0x40013800


/*************************Peripheral register definition structures********************************/

/*
 * the Peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;			/*GPIO port mode register*/
	__vo uint32_t OTYPER;			/*GPIO port output type register*/
	__vo uint32_t OSPEEDER;			/*GPIO port output speed register*/
	__vo uint32_t PUPDR;			/*GPIO port pull-up/pull-down register*/
	__vo uint32_t IDR;				/*GPIO port input data register*/
	__vo uint32_t ODR;				/*GPIO port output data register*/
	__vo uint32_t BSRR;				/*GPIO port bit set/reset register*/
	__vo uint32_t LCKR;				/*GPIO port configuration lock register*/
	__vo uint32_t AFR[2];			/*GPIO alternate function register  AFR[0] - Low / AFR[1] - High*/

}GPIO_RegDef_t;


/*
 * the Peripheral register definition structure for RCC
 */
typedef struct
{

	__vo uint32_t CR;		 	  /*RCC clock control register  		   				Address offset :0x00 */
	__vo uint32_t PLLCFGR; 	   	  /*RCC PLL configuration register      				Address offset :0x04 */
	__vo uint32_t CFGR;   	 	  /*RCC clock configuration register  					Address offset :0x08 */
	__vo uint32_t CIR;  		  /*RCC clock interrupt register		 				Address offset :0x0C */
	__vo uint32_t AHB1RSTR;    	  /*RCC AHB1 peripheral reset register			   		Address offset :0x10 */
	__vo uint32_t AHB2RSTR;  	  /*RCC AHB2 peripheral reset register  				Address offset :0x14 */
	__vo uint32_t AHB3RSTR;  	  /*RCC AHB3 peripheral reset register					Address offset :0x18 */
		 uint32_t RESERVED1;	  /*Reserved Register									Address offset :0x1C */
	__vo uint32_t APB1RSTR;  	  /*RCC APB1 peripheral reset register 					Address offset :0x20 */
	__vo uint32_t APB2RSTR;  	  /*RCC APB2 peripheral reset register					Address offset :0x24 */
	 	 uint32_t RESERVED2;	  /*Reserved Register									Address offset :0x28 */
		 uint32_t RESERVED3;	  /*Reserved Register									Address offset :0x2C */
	__vo uint32_t AHB1ENR;  	  /*RCC AHB1 peripheral clock enable register			Address offset :0x30 */
	__vo uint32_t AHB2ENR;  	  /*RCC AHB2 peripheral clock enable register			Address offset :0x34 */
	__vo uint32_t AHB3ENR;  	  /*RCC AHB2 peripheral clock enable register			Address offset :0x38 */
	 	 uint32_t RESERVED4;	  /*Reserved Register									Address offset :0x3C */
	__vo uint32_t APB1ENR;  	  /*RCC APB1 peripheral clock enable register			Address offset :0x40 */
	__vo uint32_t APB2ENR;  	  /*RCC APB2 peripheral clock enable register			Address offset :0x44 */
	 	 uint32_t RESERVED5;	  /*Reserved Register									Address offset :0x48 */
	 	 uint32_t RESERVED6;	  /*Reserved Register									Address offset :0x4C */
	__vo uint32_t AHB1LPENR;  	  /*RCC AHB1 peripheral clock enable
	 	 	 	 	 	 	 	 	in low power mode register							Address offset :0x50 */
	__vo uint32_t AHB2LPENR;  	  /*RCC AHB2 peripheral clock enable
									in low power mode register							Address offset :0x54 */
	__vo uint32_t AHB3LPENR;  	  /*RCC AHB3 peripheral clock enable
									in low power mode register							Address offset :0x58 */
	 	 uint32_t RESERVED7;	  /*Reserved Register									Address offset :0x5C */
	__vo uint32_t APB1LPENR;  	  /*RCC APB1 peripheral clock enable
									in low power mode register							Address offset :0x60 */

	__vo uint32_t APB2LPENR;  	  /*RCC APB2 peripheral clock enable
									in low power mode register							Address offset :0x64 */

	 	 uint32_t RESERVED8;	  /*Reserved Register									Address offset :0x68 */
		 uint32_t RESERVED9;	  /*Reserved Register									Address offset :0x6C */
	__vo uint32_t BDCR;  	      /*RCC Backup domain control register				    Address offset :0x70 */
	__vo uint32_t CSR;  	      /*RCC clock control & status register				    Address offset :0x74 */
	 	 uint32_t RESERVED10;	  /*Reserved Register									Address offset :0x78 */
		 uint32_t RESERVED11;	  /*Reserved Register									Address offset :0x7C */
	__vo uint32_t SSCGR;  	      /*RCC spread spectrum clock generation register		Address offset :0x80 */
	__vo uint32_t PLLI2SCFGR;	  /*RCC PLLI2S configuration register				    Address offset :0x84 */
}RCC_RegDef_t;


/*
 * the Peripheral register definition structure for RCC
 */
typedef struct{

	__vo uint32_t IMR; 			/*      Interrupt mask register    		  */
	__vo uint32_t EMR;			/*      Event mask register	    		  */
	__vo uint32_t RTSR;			/*      Rising trigger selection register    */
	__vo uint32_t FTSR;			/*      Falling trigger selection register	  */
	__vo uint32_t SWIER;		/*      Software interrupt event register	  */
	__vo uint32_t PR;			/*      Pending register		    		  */

}EXTI_RegDef_t;

/*
 * the Peripheral register definition structure for SYSCONFIG
 */
typedef struct{

	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CFGR2;
	__vo uint32_t CMPCR;
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;


/*
 * peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA 	((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC		((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI	((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG	((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)


/* CLOCK ENABLE MACROS*/
/*
 * Clock ENABLE macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

/*
 * Clock ENABLE macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))

/*
 * Clock ENABLE macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/*
 * Clock ENABLE macros for USARTx peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock ENABLE macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))


/* CLOCK DISABLE MACROS*/

/*
 * Clock DISABLE macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

/*
 * Clock DISABLE macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))

/*
 * Clock DISABLE macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock DISABLE macros for USARTx peripherals
 */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock DISABLE macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));	} while(0)
#define GPIOB_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1));	} while(0)
#define GPIOC_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2));	} while(0)
#define GPIOH_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7));	} while(0)


/*
 * returns port code for given GPIOx address
 */
#define GPIO_BASEADDR_TO_CODE(x)	((x==GPIOA)?0  :\
									 (x==GPIOB)?1  :\
									 (x==GPIOC)?2  :\
									 (x==GPIOH)?7  :0)


/*
 * Definitions of the IRQ number of interrupt line
 */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40


#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4

/*
 *  Generic macros
 */
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET 			DISABLE
#define	GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET




#endif /* INC_STM32F410XX_H_ */
