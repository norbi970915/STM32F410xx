/*
 * stm32f4xx.h
 *
 *  Created on: Feb 4, 2022
 *      Author: Norbi
 */

#ifndef INC_STM32F410XX_H_
#define INC_STM32F410XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define ROM_BASEADDR			0x1FFF0000
#define SRAM 					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR			0x40000000
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000
#define AHB1PERIPH_BASEADDR		0x40020000


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR			0x40020000
#define GPIOB_BASEADDR			0x40020400
#define GPIOC_BASEADDR			0x40020800
#define GPIOH_BASEADDR			0x40021C00
#define	RCC_BASEADDR			0x40023800

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


typedef struct
{
	__vo uint32_t CR;				/*RCC clock control register									0x00*/
	__vo uint32_t PLLCFGR;			/*RCC PLL configuration register								0x04*/
	__vo uint32_t CFGR;				/*RCC clock configuration register								0x08*/
	__vo uint32_t CIR;				/*RCC clock interrupt register									0x0C*/
	__vo uint32_t AHB1RSTR;			/*RCC AHB1 peripheral reset register							0x10*/
	uint32_t	  RESERVED0;		/*Reserved, 0x14 - 0x1C*/
	__vo uint32_t APB1RSTR;			/*RCC APB1 peripheral reset register*/
	__vo uint32_t APB2RSTR;			/*RCC APB2 peripheral reset register							0x24*/
	uint32_t	  RESERVED1[2];		/*Reserved, 0x28 - 0x2C*/
	__vo uint32_t AHB1ENR;			/*RCC AHB1 peripheral clock enable register						0x30*/
	uint32_t	  RESERVED2[2];		/*Reserved, 0x34 - 0x3C*/
	__vo uint32_t APB1ENR;			/*RCC APB1 peripheral clock enable register						0x40*/
	__vo uint32_t APB2ENR;			/*RCC APB2 peripheral clock enable register						0x44*/
	uint32_t	  RESERVED3[2];		/*Reserved, 0x48 - 0x4C*/
	__vo uint32_t AHB1LPENR;		/*RCC AHB1 peripheral clock enable in low power mode register	0x50*/
	uint32_t	  RESERVED4[2];		/*Reserved, 0x54 - 0x5C*/
	__vo uint32_t APB1LPENR;		/*RCC APB1 peripheral clock enable in low power mode register	0x60*/
	__vo uint32_t APB2LPENR;		/*RCC APB2 peripheral clock enabled in low power mode register	0x64*/
	uint32_t	  RESERVED5[2];		/*Reserved, 0x68 - 0x6C*/
	__vo uint32_t BCDR;				/*RCC Backup domain control register							0x70*/
	__vo uint32_t CSR;				/*RCC clock control & status register							0x74*/
	uint32_t	  RESERVED6[2];		/*Reserved, 0x78 - 0x7C*/
	__vo uint32_t SSCGR;			/*RCC spread spectrum clock generation register					0x80*/
	uint32_t	  RESERVED7[2];		/*Reserved, 0x84 - 0x88*/
	__vo uint32_t DCKCFGR;			/*RCC Dedicated Clocks Configuration Register					0x8C*/
	uint32_t	  RESERVED8[2];		/*Reserved, 0x90*/
	__vo uint32_t DCKCFGR2;			/*RCC dedicated Clocks Configuration Register 2					0x94*/


}RCC_RegDef_t;



/*
 * peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA 	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)


/* CLOCK ENABLE MACROS*/
/*
 * Clock ENABLE macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))

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
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))

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
#define GPIOH_REG_RESET()	do{	(RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4));	} while(0)


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
