/*
 * stm32f410xx_gpio_driver.c
 *
 *  Created on: Feb 5, 2022
 *      Author: Norbi
 */

#include "stm32f410xx_gpio_driver.h"
#include <stdio.h>


/*
 * Peripheral Clock Setup
 */
/***************************************************************
 * @fn				-	GPIO_PeriClockControl
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in] 		-	ENABLE or DISABLE MACROS
 * @param[in] 		-
 *
 * @return			-	none
 *
 * @note			-	none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
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
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
/***************************************************************
 * @fn				-	GPIO_Init
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in] 		-
 * @param[in] 		-
 *
 * @return			-	none
 *
 * @note			-	none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//1. Configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
		temp = 0;

	}
	else
	{
		// interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			//1. configure falling trigger register FTRS
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			//1. configure the rising trigger register RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			//1. configure both FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		 uint8_t temp1 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber /4;
		 uint8_t temp2 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber %4;
		 uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle ->pGPIOx);
		 SYSCFG_PCLK_EN();
		 SYSCFG->EXTICR[temp1] = (portcode << (4 * temp2));

		//3. enable the EXTI interrupt delivery using IMR(interrupt mask register)
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

	}

	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDER |= temp; //setting
	temp = 0;

	//3. Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	//5. Configure the Alternate Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alternate function registers.
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/***************************************************************
 * @fn				-	GPIO_DeInit
 *
 * @brief			-
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in] 		-
 * @param[in] 		-
 *
 * @return			-	none
 *
 * @note			-	none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data read and write
 */
/***************************************************************
 * @fn				-	GPIO_ReadFromInputPin
 *
 * @brief			-
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in] 		-	Address of the selected pin
 * @param[in] 		-
 *
 * @return			-	0 or 1
 *
 * @note			-	none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint16_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 );
	return value;

	/*uint16_t value =0;
	value =  ((pGPIOx->IDR));
	return value ;*/
}

/***************************************************************
 * @fn				-	GPIO_ReadFromInputPort
 *
 * @brief			-
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in] 		-
 * @param[in] 		-
 *
 * @return			-
 *
 * @note			-	none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/***************************************************************
 * @fn				-	GPIO_WriteToOutputPin
 *
 * @brief			-
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in] 		-
 * @param[in] 		-
 *
 * @return			-
 *
 * @note			-	none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber , uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}

/***************************************************************
 * @fn				-	GPIO_WriteToOutputPort
 *
 * @brief			-
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in] 		-
 * @param[in] 		-
 *
 * @return			-	none
 *
 * @note			-	none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/***************************************************************
 * @fn				-	GPIO_WriteToOutputPin
 *
 * @brief			-
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in] 		-
 * @param[in] 		-
 *
 * @return			-	none
 *
 * @note			-	none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx -> ODR  ^= (1<<PinNumber);
}

/*
 *  IRQ Configuration and Handling
 */

/************************************************************************
 * @Function name 			- GPIO_IRQNumberConfig
 *
 * @brief					- This Function Enables or Disables the GPIOx Clk
 *
 * @param[in]				- base address of GPIOx
 * @param[in]				- the State of given peripheral
 * @param[in]				-
 *
 *
 * @return					- none
 *
 *
 * @note					- none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |=1<<IRQNumber;
			}else if(IRQNumber >31 && IRQNumber < 64 )
			{
				// program ISER1 register
				*NVIC_ISER1 |= (1<<(IRQNumber %32));
			}else if(IRQNumber >= 64 && IRQNumber <96)
			{
				// Program ISER2 register
				*NVIC_ISER2 |= (1<<(IRQNumber %64));
			}

		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |=1<<IRQNumber;
			}else if(IRQNumber >31 && IRQNumber < 64 )
			{
				// program ICER1 register
				*NVIC_ICER1 |= (1<<(IRQNumber %32));
			}else if(IRQNumber >= 64 && IRQNumber <96)
			{
				// Program ICER2 register
				*NVIC_ISER2 |= (1<<(IRQNumber %64));
			}
		}
}

/************************************************************************
 * @Function name 			- GPIO_IRQPriorityConfig
 *
 * @brief					- This Function is for Set the priority
 *
 * @param[in]				- IRQ priority number
 * @param[in]				- the State of given peripheral
 * @param[in]				-
 *
 *
 * @return					- none
 *
 *
 * @note					- none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (iprx_section * 8) + (8 - NO_PR_BITS_IMPELMENTED);
    *(NVIC_PR_BASEADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}


/************************************************************************
 * @Function name 			-GPIO_IRQHandling
 *
 * @brief					-
 *
 * @param[in]				-
 * @param[in]				-
 * @param[in]				-
 *
 *
 * @return					- none
 *
 *
 * @note					- none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the EXTI pending register
	if((EXTI ->PR) & (1 << PinNumber))
	{
		EXTI->PR |= 1 << PinNumber;
	}
}
