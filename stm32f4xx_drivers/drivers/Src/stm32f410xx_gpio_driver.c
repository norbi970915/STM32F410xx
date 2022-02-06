/*
 * stm32f410xx_gpio_driver.c
 *
 *  Created on: Feb 5, 2022
 *      Author: Norbi
 */

#include "stm32f410xx_gpio_driver.h"


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
	}
	else
	{
		// interrupt mode
	}

	temp = 0;
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
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 );
	return value;
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

}

/*
 * IRQ COnfiguration and ISR handling
 */
/***************************************************************
 * @fn				-	GPIO_IRQConfig
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
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/***************************************************************
 * @fn				-	GPIO_IRQHandling
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
void GPIO_IRQHandling(uint8_t PinNumber)
{

}














