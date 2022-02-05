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
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		// interrupt mode
	}

	temp = 0;
	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	temp = 0;
	//3. Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	temp = 0;
	//4. Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	
	//5. Configure the Alternate Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alternate function registers.
		
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
 * @return			-	none
 *
 * @note			-	none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

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














