/*
 * stm32f410xx_spi_driver.c
 *
 *  Created on: Feb 9, 2022
 *      Author: Norbi
 */

#include "stm32f410xx_spi_driver.h"
#include "stm32f410xx.h"
#include <stdio.h>


/*
 * Peripheral Clock Setup
 */
/***************************************************************
 * @fn				-	SPI_PeriClockControl
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
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI5)
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
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}

	}
}

/*
 * Init and De-init
 */
/***************************************************************
 * @fn				-	SPI_Init
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << 15);
		//RXONLY bit must be set
		tempreg |= (1 << 10);
	}

	//3. configure the spi serial clock speed( baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

/***************************************************************
 * @fn				-	SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI5)
	{
		SPI5_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data send and receive
 */
/***************************************************************
 * @fn				-	SPI_SendData
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in] 		-
 * @param[in] 		-
 *
 * @return			-	none
 *
 * @note			-	This is a blocking call
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/***************************************************************
 * @fn				-	SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}

/*
 * IRQ Configuration and ISR Handling
 */
/***************************************************************
 * @fn				-	SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/***************************************************************
 * @fn				-	SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/***************************************************************
 * @fn				-	SPI_IRQHandling
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
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}











