/*
 * stm32f410xx_gpio_driver.h
 *
 *  Created on: Feb 5, 2022
 *      Author: Norbi
 */

#ifndef INC_STM32F410XX_GPIO_DRIVER_H_
#define INC_STM32F410XX_GPIO_DRIVER_H_

#include "stm32f410xx.h"

/*
 * GPIO pin configuration structure
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/*@GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode;			/*@GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;			/*@GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl;	/*@GPIO_PUPD_CONTROL*/
	uint8_t GPIO_PinOPType;			/*@GPIO_OUTPUT_TYPE*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/*
 * GPIO Handle structure
 */
typedef struct
{
	//pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;				//This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	//This holds GPIO pin configuration settings

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0	// 	Input (reset state)
#define GPIO_MODE_OUT		1	//	General purpose output mode
#define GPIO_MODE_ALTFN 	2	//	Alternate function mode
#define GPIO_MODE_ANALOG 	3	//	Analog mode
#define GPIO_MODE_IT_FT		4	//	Falling edge
#define GPIO_MODE_IT_RT		5	//	Rising edge
#define GPIO_MODE_IT_RFT	6	//	Rising-falling edge trigger


/*
 * @GPIO_OUTPUT_TYPE
 * GPIO pin possible output type
 */
#define GPIO_OP_TYPE_PP	0	//	Output push-pull (reset state)
#define GPIO_OP_TYPE_OD	1	//	Output open-drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0	//	Low speed
#define GPIO_SPEED_MEDIUM	1	//	Medium speed
#define GPIO_SPEED_FAST		2	//	High speed
#define GPIO_SPEED_HIGH		3	//	Very high speed


/*
 * @GPIO_PUPD_CONTROL
 * GPIO pin pull up and pull down configuration macros
 */
#define	GPIO_NO_PUPD		0	//	No pull-up, pull-down
#define GPIO_PIN_PU			1	//	Pull-up
#define GPIO_PIN_PD			2	// 	Pull-down


/***********************************************************************************************
 * 										APIs supported by this driver
 ***********************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber , uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ COnfiguration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F410XX_GPIO_DRIVER_H_ */
