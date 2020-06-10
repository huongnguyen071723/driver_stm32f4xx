/*
 * stm32f407xx_gpio_drivers.h
 *
 *  Created on: Jun 3, 2020
 *      Author: huongnguyen
 */

#ifndef STM32F407XX_GPIO_DRIVERS_H_
#define STM32F407XX_GPIO_DRIVERS_H_

#include "stm32f4xx.h"

/**
 * @GPIO_PIN_MODE
 * Definition for GPIO Pin mode
 */
#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/**
 * Definition for GPIO Pin Output Type
 */
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/**
 * Definition for GPIO Output Speed
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/**
 * Definition for GPIO Pin pull up, pull down
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/**
 * @GPIO_PIN_NUMBER
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/**
 * Configuration structure for GPIO pin
 */
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode; /*! @GPIO_PIN_MODE*/
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinCongig_t;

/**
 * Handle structure for GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinCongig_t GPIO_PinConfig;
} GPIO_Handle_t;

/**
 * Init and Deinit GPIO peripheral
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * Configure clock for GPIO
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/**
 * Read data from GPIO pin or GPIO port
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/**
 * Write data to GPIO pin or GPIO port
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/**
 * Toggle GPIO pin
 */
void GPIO_ToggleOutPutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * Configure for Interrupt GPIO
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* STM32F407XX_GPIO_DRIVERS_H_ */
