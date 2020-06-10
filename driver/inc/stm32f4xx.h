/*
 * stm32f4xx.h
 *
 *  Created on: Jun 1, 2020
 *      Author: huongnguyen
 */

#ifndef STM32F4XX_H_
#define STM32F4XX_H_
#include <stdint.h>

/**
 * some defines useful
 */
#define __vo 							volatile
#define NULL							(void *)0
#define null							NULL
#define DISABLE 						0
#define ENABLE 							1
#define SET								ENABLE
#define RESET							DISABLE
#define GPIO_PIN_SET					ENABLE
#define GPIO_PIN_RESET					DISABLE

/**
 * base address of flash and SRAM memories
 */
#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U
#define SRAM 							SRAM1_BASEADDR

/**
 * AHBx and APB bus peripheral base address
 */
#define PERIPH_BASE						0x40000000U
#define APB1PERIPH_BASE					PERIPH_BASE
#define APB2PERIPH_BASE					0x40010000U
#define AHB1PERIPH_BASE					0x40020000U
#define AHB2PERIPH_BASE					0x50000000U
#define AHB3PERIPH_BASE					0xA0000000U

/**
 * GPIOx base addresses on AHB1 bus
 */
#define GPIOA_BASEADDR					(AHB1PERIPH_BASE + 0x00U)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR					(AHB1PERIPH_BASE + 0x2000U)
#define RCC_BASEADDR					(AHB1PERIPH_BASE + 0x3800U)

/**
 * base address of peripherals on APB1
 */
#define I2C1_BASEADDR 					(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR 					(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR 					(APB1PERIPH_BASE + 0x5C00U)

#define SPI2_BASEADDR					(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR					(APB1PERIPH_BASE + 0x3C00U)
#define USART2_BASEADDR					(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR					(APB1PERIPH_BASE + 0x4800U)
#define UART4_BASEADDR					(APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASEADDR					(APB1PERIPH_BASE + 0x5000U)

#define CAN1_BASEADDR					(APB1PERIPH_BASE + 0x6400U)
#define CAN2_BASEADDR					(APB1PERIPH_BASE + 0x6800U)

/**
 * base address of peripheral on APB2
 */
#define EXTI_BASEADDR					(APB2PERIPH_BASE + 0x3C00U)
#define SPI1_BASEADDR					(APB2PERIPH_BASE + 0x3000U)
#define SPI4_BASEADDR					(APB2PERIPH_BASE + 0x3400U)

/**
 * definition structure for GPIO register
 */
typedef struct {
	__vo uint32_t MODER; /**< GPIO port mode register, Address offset: 0x00*/
	__vo uint32_t OTYPER; /**< GPIO port output type register, Address offset: 0x04*/
	__vo uint32_t OSPEEDR; /**< GPIO port output speed register, Address offset: 0x08*/
	__vo uint32_t PUPDR; /**< GPIO port pull-up/pull-down register, Address offset: 0x0C*/
	__vo uint32_t IDR; /**< GPIO port input data register, Address offset: 0x10*/
	__vo uint32_t ODR; /**< GPIO port output data register, Address offset: 0x14*/
	__vo uint32_t BSRR; /**< GPIO port bit set/reset register, Address offset: 0x18*/
	__vo uint32_t LCKR; /**< GPIO port configuration lock register, Address offset: 0x1C*/
	__vo uint32_t AFR[2]; /**< GPIO alternate function register*/
} GPIO_RegDef_t;

/**
 * definition structure for RCC register
 */
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED6;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED7;
	uint32_t RESERVED8;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED9;
	uint32_t RESERVED10;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
} RCC_RegDef_t;

/**
 *GPIOx definition (convert register address to GPIO_RegDef_t)
 */
#define GPIOA 							((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC 							((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD 							((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE 							((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF 							((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG 							((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH 							((GPIO_RegDef_t *) GPIOH_BASEADDR)
#define GPIOI 							((GPIO_RegDef_t *) GPIOI_BASEADDR)

/**
 * RCC definition (convert register address to RCC_RegDef_t)
 */
#define RCC 							((RCC_RegDef_t *) RCC_BASEADDR)

/**
 * Enable clock for GPIOx peripheral
 */
#define GPIOA_PERI_CLK_ENABLE() 		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PERI_CLK_ENABLE() 		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PERI_CLK_ENABLE() 		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PERI_CLK_ENABLE() 		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PERI_CLK_ENABLE() 		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PERI_CLK_ENABLE() 		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PERI_CLK_ENABLE() 		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PERI_CLK_ENABLE() 		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PERI_CLK_ENABLE() 		(RCC->AHB1ENR |= (1 << 8))

/**
 * Disable clock for GPIOx peripheral
 */
#define GPIOA_PERI_CLK_DISABLE() 		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PERI_CLK_DISABLE() 		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PERI_CLK_DISABLE() 		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PERI_CLK_DISABLE() 		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PERI_CLK_DISABLE() 		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PERI_CLK_DISABLE() 		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PERI_CLK_DISABLE() 		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PERI_CLK_DISABLE() 		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PERI_CLK_DISABLE() 		(RCC->AHB1ENR &= ~(1 << 8))

/**
 * Reset GPIOx peripherals
 */
#define GPIOA_REG_RESET() { RCC->AHB1RSTR |= (1<<0); RCC->AHB1RSTR &= ~(1<<0);}
#define GPIOB_REG_RESET() { RCC->AHB1RSTR |= (1<<1); RCC->AHB1RSTR &= ~(1<<1);}
#define GPIOC_REG_RESET() { RCC->AHB1RSTR |= (1<<2); RCC->AHB1RSTR &= ~(1<<2);}
#define GPIOD_REG_RESET() { RCC->AHB1RSTR |= (1<<3); RCC->AHB1RSTR &= ~(1<<3);}
#define GPIOE_REG_RESET() { RCC->AHB1RSTR |= (1<<4); RCC->AHB1RSTR &= ~(1<<4);}
#define GPIOF_REG_RESET() { RCC->AHB1RSTR |= (1<<5); RCC->AHB1RSTR &= ~(1<<5);}
#define GPIOG_REG_RESET() { RCC->AHB1RSTR |= (1<<6); RCC->AHB1RSTR &= ~(1<<6);}
#define GPIOH_REG_RESET() { RCC->AHB1RSTR |= (1<<7); RCC->AHB1RSTR &= ~(1<<7);}
#define GPIOI_REG_RESET() { RCC->AHB1RSTR |= (1<<8); RCC->AHB1RSTR &= ~(1<<8);}

/**
 * Enable clock for I2Cx peripheral
 */
#define I2C1_PERI_CLK_ENABLE()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PERI_CLK_ENABLE()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PERI_CLK_ENABLE()			(RCC->APB1ENR |= (1 << 23))

/**
 * Disable clock for I2Cx peripheral
 */
#define I2C1_PERI_CLK_DISABLE()			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PERI_CLK_DISABLE()			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PERI_CLK_DISABLE()			(RCC->APB1ENR &= ~(1 << 23))

/**
 * Enable clock for SPIx peripheral
 */
#define SPI2_PERI_CLK_ENABLE()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PERI_CLK_ENABLE()			(RCC->APB1ENR |= (1 << 15))

/**
 * Disable clock for SPIx peripheral
 */
#define SPI2_PERI_CLK_DISABLE()			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PERI_CLK_DISABLE()			(RCC->APB1ENR &= ~(1 << 15))

/**
 * Enable clock to SYSCFG peripheral
 */

#endif /* STM32F4XX_H_ */
