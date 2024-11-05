/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Nov 5, 2024
 *      Author: Modugu Sandeep Reddy
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 *
 * This is gpio pin configuration structure
 *
 */
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOType;
	uint8_t GPIO_PinAltFunMode;


}GPIO_PinConfig_t;

/*
 *
 * This is gpio handle structure
 *
 */
typedef struct{
	GPIO_RegDef_t *pGPIOx;  /* This holds the base address of the GPIO to which the pin belongs to*/
	GPIO_PinConfig_t GPIO_PinConfig;  /* This holds the pin configuration  settings */



}GPIO_Handle_t;


/******************************************************************************************************/
/*                                         APIs Supported by this driver                              */
/*                   For more information about APIs check function definitions                       */
/******************************************************************************************************/

/*
 * peripheral clock setup
 */
void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi);
/*
 * Init and de-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
 * Read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * Interrupt configuration and ISR handling Apis
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
