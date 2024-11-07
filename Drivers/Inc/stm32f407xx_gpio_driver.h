/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Nov 5, 2024
 *      Author: Modugu Sandeep Reddy
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"
/*@GPIO_PIN_MODES
 * macros of possible modes of a GPIO pin
 */
#define GPIO_MODE_IN      0
#define GPIO_MODE_OUT     1
#define GPIO_MODE_ALTFN   2
#define GPIO_MODE_ANALOG  3
#define GPIO_MODE_IT_FT   4
#define GPIO_MODE_IT_RT   5
#define GPIO_MODE_IT_RFT  6

/*@GPIO_OTYPES
 * macros of possible OTYPES of a GPIO pin
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1
/*@GPIO_OSPEED
 * macros of possible OSPEED of a GPIO pin
 */
#define GPIO_OP_LOW_SPEED     0
#define GPIO_OP_MED_SPEED     1
#define GPIO_OP_HIGH_SPEED    2
#define GPIO_OP_VHIGH_SPEED   3

/*@GPIO_PUPD
 * macros of possible PUPDR of a GPIO pin
 */
#define GPIO_NO_PUPD     0
#define GPIO_PU          1
#define GPIO_PD          2

/*
 * macros of GPIO pin NUMBERS
 */

#define GPIO_PIN_NO_0     0
#define GPIO_PIN_NO_1     1
#define GPIO_PIN_NO_2     2
#define GPIO_PIN_NO_3     3
#define GPIO_PIN_NO_4     4
#define GPIO_PIN_NO_5     5
#define GPIO_PIN_NO_6     6
#define GPIO_PIN_NO_7     7
#define GPIO_PIN_NO_8     8
#define GPIO_PIN_NO_9     9
#define GPIO_PIN_NO_10    10
#define GPIO_PIN_NO_11    11
#define GPIO_PIN_NO_12    12
#define GPIO_PIN_NO_13    13
#define GPIO_PIN_NO_14    14
#define GPIO_PIN_NO_15    15



/*
 *
 * This is gpio pin configuration structure
 *
 */
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;         /* POSSIBLE VALUES FROM @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;        /* POSSIBLE VALUES FROM @GPIO_OSPEED*/
	uint8_t GPIO_PinPuPdControl;  /* POSSIBLE VALUES FROM @GPIO_PUPD*/
	uint8_t GPIO_PinOType;        /* POSSIBLE VALUES FROM @GPIO_OTYPES*/
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
