/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Nov 5, 2024
 *      Author: Modugu Sandeep Reddy
 */

#include "stm32f407xx_gpio_driver.h"

/**********************************************************************************************************
 * @fn : GPIO_ClockControl
 *
 * @brief : This function Enable or Disable GPIO peripheral clock
 *
 * @param[1]: Base address of GPIO port
 * @param[2]: Enable or Disable macros
 * @param[3]:
 *
 * @return : none
 *
 * @Note:
 **********************************************************************************************************/
void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_CLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_CLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_CLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_CLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_CLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_CLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_CLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_CLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_CLK_EN();
		}

	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_CLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_CLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_CLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_CLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_CLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_CLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_CLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_CLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_CLK_DI();
		}

	}

}
/**********************************************************************************************************
 * @fn : GPIO_Init
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return :
 *
 * @Note:
 **********************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

}
/**********************************************************************************************************
 * @fn : GPIO_DeInit
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return :
 *
 * @Note:
 **********************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

}
/**********************************************************************************************************
 * @fn : GPIO_ReadFromInputPin
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return :
 *
 * @Note:
 **********************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	return 0;
}
/**********************************************************************************************************
 * @fn : GPIO_ReadFromInputPort
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return :
 *
 * @Note:
 **********************************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

	return 0;
}
/**********************************************************************************************************
 * @fn : GPIO_WriteToOutputPin
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return :
 *
 * @Note:
 **********************************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t value) {

}
/**********************************************************************************************************
 * @fn : GPIO_WriteToOutputPort
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return :
 *
 * @Note:
 **********************************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {

}
/**********************************************************************************************************
 * @fn : GPIO_TogglePin
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return :
 *
 * @Note:
 **********************************************************************************************************/
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

}
/**********************************************************************************************************
 * @fn : GPIO_IRQConfig
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return :
 *
 * @Note:
 **********************************************************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {

}
/**********************************************************************************************************
 * @fn : GPIO_IRQHandling
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return :
 *
 * @Note:
 **********************************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber) {

}

