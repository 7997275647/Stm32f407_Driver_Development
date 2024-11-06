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
	uint32_t temp = 0;

	// configure mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  // clearing the bit positions
		pGPIOHandle->pGPIOx->MODER = pGPIOHandle->pGPIOx->MODER | temp;  //setting bit positions according to the pin mode

	} else {

	}

	//configure speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  // clearing the bit positions
	pGPIOHandle->pGPIOx->OSPEEDR = pGPIOHandle->pGPIOx->OSPEEDR | temp;                     //setting bit positions according to the pin speed
	temp = 0;

	//configure pupd

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  // clearing the bit positions

	pGPIOHandle->pGPIOx->PUPDR = pGPIOHandle->pGPIOx->PUPDR | temp;

	//configure otype
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOType
			<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  // clearing the bit positions

	pGPIOHandle->pGPIOx->OTYPER = pGPIOHandle->pGPIOx->OTYPER | temp;
	temp = 0;

	//configure alt functonality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		// configure alternate fun registers
		uint8_t temp1,temp2 = 0;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(15<<(4* temp2));  // clearing the bit positions

		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);

	}

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
	if (pGPIOx == GPIOA) {
		RCC->AHB1RSTR |= (1 << 0);
		RCC->AHB1RSTR &= ~(1 << 0);

	} else if (pGPIOx == GPIOB) {
		RCC->AHB1RSTR |= (1 << 1);
		RCC->AHB1RSTR &= ~(1 << 1);
	} else if (pGPIOx == GPIOC) {
		RCC->AHB1RSTR |= (1 << 2);
		RCC->AHB1RSTR &= ~(1 << 2);
	} else if (pGPIOx == GPIOD) {
		RCC->AHB1RSTR |= (1 << 3);
		RCC->AHB1RSTR &= ~(1 << 3);
	} else if (pGPIOx == GPIOE) {
		RCC->AHB1RSTR |= (1 << 4);
		RCC->AHB1RSTR &= ~(1 << 4);
	} else if (pGPIOx == GPIOF) {
		RCC->AHB1RSTR |= (1 << 5);
		RCC->AHB1RSTR &= ~(1 << 5);
	} else if (pGPIOx == GPIOG) {
		RCC->AHB1RSTR |= (1 << 6);
		RCC->AHB1RSTR &= ~(1 << 6);
	} else if (pGPIOx == GPIOH) {
		RCC->AHB1RSTR |= (1 << 7);
		RCC->AHB1RSTR &= ~(1 << 7);
	} else if (pGPIOx == GPIOI) {
		RCC->AHB1RSTR |= (1 << 8);
		RCC->AHB1RSTR &= ~(1 << 8);
	}


}
/**********************************************************************************************************
 * @fn : GPIO_ReadFromInputPin
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return : 0 or 1
 *
 * @Note:
 **********************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {


	return (pGPIOx->IDR >> PinNumber) & 0x01;

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

	return pGPIOx->IDR;
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
	if (value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber) ;
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNumber) ;
	}


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

	pGPIOx->ODR = value;


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
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);

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
