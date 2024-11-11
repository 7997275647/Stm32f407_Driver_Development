/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Nov 11, 2024
 *      Author: Modugu Sandeep Reddy
 */

#include "stm32f407xx_spi_driver.h"
/*
 * peripheral clock setup
 */
void SPI_ClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_CLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_CLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_CLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_CLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_CLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_CLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_CLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_CLK_DI();
		}

	}


}
/*
 * Init and de-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){


	uint32_t tempreg =0;
	//configure the device mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2);


	//configure bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//BIDI mode should be cleared
		tempreg &= ~(1<< 15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//BIDI mode should be set
		tempreg |= (1<< 15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX){
		//BIDI mode should be cleared
		tempreg &= ~(1<< 15);
		//RX only should be set
		tempreg |= (1<< 10);

	//configure sclk speed
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << 3);

	}

    //configure Data Frame Format

	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << 11);

	//configure CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << 1);

	//configure CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHASE << 0);

	//store tempreg into SPI|_CR1
	pSPIHandle->pSPIx->CR1 = tempreg;


}
void SPI_DeInit(SPI_Handle_t *pSPIHandle){
	if (pSPIHandle->pSPIx == SPI1){
		RCC->APB2RSTR |= (1<< 12);
		RCC->APB2RSTR &= ~(1<< 12);

	}else if (pSPIHandle->pSPIx == SPI2){
		RCC->APB2RSTR |= (1<< 13);
		RCC->APB2RSTR &= ~(1<< 13);
	}else if (pSPIHandle->pSPIx == SPI2){
		RCC->APB1RSTR |= (1<< 14);
		RCC->APB1RSTR &= ~(1<< 14);

	}else if (pSPIHandle->pSPIx == SPI3){
		RCC->APB1RSTR |= (1<< 15);
		RCC->APB1RSTR &= ~(1<< 15);
	}

}
/*
 * Spi Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

}

void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

}
/*
 * Interrupt configuration and ISR handling Apis
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

}

