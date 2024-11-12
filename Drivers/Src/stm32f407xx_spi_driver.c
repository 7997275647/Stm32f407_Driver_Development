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

	SPI_ClockControl(pSPIHandle->pSPIx , ENABLE);
	uint32_t tempreg =0;
	//configure the device mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);


	//configure bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//BIDI mode should be cleared
		tempreg &= ~(1<< SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//BIDI mode should be set
		tempreg |= (1<< SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX){
		//BIDI mode should be cleared
		tempreg &= ~(1<< SPI_CR1_BIDIMODE);
		//RX only should be set
		tempreg |= (1<< SPI_CR1_RXONLY);

	//configure sclk speed
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	}

    //configure Data Frame Format

	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//configure CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//configure CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHASE << SPI_CR1_CPHA);


	//store tempreg into SPI|_CR1
	pSPIHandle->pSPIx->CR1 = tempreg;


}



void SPI_Control(SPI_Handle_t *pSPIHandle, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else{
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
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
/**********************************************************************************************************
 * @fn : SPI_SendData
 * @brief :
 * @param[1]:
 * @param[2]:
 * @param[3]:
 *
 * @return :
 *
 * @Note: This is a blocking call
 **********************************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		//Follow state diagram of SPI send data
	while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)== FLAG_RESET);
	    //Check DFF BIT FIELD
	if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
		//16 bit Frame format
		//load 16 bit to SPI_DR
		pSPIx->DR = *(uint16_t *)pTxBuffer;
		Len -= 2;
		(uint16_t *)pTxBuffer++;
	} else if ((!(pSPIx->CR1 & (1 << SPI_CR1_DFF)))) {
		//8 bit Frame format
		//load 8 bit to SPI_DR
		pSPIx->DR = *pTxBuffer;
		Len --;
		pTxBuffer++;
	}
	}

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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if (pSPIx->SR & FlagName){
		return FLAG_SET;
	}else{
		return FLAG_RESET;
	}
}

