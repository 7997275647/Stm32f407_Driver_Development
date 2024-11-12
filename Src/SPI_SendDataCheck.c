/*
 * SPI_SendDataCheck.c
 *
 *  Created on: Nov 12, 2024
 *      Author: modug
 */

#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"




void delay(){
	for (uint32_t i = 0; i<500000/2; i++){

	}
}

void EXTI0_IRQHandler(){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_TogglePin(GPIOD,GPIO_PIN_NO_12);
}
//PB15 MOSI
//PB13 SCLK
//PB14 MISO
//PB12 NSS
//ALTFN MODE 5

int main(void)
{

	GPIO_Handle_t SPI_MOSI;
	memset(&SPI_MOSI, 0 , sizeof(SPI_MOSI));
	SPI_MOSI.pGPIOx = GPIOB;
	SPI_MOSI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	SPI_MOSI.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_MOSI.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_MOSI.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_HIGH_SPEED;
	SPI_MOSI.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	SPI_MOSI.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t SPI_SCLK;
	memset(&SPI_SCLK, 0 , sizeof(SPI_SCLK));
	SPI_SCLK.pGPIOx = GPIOB;
	SPI_SCLK.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	SPI_SCLK.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_SCLK.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_SCLK.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_HIGH_SPEED;
	SPI_SCLK.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	SPI_SCLK.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Handle_t SPI_MISO;
	memset(&SPI_MISO, 0 , sizeof(SPI_MISO));
	SPI_MISO.pGPIOx = GPIOB;
	SPI_MISO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	SPI_MISO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_MISO.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_MISO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_HIGH_SPEED;
	SPI_MISO.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	SPI_MISO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t SPI_NSS;
	memset(&SPI_NSS, 0 , sizeof(SPI_NSS));
	SPI_NSS.pGPIOx = GPIOB;
	SPI_NSS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	SPI_NSS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_NSS.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_NSS.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_HIGH_SPEED;
	SPI_NSS.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	SPI_NSS.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&SPI_NSS);
	GPIO_Init(&SPI_MISO);
	GPIO_Init(&SPI_MOSI);
	GPIO_Init(&SPI_SCLK);




	SPI_Handle_t SPI2Handler;
	SPI2Handler.pSPIx = SPI2;
	SPI2Handler.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handler.SPIConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2Handler.SPIConfig.SPI_SSM = SPI_SSM_EN;
	SPI2Handler.SPIConfig.SPI_DFF = SPI_DFF8;
	SPI2Handler.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handler.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handler.SPIConfig.SPI_CPHASE = SPI_CPHASE_LOW;
	SPI2Handler.SPIConfig.SPI_SSI = SPI_SSI_EN;

	SPI_Init(&SPI2Handler);

	// SPI is by default disabled so ENABLE it
	SPI_Control(&SPI2Handler,ENABLE);

	char data[] = "Hello world!";
    SPI_SendData(SPI2, (uint8_t *)data, strlen(data));

    SPI_Control(&SPI2Handler,DISABLE);


    /* Loop forever */
	while(1){



	}

	return 0;
}

