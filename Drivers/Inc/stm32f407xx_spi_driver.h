/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Nov 11, 2024
 *      Author: Modugu Sandeep Reddy
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*@SPI_DEVICEMODES
 * macros of possible modes of a GPIO pin
 */
#define SPI_MODE_MASTER 1
#define SPI_MODE_SLAVE  0

/*@SPI_BusConfig
 * macros of possible modes of a GPIO pin
 */
#define SPI_BUS_CONFIG_FD           0
#define SPI_BUS_CONFIG_HD           1
#define SPI_BUS_CONFIG_SIMPLEX_RX   2



/*@SPI_SclkSpeed
 * macros of possible modes of a GPIO pin
 */

#define SPI_SCLK_SPEED_DIV2      0
#define SPI_SCLK_SPEED_DIV4      1
#define SPI_SCLK_SPEED_DIV8      2
#define SPI_SCLK_SPEED_DIV16     3
#define SPI_SCLK_SPEED_DIV32     4
#define SPI_SCLK_SPEED_DIV64     5
#define SPI_SCLK_SPEED_DIV128    6
#define SPI_SCLK_SPEED_DIV256    7

/*@SPI_DFF
 * macros of possible modes of a GPIO pin
 */
#define SPI_DFF8   0
#define SPI_DFF16  1

/*@SPI_CPOL
 * macros of possible modes of a GPIO pin
 */
#define SPI_CPOL_LOW   0
#define SPI_CPOL_HIGH  1

/*@SPI_CPHASE
 * macros of possible modes of a GPIO pin
 */
#define SPI_CPHASE_LOW   0
#define SPI_CPHASE_HIGH  1

/*@SPI_SSM
 * macros of possible modes of a GPIO pin
 */
#define SPI_SSM_DI   0
#define SPI_SSM_EN   1
/*@SPI_SSM
 * macros of possible modes of a GPIO pin
 */
#define SPI_SSI_DI   0
#define SPI_SSI_EN   1



#define SPI_TXE_FLAG            (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG           (1 << SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG         (1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG            (1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG         (1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG           (1 << SPI_SR_MODF)
#define SPI_OVR_FLAG            (1 << SPI_SR_OVR)
#define SPI_BSY_FLAG            (1 << SPI_SR_BSY)
#define SPI_FRE_FLAG            (1 << SPI_SR_FRE)




typedef struct{

	uint8_t SPI_DeviceMode;        /* POSSIBLE VALUES FROM @SPI_DEVICE_MODES*/
	uint8_t SPI_BusConfig;         /* POSSIBLE VALUES FROM @SPI_BUSCONFIG*/
	uint8_t SPI_SclkSpeed;         /* POSSIBLE VALUES FROM @GPIO_OSPEED*/
	uint8_t SPI_DFF;               /* POSSIBLE VALUES FROM @GPIO_PUPD*/
	uint8_t SPI_CPOL;               /* POSSIBLE VALUES FROM @GPIO_OTYPES*/
	uint8_t SPI_CPHASE;
	uint8_t SPI_SSM;
	uint8_t SPI_SSI;

}SPI_Config_t;


typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

}SPI_Handle_t;



/******************************************************************************************************/
/*                                         APIs Supported by this driver                              */
/*                   For more information about APIs check function definitions                       */
/******************************************************************************************************/

/*
 * peripheral clock setup
 */
void SPI_ClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi);
/*
 * Init and de-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Control(SPI_Handle_t *pSPIHandle,uint8_t EnorDi);
void SPI_DeInit(SPI_Handle_t *pSPIHandle);
/*
 * Read and write
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
/*
 * Interrupt configuration and ISR handling Apis
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
