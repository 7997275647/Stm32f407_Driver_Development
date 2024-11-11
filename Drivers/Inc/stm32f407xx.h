/*
 * stm32f407xx.h
 *
 *  Created on: Nov 4, 2024
 *      Author: Modugu Sandeep Reddy
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * PROCESSOR SPECIFIC MACROS
 * ARM CORTEX MX PROCESSOR NVIC_ISERx BASE ADDRESSES
 *
 * */
#define NVIC_ISER0  ((uint32_t *)0xE000E100)
#define NVIC_ISER1  (uint32_t *)0xE000E104
#define NVIC_ISER2  (uint32_t *)0xE000E108
#define NVIC_ISER3  (uint32_t *)0xE000E10C
#define NVIC_ISER4  (uint32_t *)0xE000E110
/*
 *
 * ARM CORTEX MX PROCESSOR NVIC_ISERx BASE ADDRESSES
 *
 * */

#define NVIC_ICER0  (uint32_t *)0xE000E180
#define NVIC_ICER1  (uint32_t *)0xE000E184
#define NVIC_ICER2  (uint32_t *)0xE000E188
#define NVIC_ICER3  (uint32_t *)0xE000E18C
#define NVIC_ICER4  (uint32_t *)0xE000E190
/*
 *
 * ARM CORTEX MX PROCESSOR NVIC priority register BASE ADDRESSES
 *
 * */
#define NVIC_IPR  (uint32_t *)0xE000E400



#define NO_PR_BITS_IMPLEMENTED     4

/*
 *
 * BASE ADDRESSES OF FLASH AND SRAM MEMEORY
 *
 * */
#define FLASH_BASEADDR  0x08000000U     //BASE ADDRESS OF FLASH MEMORY
#define SRAM1_BASEADDR  0x20000000U     //BASE ADDRESS OF SRAM1
#define SRAM2_BASEADDR  0x2001C000U     //BASE ADDR OF SRAM2
#define ROM_BASEADDR    0x1FFF0000U     //BASE ADDR OF SYSTEM MEMORY
#define SRAM            SRAM1_BASEADDR

/*
 *
 * BASE ADDRESSES OF BUS DOMAINS
 *
 * */

#define AHB3_BASEADDR  0x60000000U       //AHB3 BASE ADDRESS
#define AHB2_BASEADDR  0x50000000U       //AHB2 BASE ADDRESS
#define AHB1_BASEADDR  0x40020000U       //AHB1 BASE ADDRESS
#define APB2_BASEADDR  0x40010000U       //APB2 BASE ADDRESS
#define APB1_BASEADDR  0x40000000U       //APB1 BASE ADDRESS


/*
 *
 * BASE ADDRESSES OF AHB1 PERIPHERALS
 *
 * */

#define GPIOA_BASEADDR (AHB1_BASEADDR+0x0000)     //BASE ADDRESS OF GPIOA PERIPHERAL
#define GPIOB_BASEADDR (AHB1_BASEADDR+0x0400)
#define GPIOC_BASEADDR (AHB1_BASEADDR+0x0800)
#define GPIOD_BASEADDR (AHB1_BASEADDR+0x0C00)
#define GPIOE_BASEADDR (AHB1_BASEADDR+0x1000)
#define GPIOF_BASEADDR (AHB1_BASEADDR+0x1400)
#define GPIOG_BASEADDR (AHB1_BASEADDR+0x1800)
#define GPIOH_BASEADDR (AHB1_BASEADDR+0x1C00)
#define GPIOI_BASEADDR (AHB1_BASEADDR+0x2000)


#define RCC_BASEADDR        (AHB1_BASEADDR+0x3800)       // RCC BASE ADDRESS
#define CRC_BASEADDR        (AHB1_BASEADDR+0x3000)      // CRC BASE ADDR
#define BKPSRAM_BASEADDR    (AHB1_BASEADDR+0x4000)      // BKPSRAM BASE ADDR
#define DMA1_BASEADDR       (AHB1_BASEADDR+0x6000)      // DMA1 BASE ADDR
#define DMA2_BASEADDR       (AHB1_BASEADDR+0x6400)      // DMA2 BASE ADDR


/*
 *
 * BASE ADDRESSES OF APB1 PERIPHERALS
 *
 * */
#define TIM2_BASEADDR       (APB1_BASEADDR+0x0000)     //TIM2 BASE ADDR
#define TIM3_BASEADDR       (APB1_BASEADDR+0x0400)     //TIM2 BASE ADDR
#define TIM4_BASEADDR       (APB1_BASEADDR+0x0800)     //TIM2 BASE ADDR
#define TIM5_BASEADDR       (APB1_BASEADDR+0x0C00)     //TIM2 BASE ADDR
#define TIM6_BASEADDR       (APB1_BASEADDR+0x1000)     //TIM2 BASE ADDR
#define TIM7_BASEADDR       (APB1_BASEADDR+0x1400)     //TIM2 BASE ADDR
#define TIM12_BASEADDR      (APB1_BASEADDR+0x1800)     //TIM2 BASE ADDR
#define TIM13_BASEADDR      (APB1_BASEADDR+0x1C00)     //TIM2 BASE ADDR
#define TIM14_BASEADDR      (APB1_BASEADDR+0x2000)     //TIM2 BASE ADDR
#define TIM2_BASEADDR       (APB1_BASEADDR+0x0000)     //TIM2 BASE ADDR

#define SPI2_BASEADDR       (APB1_BASEADDR+0x3800)     //SPI2 BASE ADDR
#define SPI3_BASEADDR       (APB1_BASEADDR+0x3C00)     //SPI2 BASE ADDR
#define USART2_BASEADDR     (APB1_BASEADDR+0x4400)     //SPI2 BASE ADDR
#define USART3_BASEADDR     (APB1_BASEADDR+0x4800)     //SPI2 BASE ADDR
#define UART4_BASEADDR      (APB1_BASEADDR+0x4C00)     //SPI2 BASE ADDR
#define UART5_BASEADDR      (APB1_BASEADDR+0x5000)     //SPI2 BASE ADDR
#define I2C1_BASEADDR       (APB1_BASEADDR+0x5400)     //SPI2 BASE ADDR
#define I2C2_BASEADDR       (APB1_BASEADDR+0x5800)     //SPI2 BASE ADDR
#define I2C3_BASEADDR       (APB1_BASEADDR+0x5C00)     //SPI2 BASE ADDR
#define CAN1_BASEADDR       (APB1_BASEADDR+0x6400)     //SPI2 BASE ADDR
#define CAN2_BASEADDR       (APB1_BASEADDR+0x6800)     //SPI2 BASE ADDR


/*
 *
 * BASE ADDRESSES OF APB2 PERIPHERALS
 *
 * */

#define USART1_BASEADDR       (APB2_BASEADDR+0x1000)     //USART1 BASE ADDR
#define USART6_BASEADDR       (APB2_BASEADDR+0x1400)     //USART1 BASE ADDR
#define SPI1_BASEADDR         (APB2_BASEADDR+0x3000)     //USART1 BASE ADDR
#define EXTI_BASEADDR         (APB2_BASEADDR+0x3C00)     //USART1 BASE ADDR
#define SYSCFG_BASEADDR       (APB2_BASEADDR+0x3800)     //USART1 BASE ADDR




/*
 *
 * PERIPHERAL DEFINITIONS
 *
 * */

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)


#define RCC   ((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI   ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG   ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)






/*
 *
 * CLOCK ENABLE MACROS FOR GPIO PERIPHERAL
 *
 * */

#define GPIOA_CLK_EN()  RCC->AHB1ENR |= (1 << 0)
#define GPIOB_CLK_EN()  RCC->AHB1ENR |= (1 << 1)
#define GPIOC_CLK_EN()  RCC->AHB1ENR |= (1 << 2)
#define GPIOD_CLK_EN()  RCC->AHB1ENR |= (1 << 3)
#define GPIOE_CLK_EN()  RCC->AHB1ENR |= (1 << 4)
#define GPIOF_CLK_EN()  RCC->AHB1ENR |= (1 << 5)
#define GPIOG_CLK_EN()  RCC->AHB1ENR |= (1 << 6)
#define GPIOH_CLK_EN()  RCC->AHB1ENR |= (1 << 7)
#define GPIOI_CLK_EN()  RCC->AHB1ENR |= (1 << 8)



/*
 *
 * CLOCK ENABLE MACROS FOR I2C PERIPHERAL
 *
 * */

#define I2C1_CLK_EN()  RCC->APB1ENR |= (1 << 21)
#define I2C2_CLK_EN()  RCC->APB1ENR |= (1 << 22)
#define I2C3_CLK_EN()  RCC->APB1ENR |= (1 << 23)

/*
 *
 * CLOCK ENABLE MACROS FOR GPIO
 *
 * */

#define SPI1_CLK_EN()  RCC->APB2ENR |= (1 << 12)
#define SPI4_CLK_EN()  RCC->APB2ENR |= (1 << 13)
#define SPI2_CLK_EN()  RCC->APB1ENR |= (1 << 14)
#define SPI3_CLK_EN()  RCC->APB1ENR |= (1 << 15)


/*
 *
 * CLOCK ENABLE MACROS FOR UART PERIPHERAL
 *
 * */

#define USART1_CLK_EN()  RCC->APB2ENR |= (1 << 4)
#define USART2_CLK_EN()  RCC->APB1ENR |= (1 << 17)
#define USART3_CLK_EN()  RCC->APB1ENR |= (1 << 18)
#define UART4_CLK_EN()   RCC->APB1ENR |= (1 << 19)
#define UART5_CLK_EN()   RCC->APB1ENR |= (1 << 20)
#define USART6_CLK_EN()  RCC->APB2ENR |= (1 << 5)
#define UART7_CLK_EN()   RCC->APB1ENR |= (1 << 30)
#define UART8_CLK_EN()   RCC->APB1ENR |= (1 << 31)


/*
 *
 * CLOCK ENABLE MACROS FOR SYSCFG PERIPHERAL
 *
 * */

#define SYSCFG_CLK_EN()   RCC->APB2ENR |= (1 << 14)


/*
 *
 * CLOCK DISABLE MACROS FOR GPIO PERIPHERAL
 *
 * */
#define GPIOA_CLK_DI()  RCC->AHB1ENR &= ~(1 << 0)
#define GPIOB_CLK_DI()  RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_CLK_DI()  RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_CLK_DI()  RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_CLK_DI()  RCC->AHB1ENR &= ~(1 << 4)
#define GPIOF_CLK_DI()  RCC->AHB1ENR &= ~(1 << 5)
#define GPIOG_CLK_DI()  RCC->AHB1ENR &= ~(1 << 6)
#define GPIOH_CLK_DI()  RCC->AHB1ENR &= ~(1 << 7)
#define GPIOI_CLK_DI()  RCC->AHB1ENR &= ~(1 << 8)


/*
 *
 * CLOCK ENABLE MACROS FOR I2C PERIPHERAL
 *
 * */

#define I2C1_CLK_DI()  RCC->APB1ENR &= ~(1 << 21)
#define I2C2_CLK_DI()  RCC->APB1ENR &= ~(1 << 22)
#define I2C3_CLK_DI()  RCC->APB1ENR &= ~(1 << 23)

/*
 *
 * CLOCK ENABLE MACROS FOR GPIO
 *
 * */

#define SPI1_CLK_DI()  RCC->APB2ENR &= ~(1 << 12)
#define SPI4_CLK_DI()  RCC->APB2ENR &= ~(1 << 13)
#define SPI2_CLK_DI()  RCC->APB1ENR &= ~(1 << 14)
#define SPI3_CLK_DI()  RCC->APB1ENR &= ~(1 << 15)


/*
 *
 * CLOCK ENABLE MACROS FOR UART PERIPHERAL
 *
 * */

#define USART1_CLK_DI()  RCC->APB2ENR &= ~(1 << 4)
#define USART2_CLK_DI()  RCC->APB1ENR &= ~(1 << 17)
#define USART3_CLK_DI()  RCC->APB1ENR &= ~(1 << 18)
#define UART4_CLK_DI()   RCC->APB1ENR &= ~(1 << 19)
#define UART5_CLK_DI()   RCC->APB1ENR &= ~(1 << 20)
#define USART6_CLK_DI()  RCC->APB2ENR &= ~(1 << 5)
#define UART7_CLK_DI()   RCC->APB1ENR &= ~(1 << 30)
#define UART8_CLK_DI()   RCC->APB1ENR &= ~(1 << 31)


/*
 *
 * CLOCK ENABLE MACROS FOR SYSCFG PERIPHERAL
 *
 * */

#define SYSCFG_CLK_DI()   RCC->APB2ENR &= ~(1 << 14)


#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE

#define GPIO_PIN_SET         1
#define GPIO_PIN_RESET       0



/*
 *
 * vector table MACROS for EXTI IRQ NUMBERS
 *
 * */

#define EXTI_NO_0            6
#define EXTI_NO_1            7
#define EXTI_NO_2            8
#define EXTI_NO_3            9
#define EXTI_NO_4           10
#define EXTI_NO_9_TO_5      23
#define EXTI_NO_10_TO_15    40

/*
 *
 * NVIC priority MACROS
 *
 * */

#define NVIC_PRIO_0            0
#define NVIC_PRIO_1            1
#define NVIC_PRIO_2            2
#define NVIC_PRIO_3            3
#define NVIC_PRIO_4            4
#define NVIC_PRIO_5            5
#define NVIC_PRIO_6            6
#define NVIC_PRIO_7            7
#define NVIC_PRIO_8            8
#define NVIC_PRIO_9            9
#define NVIC_PRIO_10           10
#define NVIC_PRIO_11           11
#define NVIC_PRIO_12           12
#define NVIC_PRIO_13           13
#define NVIC_PRIO_14           14
#define NVIC_PRIO_15           15


/*
 *
 * PERIPHERAL REGISTER DEFINITION STRUCTURES
 *
 *
 *NOTE: REGISTERS ARE SPECIFIC TO STM32F407XX MCU
 *
 *
 * */

typedef struct{
	__vo uint32_t MODER;         // GPIO port mode register ADDR OFFSET 0X00
	__vo uint32_t OTYPER;        // GPIO port output type register ADDR OFFSET 0X04
	__vo uint32_t OSPEEDR;       // GPIO port output speed register ADDR OFFSET 0X08
	__vo uint32_t PUPDR;         // GPIO port pull-up/pull-down register ADDR OFFSET 0X0C
	__vo uint32_t IDR;           // GPIO port input data register ADDR OFFSET 0X10
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];        //AFR[0] IS AFRL REGISTER AND AFR[1] IS AFRH REGISTER

}GPIO_RegDef_t;




typedef struct{
	__vo uint32_t CR;         // GPIO port mode register ADDR OFFSET 0X00
	__vo uint32_t PLLCFGR;        // GPIO port output type register ADDR OFFSET 0X04
	__vo uint32_t CFGR;       // GPIO port output speed register ADDR OFFSET 0X08
	__vo uint32_t CIR;         // GPIO port pull-up/pull-down register ADDR OFFSET 0X0C
	__vo uint32_t AHB1RSTR;           // GPIO port input data register ADDR OFFSET 0X10
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;        //AFR[0] IS AFRL REGISTER AND AFR[1] IS AFRH REGISTER
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1;
	__vo uint32_t RESERVED2;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t RESERVED5;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;

	__vo uint32_t RESERVED6;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESRVED7;
	__vo uint32_t RESERVED8;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED9;
	__vo uint32_t RESERVED10;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLISAICFGR;
	__vo uint32_t DCKCFGR;

}RCC_RegDef_t;


typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
     uint32_t RESERVED[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;





#endif /* INC_STM32F407XX_H_ */
