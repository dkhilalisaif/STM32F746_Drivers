/*
 * stm32f746xx.h
 *
 *  Created on: Mars 20, 2020
 *      Author: saif
 */

#ifndef INC_STM32F746XX_H_
#define INC_STM32F746XX_H_

#include <stdint.h>



/********************************** Macros **************************************/
#define __vo                           volatile

#define SET                            1
#define RESET                          0

#define ENABLE                         1
#define DISABLE                        0


#define GPIO_PIN_SET                   1
#define GPIO_PIN_RESET                 0

#define FLAG_SET                       1
#define FLAG_RESET                     0

/*
 *    Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR                  0x08000000U
#define SRAM1_BASEADDR                  0x20010000U
#define SRAM                            SRAM1_BASEADDR
#define SRAM2_BASEADDR                  0x2004C000U
#define ROM_BASEADDR                    0x1FF00000U

/*****************************************START: Processor Specific Details *******************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses
 */

#define NVIC_ISER0                      ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1                      ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2                      ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3                      ((__vo uint32_t*)0xE000E10C)


/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses
 */

#define NVIC_ICER0                      ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1                      ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2                      ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3                      ((__vo uint32_t*)0xE000E18C)



/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */

#define NVIC_PR_BASE_ADDR                ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED            4
/*
 *    AHBx and APBx Bus Peripheral base addresses
 */



#define PERIPH_BASE                    0x40000000U
#define APB1PERIPH_BASE                PERIPH_BASE
#define APB2PERIPH_BASE                0x40010000U
#define AHB1PERIPH_BASE                0x40020000U
#define AHB2PERIPH_BASE                0x50000000U
#define AHB3PERIPH_BASE                0xA0000000U


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

//GPIOi Peripherals   i = { A, B, ..., k}
#define GPIOA_BASEADDR                 (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR                 (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR                 (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR                 (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR                 (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR                 (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR                 (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR                 (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR                 (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR                 (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR                 (AHB1PERIPH_BASE + 0x2800)

// CRC Peripheral
#define CRC_BASEADDR                   (AHB1PERIPH_BASE + 0x3000)

// RCC Peripheral
#define RCC_BASEADDR                   (AHB1PERIPH_BASE + 0x3800)

// Flash interface register Peripheral
#define Flash_Interface_BASEADDR       (AHB1PERIPH_BASE + 0x3C00)

//BKPSRAM Peripheral
#define  BKPSRAM_BASEADDR              (AHB1PERIPH_BASE + 0x4000)

// DMAx Peripheral
#define DMA1_BASEADDR                  (AHB1PERIPH_BASE + 0x6000)
#define DMA2_BASEADDR                  (AHB1PERIPH_BASE + 0x6400)

// ETHERNET MAC Peripheral
#define ETHERNET_MAC_BASEADDR          (AHB1PERIPH_BASE + 0x8000)

// DMA2D Peripheral
#define DMA2D_BASEADDR                 (AHB1PERIPH_BASE + 0xB000)




/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define SPI2_BASEADDR                  (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR                  (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR                (APB1PERIPH_BASE + 0x3C00)
#define USART3_BASEADDR                (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR                 (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR                 (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASEADDR                  (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR                  (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR                  (APB1PERIPH_BASE + 0x5C00)
#define I2C4_BASEADDR                  (APB1PERIPH_BASE + 0x6000)
#define UART7_BASEADDR                 (APB1PERIPH_BASE + 0x7800)
#define UART8_BASEADDR                 (APB1PERIPH_BASE + 0x7C00)


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define USART1_BASEADDR                (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR                (APB2PERIPH_BASE + 0x1400)
#define SPI1_BASEADDR                  (APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR                  (APB2PERIPH_BASE + 0x3400)
#define SYSCFG_BASEADDR                (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR                  (APB2PERIPH_BASE + 0x3C00)
#define SPI5_BASEADDR                  (APB2PERIPH_BASE + 0x5000)
#define SPI6_BASEADDR                  (APB2PERIPH_BASE + 0x5400)



/**********************Peripheral register definition structures***************************/

/*
 * GPIO Peripheral register definition
 */
typedef struct
{
	 __vo uint32_t MODER;      // GPIO port mode register
	 __vo uint32_t OTYPER;     // GPIO port output type register
	 __vo uint32_t OSPEEDR;    // GPIO port output speed register
	 __vo uint32_t PUPDR;      // GPIO port pull-up/pull-down register
	 __vo uint32_t IDR;        // GPIO port input data register
	 __vo uint32_t ODR;        // GPIO port output data register
	 __vo uint32_t BSRR;       // GPIO port bit set/reset register
	 __vo uint32_t LCKR;       // GPIO port configuration lock register
	 __vo uint32_t AFR[2];     // AFR[0]: GPIO alternate function low register , AFR[1]: GPIO alternate function high register

}GPIO_RegDef_t;

/*
 * peripheral definitions (peripheral base addresses typexasted to xxx_RegDef_t)
 */
#define GPIOA                         ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                         ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                         ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                         ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                         ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                         ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                         ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                         ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI                         ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ                         ((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK                         ((GPIO_RegDef_t*)GPIOK_BASEADDR)


/*
 * RCC Peripheral register definition
 */
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED5;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED7[2];
}RCC_RegDef_t;

#define RCC                         ((RCC_RegDef_t*)RCC_BASEADDR)
/*
 * EXTI Peripheral register definition
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;

#define EXTI                        ((EXTI_RegDef_t*)EXTI_BASEADDR)


/*
 * SYSCFG  Peripheral register definition
 */

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;

}SYSCFG_RegDef_t;

#define SYSCFG                        ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define GPIO_BASEADDR_TO_CODE(x)    ( (x == GPIOA)? 0 :\
		                              (x == GPIOB)? 1 :\
				                      (x == GPIOC)? 2 :\
				              		  (x == GPIOD)? 3 :\
				           			  (x == GPIOE)? 4 :\
				   					  (x == GPIOF)? 5 :\
									  (x == GPIOG)? 6 :\
									  (x == GPIOH)? 7 :\
                                      (x == GPIOI)? 8 :\
                                      (x == GPIOJ)? 9 :\
									  (x == GPIOK)? 10 :0 )

/*
 * SPI/ I2S  Peripheral register definition
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;


}SPI_RegDef_t;

#define SPI1                        ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                        ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                        ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4                        ((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5                        ((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6                        ((SPI_RegDef_t*)SPI6_BASEADDR)
/*
 * clock enable macros for GPIOx peripherals
 */


#define GPIOA_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<0)  )
#define GPIOB_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<1)  )
#define GPIOC_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<2)  )
#define GPIOD_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<3)  )
#define GPIOE_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<4)  )
#define GPIOF_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<5)  )
#define GPIOG_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<6)  )
#define GPIOH_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<7)  )
#define GPIOI_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<8)  )
#define GPIOJ_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<9)  )
#define GPIOK_PCLK_EN()              ( RCC->AHB1ENR  |= (1<<10) )


/*
 * clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()              ( RCC->APB1ENR  |= (1<<21)  )
#define I2C2_PCLK_EN()              ( RCC->APB1ENR  |= (1<<22)  )
#define I2C3_PCLK_EN()              ( RCC->APB1ENR  |= (1<<23)  )
#define I2C4_PCLK_EN()              ( RCC->APB1ENR  |= (1<<24)  )


/*
 * clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()              ( RCC->APB2ENR  |= (1<<12)  )
#define SPI2_PCLK_EN()              ( RCC->APB1ENR  |= (1<<14)  )
#define SPI3_PCLK_EN()              ( RCC->APB1ENR  |= (1<<15)  )
#define SPI4_PCLK_EN()              ( RCC->APB2ENR  |= (1<<13)  )
#define SPI5_PCLK_EN()              ( RCC->APB2ENR  |= (1<<20)  )
#define SPI6_PCLK_EN()              ( RCC->APB2ENR  |= (1<<21)  )

/*
 * clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()              ( RCC->APB2ENR  |= (1<<4)  )
#define USART2_PCLK_EN()              ( RCC->APB1ENR  |= (1<<17) )
#define USART3_PCLK_EN()              ( RCC->APB1ENR  |= (1<<18) )
#define UART4_PCLK_EN()               ( RCC->APB1ENR  |= (1<<19) )
#define UART5_PCLK_EN()               ( RCC->APB1ENR  |= (1<<20) )
#define USART6_PCLK_EN()              ( RCC->APB2ENR  |= (1<<5)  )
#define UART7_PCLK_EN()               ( RCC->APB1ENR  |= (1<<30) )
#define UART8_PCLK_EN()               ( RCC->APB1ENR  |= (1<<31) )


/*
 * clock enable macro for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN()              ( RCC->APB2ENR  |= (1<<14)  )


/*
 * clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<0)  )
#define GPIOB_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<1)  )
#define GPIOC_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<2)  )
#define GPIOD_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<3)  )
#define GPIOE_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<4)  )
#define GPIOF_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<5)  )
#define GPIOG_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<6)  )
#define GPIOH_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<7)  )
#define GPIOI_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<8)  )
#define GPIOJ_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<9)  )
#define GPIOK_PCLK_DI()              ( RCC->AHB1ENR  &= ~(1<<10) )

/*
 * clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()              ( RCC->APB1ENR  &= ~(1<<21)  )
#define I2C2_PCLK_DI()              ( RCC->APB1ENR  &= ~(1<<22)  )
#define I2C3_PCLK_DI()              ( RCC->APB1ENR  &= ~(1<<23)  )
#define I2C4_PCLK_DI()              ( RCC->APB1ENR  &= ~(1<<24)  )


/*
 * clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()              ( RCC->APB2ENR  |= (1<<12)  )
#define SPI2_PCLK_DI()              ( RCC->APB1ENR  |= (1<<14)  )
#define SPI3_PCLK_DI()              ( RCC->APB1ENR  |= (1<<15)  )
#define SPI4_PCLK_DI()              ( RCC->APB2ENR  |= (1<<13)  )
#define SPI5_PCLK_DI()              ( RCC->APB2ENR  |= (1<<20)  )
#define SPI6_PCLK_DI()              ( RCC->APB2ENR  |= (1<<21)  )

/*
 * clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()              ( RCC->APB2ENR  |= (1<<4)  )
#define USART2_PCLK_DI()              ( RCC->APB1ENR  |= (1<<17) )
#define USART3_PCLK_DI()              ( RCC->APB1ENR  |= (1<<18) )
#define UART4_PCLK_DI()               ( RCC->APB1ENR  |= (1<<19) )
#define UART5_PCLK_DI()               ( RCC->APB1ENR  |= (1<<20) )
#define USART6_PCLK_DI()              ( RCC->APB2ENR  |= (1<<5)  )
#define UART7_PCLK_DI()               ( RCC->APB1ENR  |= (1<<30) )
#define UART8_PCLK_DI()               ( RCC->APB1ENR  |= (1<<31) )

/*
 *  Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));}while(0)
#define GPIOJ_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<9)); (RCC->AHB1RSTR &= ~(1<<9));}while(0)
#define GPIOK_REG_RESET()            do{ (RCC->AHB1RSTR |= (1<<10)); (RCC->AHB1RSTR &= ~(1<10));}while(0)

/*
 *  IRQ(Interrupt request) numbers of STM32F746x MCU
 *  NOTE: update these macros with valid values according to your MCU
 *  TODO: complete the list in case of need
 */

#define IRQ_NO_EXTI0          6
#define IRQ_NO_EXTI1          7
#define IRQ_NO_EXTI2          8
#define IRQ_NO_EXTI3          9
#define IRQ_NO_EXTI4          10
#define IRQ_NO_EXTI9_5        23
#define IRQ_NO_EXTI15_10      40


/************************************************************************************
 *                 Macros for all the possible priority levels                      *
 ************************************************************************************/
#define NVIC_IRQ_PRIO_0       0
#define NVIC_IRQ_PRIO_1       1
#define NVIC_IRQ_PRIO_2       2
#define NVIC_IRQ_PRIO_3       3
#define NVIC_IRQ_PRIO_4       4
#define NVIC_IRQ_PRIO_5       5
#define NVIC_IRQ_PRIO_6       6
#define NVIC_IRQ_PRIO_7       7
#define NVIC_IRQ_PRIO_8       8
#define NVIC_IRQ_PRIO_9       9
#define NVIC_IRQ_PRIO_10      10
#define NVIC_IRQ_PRIO_11      11
#define NVIC_IRQ_PRIO_12      12
#define NVIC_IRQ_PRIO_13      13
#define NVIC_IRQ_PRIO_14      14
#define NVIC_IRQ_PRIO_15      15

/***********************************************************************************
 *                  Bit position definitions of SPI peripheral                     *
 ***********************************************************************************/
#define SPIx_CR1_CPHA         0
#define SPIx_CR1_CPOL         1
#define SPIx_CR1_MSTR         2
#define SPIx_CR1_BR           3
#define SPIx_CR1_SPE          6
#define SPIx_CR1_LSBFIRST     7
#define SPIx_CR1_SSI          8
#define SPIx_CR1_SSM          9
#define SPIx_CR1_RXONLY       10
#define SPIx_CR1_CRCL         11
#define SPIx_CR1_CRCNEXT      12
#define SPIx_CR1_CRCEN        13
#define SPIx_CR1_BIDIOE       14
#define SPIx_CR1_BIDIMODE     15

/*
 * Bit Position defenitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN          0
#define SPI_CR2_TXDMAEN          1
#define SPI_CR2_SSOE             2
#define SPI_CR2_NSSP             3
#define SPI_CR2_FRF              4
#define SPI_CR2_ERRIE            5
#define SPI_CR2_RXNEIE           6
#define SPI_CR2_TXEIE            7
#define SPI_CR2_DS               8
#define SPI_CR2_FRXTH            12
#define SPI_CR2_LDMA_RX          13
#define SPI_CR2_LDMA_TX          14
/*
 * Bit Position definitions SPI_SR
 */

#define SPI_SR_RXE               0
#define SPI_SR_TXE               1
#define SPI_SR_CHSIDE            2
#define SPI_SR_UDR               3
#define SPI_SR_CRCERR            4
#define SPI_SR_MODF              5
#define SPI_SR_OVR               6
#define SPI_SR_BSY               7
#define SPI_SR_FRE               8
#define SPI_SR_FRLVL             9
#define SPI_SR_FTLVL             11



#include "stm32f746xx_gpio_driver.h"
#include "stm32f746xx_spi_driver.h"


#endif /* INC_STM32F746XX_H_ */
