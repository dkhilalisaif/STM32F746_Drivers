/*
 * stm32f746xx_gpio_driver.h
 *
 *  Created on: Mars 20 ,2020
 *      Author: saif
 */

#ifndef INC_STM32F746XX_GPIO_DRIVER_H_
#define INC_STM32F746XX_GPIO_DRIVER_H_

#include "stm32f746xx.h"



typedef struct
{
	uint8_t GPIO_PinNumber;       /*!< possible values from @GPIO_PIN_NUMBER    >*/
	uint8_t GPIO_PinMode;         /*!< possible values from @GPIO_PIN_MODES     >*/
	uint8_t GPIO_PinSpeed;        /*!< possible values from @GPIO_PIN_SPEEDS    >*/
	uint8_t GPIO_PinPuPdControl;  /*!< possible values from @GPIO_PIN_PUPD      >*/
	uint8_t GPIO_PinOPType;       /*!< possible values from @GPIO_PIN_OP_TYPE   >*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/*
 * This is the Handle structure for a GPIO pin
 */

typedef struct
{
	//pointer to hold the base address of GPIO peripheral
	GPIO_RegDef_t *pGPIOx; /* < This holds the base address of the GPIO PORT to which the pin belongs  >*/
	GPIO_PinConfig_t GPIO_PinConfig; /*< This holds GPIO pin configuration settings >*/


}GPIO_Handle_t;




/*
 * @GPIO_PIN_NUMBER
 * GPIO pin possible modes
 */
#define GPIO_PIN_0          0
#define GPIO_PIN_1          1
#define GPIO_PIN_2          2
#define GPIO_PIN_3          3
#define GPIO_PIN_4          4
#define GPIO_PIN_5          5
#define GPIO_PIN_6          6
#define GPIO_PIN_7          7
#define GPIO_PIN_8          8
#define GPIO_PIN_9          9
#define GPIO_PIN_10         10
#define GPIO_PIN_11         11
#define GPIO_PIN_12         12
#define GPIO_PIN_13         13
#define GPIO_PIN_14         14
#define GPIO_PIN_15         15
/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN         0
#define GPIO_MODE_OUT        1
#define GPIO_MODE_ALTFUN     2
#define GPIO_MODE_ANALOG     3
#define GPIO_MODE_IT_FT      4
#define GPIO_MODE_IT_RT      5
#define GPIO_MODE_IT_RFT     6

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP      0
#define GPIO_OP_TYPE_OD      1


/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin possible output speeds
 */
#define GPIO_OP_SPEED_LOW       0
#define GPIO_OP_SPEED_MEDIUM    1
#define GPIO_OP_SPEED_FAST      2
#define GPIO_OP_SPEED_HIGH      3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up down configuration macros
 */
#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2


/*********************************************************************************************
 *                                 APIs Supported by this driver
 *      For more information about the APIs check the function definitions in the .c file
 **********************************************************************************************/

/*
 * peripheral clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

/*
 * Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t GPIO_PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t GPIO_PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t GPIO_PinNumber);
/*
 * IRQ configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t GPIO_PinNumber);







#endif /* INC_STM32F746XX_GPIO_DRIVER_H_ */
