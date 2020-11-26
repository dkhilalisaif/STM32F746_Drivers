/*
 * stm32f746xx_gpio_driver.c
 *
 *  Created on: Mars 20 ,2020
 *      Author: saif
 */

#include "stm32f746xx_gpio_driver.h"




/*
 * peripheral clock setup
 */

/****************************************************************************
 * @fn            - GPIO_PeriClockControl
 *
 * @brief         - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]     - Base address of the GPIO peripheral
 * @param[in]     - Enable or Disable macros
 *
 *
 * @return        - none
 *
 * @note          - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
		else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}
		else
		{
			GPIOK_PCLK_EN();
		}

	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
		else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DI();
		}
		else
		{
			GPIOK_PCLK_DI();
		}
	}



}

/*
 * Init and Deinit
 */

/****************************************************************************
 * @fn            - GPIO_Init
 *
 * @brief         - This function Initialise the given GPIO peripheral
 *
 * @param[in]     - Base address of the GPIO peripheral
 *
 *
 * @return        - none
 *
 * @note          - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	uint32_t temp=0;


	//1 . configure the mode of gpio pin
	if ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;

	}
	else
	{
		//interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
		{
			//1. configure the FTSR
			EXTI->FTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			EXTI->RTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure the RTSR and and FTSR
			EXTI->FTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);



		//3. enable the EXTI interrupt delivery  using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

	}



	//2 . configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3 . configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4 . configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5 . configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
	{
		//configure the alternate function register  //TODO
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2 ));
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << temp2 );

	}


}

/****************************************************************************
 * @fn            - GPIO_DeInit
 *
 * @brief         - This function de-initialise the given GPIO peripheral
 *
 * @param[in]     - Base address of the GPIO peripheral
 *
 *
 * @return        - none
 *
 * @note          - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA )
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();;
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
	else if (pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	}
	else
	{
		GPIOK_REG_RESET();
	}

}


/****************************************************************************
 * @fn            - GPIO_ReadFromInputPin
 *
 * @brief         - This function read from the given GPIO_PIN number
 *
 * @param[in]     - Base address of the GPIO peripheral
 * @param[in]     - PIN Number
 *
 * @return        - PIN STATE
 *
 * @note          - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t GPIO_PinNumber)
{
	uint8_t value;
	value = (uint8_t)(( pGPIOx->IDR >> GPIO_PinNumber ) & 0x00000001);
	return value;

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)( pGPIOx->IDR);
	return value;

}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t GPIO_PinNumber,uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		//write 1
		pGPIOx->ODR |= ( 0x1 << GPIO_PinNumber );

	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~( 0x1 << GPIO_PinNumber );
	}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
	pGPIOx->ODR |= value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t GPIO_PinNumber)
{
	pGPIOx->ODR ^= ( 1 << GPIO_PinNumber );

}
/*
 * IRQ configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}
		else if( (IRQNumber > 31) && (IRQNumber < 64) )
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if((IRQNumber > 64) && (IRQNumber < 96) )
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}


	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}
		else if( (IRQNumber > 31) && (IRQNumber < 64) )
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}
		else if((IRQNumber > 64) && (IRQNumber < 96) )
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}
	}

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	//1. Find out the IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);

}
void GPIO_IRQHandling(uint8_t GPIO_PinNumber)
{
	// clear the Pending Register corresponding to the Pin Number
	if (EXTI->PR & ( 1 << GPIO_PinNumber))
	{
		//clear
		EXTI->PR |= (1 << GPIO_PinNumber);

	}

}



