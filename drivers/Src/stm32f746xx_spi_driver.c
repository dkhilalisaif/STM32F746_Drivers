/*
 * stm32f746xx_spi_driver.c
 *
 *  Created on: 8 Sept 2020
 *      Author: saif
 */


#include "stm32f746xx_spi_driver.h"


/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}
		else if (pSPIx == SPI6)
		{
			SPI6_PCLK_EN();
		}

	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}
		else if (pSPIx == SPI6)
		{
			SPI6_PCLK_DI();
		}

	}


}

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//clk enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	//  SPI_CR1 Configuration

	uint32_t tempreg = 0;

	//1. device mode configuration
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPIx_CR1_MSTR ;
	//2. SPI_BUS Configurations
	if ( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidirectional mode should be cleared
		tempreg &= ~( 1 << SPIx_CR1_BIDIMODE );
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidirectional mode should be set
		tempreg |=  ( 1 << SPIx_CR1_BIDIMODE );
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidirectional mode should be cleared
		tempreg &= ~( 1 << SPIx_CR1_BIDIMODE );
		//RX only should be set
		tempreg |= (1 << SPIx_CR1_RXONLY );

	}

	//3. SPI_SclkSpeed configuration
	tempreg |= ( pSPIHandle->SPI_Config.SPI_SclkSpeed << SPIx_CR1_BR ) ;


	//4. CPOL Configuration
	tempreg |= ( pSPIHandle->SPI_Config.SPI_CPOL << SPIx_CR1_CPOL );

	//5. CPHA Configuration
	tempreg |= ( pSPIHandle->SPI_Config.SPI_CPHA << SPIx_CR1_CPHA );

	//SSM enable
	tempreg |= ( pSPIHandle->SPI_Config.SPI_SSM << SPIx_CR1_SSM );

	pSPIHandle->pSPIx->CR1 = tempreg;



	//  SPI_CR2 Configuration
	//6.Data size configuration
		// needs verification   ; TODO
	pSPIHandle->pSPIx->CR2 |= ( pSPIHandle->SPI_Config.SPI_DataSize << SPI_CR2_DS);



}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}


uint8_t SPI_GetFlag_status(SPI_RegDef_t *pSPIx,  uint32_t FlagName)
{
	if (pSPIx->SR & FlagName )
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * Data Send and Receive
 */
void SPI_Send_8Bits_Data(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. wait until TXE is set
		while ( SPI_GetFlag_status(pSPIx,SPI_TXE_FLAG) == FLAG_RESET)
		{

		}

			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
	}
}

/*
void SPI_Send_16Bits_Data(SPI_RegDef_t *pSPIx, uint32_t *pTxBuffer, uint32_t Len)
	{
	     while (Len > 0)
	     	{
		     	//1. wait until TXE is set
		    	while ( SPI_GetFlag_status(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

	         	//1. Load the data in to the DR
	         	pSPIx->DR = *(pTxBuffer);
	        	Len--;
	        	Len--;
	        	pTxBuffer++;

	        }


     }
*/


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}



/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}
void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{

		pSPIx->CR1 |= 1 << SPIx_CR1_SPE;
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPIx_CR1_SPE);

	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{

		pSPIx->CR1 |= 1 << SPIx_CR1_SSI;
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPIx_CR1_SSI);

	}

}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{

		pSPIx->CR2 |= 1 << SPI_CR2_SSOE;
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);

	}
}




