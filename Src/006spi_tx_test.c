 /*
 * 006spi_tx_test.c
 *
 *  Created on: 27 Oct 2020
 *      Author: saif
 */
/*      includes          */
#include "stm32f746xx.h"
#include <string.h>

/*
 * PB14 ----> MISO
 * PB15 ----> MOSI
 * PI0 -----> NSS
 * PI1  ----> SCK
 * AF mode -> 5
 */

void SDK_GPIO_init(void)
{
	GPIO_Handle_t MySPI;
	GPIOB_PCLK_EN();
	GPIOI_PCLK_EN();

	MySPI.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	MySPI.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	MySPI.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_PP;
	MySPI.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	MySPI.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	//MISO
	MySPI.pGPIOx = GPIOB;
	MySPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&MySPI);

	//MOSI
	MySPI.pGPIOx = GPIOB;
	MySPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&MySPI);

	//NSS
	MySPI.pGPIOx = GPIOI;
	MySPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIO_Init(&MySPI);

	//SCK
	MySPI.pGPIOx = GPIOI;
	MySPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GPIO_Init(&MySPI);
}

void SDK_SPI2_init(void)
{
	SPI_Handle_t MySPI2;
	MySPI2.pSPIx = SPI2;
	MySPI2.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	MySPI2.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER ;
	MySPI2.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	MySPI2.SPI_Config.SPI_DataSize = SPI_DS_8BITS;
	MySPI2.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	MySPI2.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	MySPI2.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&MySPI2);
}
int main(void)
{
	/*     Private data declaration     */
	char mydata[] = "Hello World";

	//Initialisation of GPIO PINs that we'll need for our SPI
	SDK_GPIO_init();


	//Initialisation of the SPI Peripheral
	SDK_SPI2_init();


	//makes NSS Signal internally high and avoids MODF errors
	SPI_SSIConfig(SPI2,ENABLE);


	// Enable SPI2 Peripheral
	SPI_Peripheral_Control(SPI2,ENABLE);


	//send data
	SPI_Send_8Bits_Data(SPI2,(uint8_t *) mydata, strlen(mydata));

	while(1)
	{


	}


	return 0;
}


