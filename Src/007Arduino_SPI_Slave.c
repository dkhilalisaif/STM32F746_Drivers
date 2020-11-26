/*
 * 006spi_tx_test.c
 *
 *  Created on: 30 Oct 2020
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

void GPIO_Button_Init(void);
void SDK_GPIO_init(void);
void SDK_SPI2_init(void);
void delay(void);



void GPIO_Button_Init(void)
{

	// Button Configuration
	GPIO_Handle_t Button;
	Button.pGPIOx = GPIOI;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_11;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOI,ENABLE);

	GPIO_Init(&Button);

	//IRQ  configuration
	//GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO_15);
	//GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

}
//-1.

void SDK_GPIO_init(void)
{
	GPIO_Handle_t MySPI;
	GPIOB_PCLK_EN();
	GPIOI_PCLK_EN();

	MySPI.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	MySPI.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	MySPI.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_PP;
	MySPI.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	MySPI.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	// miso
	MySPI.pGPIOx = GPIOB;
	MySPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&MySPI);
	//MOSI

	MySPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&MySPI);

	//NSS
	MySPI.pGPIOx = GPIOI;
	MySPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIO_Init(&MySPI);

	//SCK

	MySPI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GPIO_Init(&MySPI);
}
//0.

void SDK_SPI2_init(void)
{
	SPI_Handle_t MySPI2;
	MySPI2.pSPIx = SPI2;
	MySPI2.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	MySPI2.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER ;
	MySPI2.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16;
	MySPI2.SPI_Config.SPI_DataSize = SPI_DS_8BITS;
	MySPI2.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	MySPI2.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	MySPI2.SPI_Config.SPI_SSM = SPI_SSM_DI; //Hardware slave managment

	SPI_Init(&MySPI2);
}

void delay(void)
{
	for(uint32_t i = 0; i< 500000/2 ; i++);
}



int main(void)
{
	/*     Private data declaration     */
	char mydata[] = "Hello World";

	GPIO_Button_Init();

	//1.Initialisation of GPIO PINs that we'll need for our SPI
	SDK_GPIO_init();


	//2.Initialisation of the SPI Peripheral
	SDK_SPI2_init();

	/*
	 * 6. Making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high when SPE =0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);


	//makes NSS Signal internally high and avoids MODF errors
	//SPI_SSIConfig(SPI2,ENABLE);


	while(1)
	{
		//7. wait till Button is clicked
		while( !(GPIO_ReadFromInputPin(GPIOI, GPIO_PIN_11)))
		{

		}





			delay();


			// 4.Enable SPI2 Peripheral
			SPI_Peripheral_Control(SPI2,ENABLE);


			//9. send length information
			uint8_t datalen = strlen(mydata);
			SPI_Send_8Bits_Data(SPI2, &datalen, 1);

			//3.send data
			SPI_Send_8Bits_Data(SPI2,(uint8_t *) mydata, strlen(mydata));

			//8.let's confirm that SPI is not BUSY
			while (SPI_GetFlag_status(SPI2, SPI_BSY_FLAG));

			//5.SPI2 disable
			SPI_Peripheral_Control(SPI2,DISABLE);


	}


	return 0;
}

