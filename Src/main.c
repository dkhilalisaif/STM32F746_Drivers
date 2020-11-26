#include "stm32f746xx.h"



void delay(void)
{
	for(uint32_t i=0; i<1000000; i++ );
}




int main(void)
{
	GPIO_Handle_t MyGPIO;
	MyGPIO.pGPIOx = GPIOI;
	MyGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	MyGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	MyGPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	MyGPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	MyGPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOI,ENABLE);
	GPIO_Init(&MyGPIO);


	while(1)
	{
		GPIO_ToggleOutputPin(GPIOI,GPIO_PIN_1);
		delay();



	}

	return 0;


	void EXTI0_IRQHandler(void)
	{
		//Handle the interrupt
		GPIO_IRQHandling(0);
	}

}




