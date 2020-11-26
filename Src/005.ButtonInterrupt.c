
#include "stm32f746xx.h"






int main(void)
{
	//LED Pin Configuration
	GPIO_Handle_t LED;
	LED.pGPIOx = GPIOI;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_LOW;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&LED);


	// Button Configuration
	GPIO_Handle_t Button;
	Button.pGPIOx = GPIOI;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_11;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOI,ENABLE);

	GPIO_Init(&Button);

	//IRQ C configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);


	while(1)
	{

	}
}


void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_11);
	GPIO_ToggleOutputPin(GPIOI, GPIO_PIN_1);


}









