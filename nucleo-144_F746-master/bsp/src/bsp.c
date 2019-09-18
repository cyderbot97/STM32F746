/*
 * bsp.c
 *
 *  Created on: 5 août 2017
 *      Author: Laurent
 */

#include "bsp.h"

/*
 *
 */
void BSP_NVIC_Init(){

	NVIC_SetPriority(ADC_IRQn, 1);
	NVIC_EnableIRQ(ADC_IRQn);

}

/*
 * ADC
 *
 */
void adc_init(void)
{
	//Enable ADC1 clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	//Enable gpioA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//configure PA0 as analog
	GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;
	GPIOA->MODER |= (0x03 << GPIO_MODER_MODER0_Pos);


	//Set Regular channel sequence length
	ADC1->SQR1 &= ~ADC_SQR1_L_Msk;
	ADC1->SQR1 |= ~ADC_SQR1_L_0;


	//Set continuous conversion mode
	//ADC1->CR2 |= ADC_CR2_CONT;		-> too fast

	//12-bits resolution
	ADC1->CR1 &= ~ADC_CR1_RES_Msk;

	//Sampling time = 15 cycles
	ADC1->SMPR1 = 0x07;

	//Enable interrupt
	ADC1->CR1 |= ADC_CR1_EOCIE;

	ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC1
	//while(!ADC1->ISR & ADC_ISR_ADRD); // wait for ADRDY
	ADC1->CR2 |= ADC_CR2_SWSTART;

}


/*
 * BSP_LED_Init()
 * Initialize LED pin (PB0, PB7, PB14) as  High-Speed Push-Pull Outputs
 * Set LED initial state to OFF
 */

void BSP_LED_Init()
{
	// Enable GPIOB clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// Configure PB0, PB7, PB14 as output
	GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER14_Msk);
	GPIOB->MODER |=  (0x01 <<GPIO_MODER_MODER0_Pos) | (0x01 <<GPIO_MODER_MODER7_Pos) | (0x01 <<GPIO_MODER_MODER14_Pos);

	// Configure PB0, PB7, PB14 as Push-Pull output
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0 |GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_14);

	// Configure PB0, PB7, PB14 as High-Speed Output
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0_Msk | GPIO_OSPEEDER_OSPEEDR7_Msk | GPIO_OSPEEDER_OSPEEDR14_Msk);
	GPIOB->OSPEEDR |=  (0x03 <<GPIO_OSPEEDER_OSPEEDR0_Pos) | (0x03 <<GPIO_OSPEEDER_OSPEEDR7_Pos) | (0x03 <<GPIO_OSPEEDER_OSPEEDR14_Pos);

	// Disable PB0, PB7, PB14 Pull-up/Pull-down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk | GPIO_PUPDR_PUPDR7_Msk | GPIO_PUPDR_PUPDR14_Msk);

	// Set Initial State OFF
	GPIOB->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_7 | GPIO_BSRR_BR_14;
}

/*
 * BSP_LED_On()
 * Turn LED On
 */

void BSP_LED_On(uint8_t id)
{
	switch (id)
	{
		case 0:
		{
			GPIOB->BSRR = GPIO_BSRR_BS_0;
			break;
		}

		case 1:
		{
			GPIOB->BSRR = GPIO_BSRR_BS_7;
			break;
		}

		case 2:
		{
			GPIOB->BSRR = GPIO_BSRR_BS_14;
			break;
		}
	}
}

/*
 * BSP_LED_Off()
 * Turn LED Off
 */

void BSP_LED_Off(uint8_t id)
{
	switch (id)
	{
		case 0:
		{
			GPIOB->BSRR = GPIO_BSRR_BR_0;
			break;
		}

		case 1:
		{
			GPIOB->BSRR = GPIO_BSRR_BR_7;
			break;
		}

		case 2:
		{
			GPIOB->BSRR = GPIO_BSRR_BR_14;
			break;
		}
	}
}

/*
 * BSP_LED_Toggle()
 * Toggle LED
 */

void BSP_LED_Toggle(uint8_t id)
{
	switch (id)
	{
		case 0:
		{
			GPIOB->ODR ^= GPIO_ODR_ODR_0;
			break;
		}

		case 1:
		{
			GPIOB->ODR ^= GPIO_ODR_ODR_7;
			break;
		}

		case 2:
		{
			GPIOB->ODR ^= GPIO_ODR_ODR_14;
			break;
		}
	}
}


/*
 * BSP_PB_Init()
 * Initialize Push-Button pin (PC13) as input without Pull-up/Pull-down
 */

void BSP_PB_Init()
{
	// Enable GPIOC clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	// Configure PC13 as input
	GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk;
	GPIOC->MODER |= (0x00 <<GPIO_MODER_MODER13_Pos);

	// Disable PC13 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR13_Msk;
}

/*
 * BSP_PB_GetState()
 * Returns the state of the button (0=released, 1=pressed)
 */

uint8_t	BSP_PB_GetState()
{
	uint8_t state;

	if ((GPIOC->IDR & GPIO_IDR_IDR_13) == GPIO_IDR_IDR_13)
	{
		state = 1;
	}
	else
	{
		state = 0;
	}

	return state;
}


/*
 * BSP_Console_Init()
 * USART3 @ 115200 Full Duplex
 * 1 start - 8-bit - 1 stop
 * TX -> PD8 (AF7)
 * RX -> PD9 (AF7)
 */

void BSP_Console_Init()
{
	// Enable GPIOD clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

	// Configure PD8, PD9 as AF mode
	GPIOD->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
	GPIOD->MODER |=  (0x02 <<GPIO_MODER_MODER8_Pos) | (0x02 <<GPIO_MODER_MODER9_Pos);

	// Connect to USART3 TX (AF7)
	GPIOD->AFR[1] &= ~(0x000000FF);
	GPIOD->AFR[1] |=   0x00000077;

	// Enable USART3 Clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	// Disable USART3 and clears CR1 register
	// Default is 1S|8B|1S, no parity
	USART3->CR1 = 0x00000000;
	USART3->CR2 = 0x00000000;
	USART3->CR3 = 0x00000000;

	// Enable Receiver and Transmitter mode
	USART3->CR1 |= USART_CR1_RE | USART_CR1_TE;

	// Baudrate = 115200
	USART3->CR1 &= ~USART_CR1_OVER8;
	USART3->BRR = 0x1D4;

	// Enable USART3
	USART3->CR1 |= USART_CR1_UE;
}


