/*
 * main.c
 *
 *  Created on: 24 août 2018
 *      Author: Laurent
 */

#include "stm32f7xx.h"
#include "main.h"
#include "bsp.h"
#include "delay.h"
/*
 * Variable
 */

uint16_t data;

/*
 * Local Static Functions
 */

static uint8_t SystemClock_Config	(void);


/*
 * Project Entry Point
 */

int main(void)
{
	// Configure System Clock for 168MHz from 8MHz HSE
	SystemClock_Config();

	// Initialize LED and USER Button
	BSP_LED_Init();
	BSP_PB_Init();

	// Initialize Debug Console
	BSP_Console_Init();
	adc_init();
	BSP_NVIC_Init();
	my_printf("\r\nConsole Ready!\r\n");
	my_printf("SYSCLK = %d Hz\r\n", SystemCoreClock);

	// Loop forever
	while(1)
	{
		// LED test
		BSP_LED_Toggle(GREEN);
		delay_ms(50);
		BSP_LED_Toggle(BLUE);
		delay_ms(50);
		BSP_LED_Toggle(RED);
		delay_ms(50);

		// USER Button & Console test
		if(BSP_PB_GetState() == 1)
		{
			my_printf("#");
		}
		ADC1->CR2 |= ADC_CR2_SWSTART;

//		delay_ms(200);
	}
}


/*
 * 	Clock configuration for the Nucleo STM32F746ZG board
 * 	HSE input Bypass Mode 				-> 8MHz
 * 	SYSCLK, AHB							-> 216MHz
 * 	APB1								-> 54MHz  (periph) 108MHz (timers)
 * 	APB2								-> 108MHz (periph) 216MHz (timers)
 *
 */

static uint8_t SystemClock_Config(void)
{
	uint32_t	status;
	uint32_t	timeout;

	// Start HSE
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	// Wait until HSE is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_HSERDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (1);	// HSE error


	// Enable the power regulator scale mode 1
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR1 |= PWR_CR1_VOS;

	// Configure the main PLL
	#define PLL_M	8		// 8MHz HSE down-to 1MHz PLL input
	#define PLL_N	432		// 432 MHz VCO output
	#define PLL_P	2		// 216 MHz PLL output
	#define PLL_Q	9		// 48  MHz (USB)

	RCC->PLLCFGR = PLL_M | (PLL_N <<6) | (((PLL_P >> 1) -1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;

	// Enter Over-drive power mode
	PWR->CR1 |= PWR_CR1_ODEN;

	// Wait until OD is ready
	timeout = 1000;

	do
	{
		status = PWR->CSR1 & PWR_CSR1_ODRDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (2);	// PWR error

	// Switch to OD power supply
	PWR->CR1 |= PWR_CR1_ODSWEN;

	// Wait until supply is switched
	timeout = 1000;

	do
	{
		status = PWR->CSR1 & PWR_CSR1_ODSWRDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (2);	// PWR error


	// Configure Flash ART accelerator, prefetch and wait state (increase wait states at 216 MHz)
	FLASH->ACR = FLASH_ACR_ARTEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_4WS;

	// Configure AHB/APB prescalers
	// AHB  Prescaler = /1	-> 216 MHz
	// APB1 Prescaler = /4  -> 54  MHz
	// APB2 Prescaler = /2  -> 108 MHz
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	// Wait until PLL is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (3);	// PLL error


	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL is switched on
	timeout = 1000;

	do
	{
		status = RCC->CFGR & RCC_CFGR_SWS;
		timeout--;
	} while ((status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	if (timeout == 0) return (4);	// SW error


	// Update System core clock
	SystemCoreClockUpdate();
	return (0);
}
