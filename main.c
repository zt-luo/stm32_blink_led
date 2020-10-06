/** \file main.c
 * \brief Sample STM32 project
 * \details This file holds a very basic code for STM32F103RB. This code
 * enables all GPIO ports, configures Flash wait-states and enables the PLL
 * to achieve the highest allowed frequency for STM32F103RB (72MHz). Main
 * code block just blinks the LED. The LED port and pin are defined in
 * config.h file. Target core frequency and quartz crystal resonator
 * frequency are defined there as well.
 *
 * \author Freddie Chopin, http://www.freddiechopin.info/
 * \date 2012-01-07
 */

/******************************************************************************
* project: stm32_blink_led
* chip: STM32F103RB
* compiler: arm-none-eabi-gcc (Sourcery CodeBench Lite 2011.09-69) 4.6.1
*
* prefix: (none)
*
* available global functions:
* 	int main(void)
*
* available local functions:
* 	static void flash_latency(uint32_t frequency)
* 	static uint32_t pll_start(uint32_t crystal, uint32_t frequency)
* 	static void system_init(void)
*
* available interrupt handlers:
******************************************************************************/

/*
+=============================================================================+
| includes
+=============================================================================+
*/

#include <stdint.h>

#include "inc/stm32f10x.h"

#include "config.h"

#include "hdr/hdr_rcc.h"
#include "hdr/hdr_gpio.h"

#include "gpio.h"

/*
+=============================================================================+
| module variables
+=============================================================================+
*/

/*
+=============================================================================+
| local functions' declarations
+=============================================================================+
*/

static void flash_latency(uint32_t frequency);
static uint32_t pll_start(uint32_t crystal, uint32_t frequency);
static void system_init(void);

/*
+=============================================================================+
| global functions
+=============================================================================+
*/

/*------------------------------------------------------------------------*//**
* \brief main code block
* \details Call some static initialization functions and blink the led with
* frequency defined via count_max variable.
*//*-------------------------------------------------------------------------*/

int main(void)
{
	volatile uint32_t count, count_max = 1000000;

	system_init();
	pll_start(CRYSTAL, FREQUENCY);

	gpio_pin_cfg(LED_GPIO, LED_pin, GPIO_CRx_MODE_CNF_OUT_PP_10M_value);

	while (1)
	{
		for (count = 0; count < count_max; count++);	// delay
		LED_bb = 1;
		for (count = 0; count < count_max; count++);	// delay
		LED_bb = 0;
	}
}

/*
+=============================================================================+
| local functions
+=============================================================================+
*/

/*------------------------------------------------------------------------*//**
* \brief Configures Flash latency
* \details Configures Flash latency (wait-states) which allows the chip to run
* at higher speeds
*
* \param [in] frequency defines the target frequency of the core
*//*-------------------------------------------------------------------------*/

static void flash_latency(uint32_t frequency)
{
	uint32_t wait_states;

	if (frequency < 24000000ul)				// 0 wait states for core speed below 24MHz
		wait_states = 0;
	else if (frequency < 48000000ul)		// 1 wait state for core speed between 24MHz and 48MHz
		wait_states = 1;
	else									// 2 wait states for core speed over 48MHz
		wait_states = 2;

	FLASH->ACR |= wait_states;				// set the latency
}

/*------------------------------------------------------------------------*//**
* \brief Starts the PLL
* \details Configure and enable PLL to achieve some frequency with some crystal.
* Before the speed change Flash latency is configured via flash_latency(). PLL
* parameter mul is based on both function parameters. The PLL is set up,
* started and connected. APB1 clock ratio is set to 1:2 (max freq = 36MHz)
*
* \param [in] crystal is the frequency of the crystal resonator connected to the
* STM32F103RB
* \param [in] frequency is the desired target frequency after enabling the PLL
*
* \return real frequency that was set
*//*-------------------------------------------------------------------------*/

static uint32_t pll_start(uint32_t crystal, uint32_t frequency)
{
	uint32_t mul;

	RCC_CR_HSEON_bb = 1;					// enable HSE clock
	flash_latency(frequency);				// configure Flash latency for desired frequency

	mul = frequency / crystal;				// PLL multiplier calculation

	if (mul > 16)							// max PLL multiplier is 16
		mul = 16;

	frequency = crystal * mul;

	RCC->CFGR |= ((mul - 2) << RCC_CFGR_PLLMUL_bit) | RCC_CFGR_PLLSRC | RCC_CFGR_PPRE1_DIV2;	// configuration of PLL: HSE x (mul), APB1 clk = /2

	while (!RCC_CR_HSERDY_bb);				// wait for stable clock

	RCC_CR_PLLON_bb = 1;					// enable PLL
	while (!RCC_CR_PLLRDY_bb);				// wait for PLL lock

	RCC->CFGR |= RCC_CFGR_SW_PLL;			// change SYSCLK to PLL
	while (((RCC->CFGR) & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);	// wait for switch

	return frequency;
}

/*------------------------------------------------------------------------*//**
* \brief Initializes system
* \details Enables all GPIO ports
*//*-------------------------------------------------------------------------*/

static void system_init(void)
{
	gpio_init();
}

/*
+=============================================================================+
| ISRs
+=============================================================================+
*/

/******************************************************************************
* END OF FILE
******************************************************************************/
