/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Damjan Marion <damjan.marion@gmail.com>
 * Copyright (C) 2011 Mark Panajotovic <marko@electrontube.org>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f1/flash.h>
void compute();

/* Enable internal high-speed oscillator. */
void set_clk(int mul)
{
	rcc_osc_on(HSI);
	rcc_wait_for_osc_ready(HSI);
	rcc_osc_off(PLL);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	/* Enable external high-speed oscillator 8MHz. */
	rcc_osc_on(HSE);
	rcc_wait_for_osc_ready(HSE);
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

	/*
	* Set prescalers for AHB, ADC, ABP1, ABP2.
	* Do this before touching the PLL (TODO: why?).
	*/
	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);    /* Set. 24MHz Max. 72MHz */
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV2);  /* Set. 12MHz Max. 14MHz */
	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_NODIV);    /* Set. 24MHz Max. 36MHz */
	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);    /* Set. 24MHz Max. 72MHz */
	/*
	* Sysclk runs with 24MHz -> 0 waitstates.
	* 0WS from 0-24MHz
	* 1WS from 24-48MHz
	* 2WS from 48-72MHz
	*/
	flash_set_ws(FLASH_ACR_LATENCY_0WS);
	/*
	* Set the PLL multiplication factor to 3.
	* 8MHz (external) * 3 (multiplier) = 24MHz
	*/
	rcc_set_pll_multiplication_factor(mul);
	/* Select HSE as PLL source. */
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);
	/*
	* External frequency undivided before entering PLL
	* (only valid/needed for HSE).
	*/
	rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);
	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(PLL);
	rcc_wait_for_osc_ready(PLL);
	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);
	/* Set the peripheral clock frequencies used */
	rcc_ppre1_frequency = 24000000;
	rcc_ppre2_frequency = 24000000;
}

/* Set STM32 to 24 MHz. */

int clk = 0;

static void clock_setup(void)
{
	set_clk(clk);

	/* Enable GPIOC clock. */
	// rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
    // rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
}

static void gpio_setup(void)
{
	/* Set GPIO8/9 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
}

static void exti_setup()
{
	exti_select_source(GPIO0, GPIOA);
	exti_set_trigger(GPIO0, EXTI_TRIGGER_RISING);
	exti_enable_request(GPIO0);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);
	nvic_enable_irq(NVIC_EXTI0_IRQ);
}

int main(void)
{
	int i;

    // Wait for a while
	for (i = 0; i < 5000000; i++)
	{
		__asm__("nop");
	}

	clock_setup();
	gpio_setup();
	exti_setup();

	SCB_SCR |= 4; // Set deep sleep

	while(1)
	{
		__asm__("wfi");
	}

	return 0;
}

void exti0_isr()
{
	int i =0;

    
	for(i = 0;i < 200; ++i)
		compute();
	set_clk(0);
	for(i = 0;i < 200; ++i)
		compute();
    exti_reset_request(EXTI0);
}

int ms = 0;

volatile int comp_val[1024] = {0};

void compute()
{
	int i;

	for(i = 2;i < 1024; ++i)
	{
		comp_val[i] = comp_val[i-1] * 3 + comp_val[i-2] * comp_val[i-1] + 1;
	}
}

