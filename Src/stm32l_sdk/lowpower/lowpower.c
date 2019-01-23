/* ==========================================================
 * lowpower.c - lowPower implementation for stm32L0x1
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 2 sept. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 Disk91
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU LESSER General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * ----------------------------------------------------------
 * STOP MODE CONFIGURATION REQUIREMENT
 * - LPUART WakeUp => clock mode need to be HSI, max speed 9600
 * - RTC WakeUp => Activate ClkSource, Calendar, Internal WakeUp, Clk config : LSI, RTC/NVIC => Interrupt activated
 * - GPIO WakeUp => Activate the GPIO as ExtInterrupt, set with Pull & Trigger en Fall/Rise, activate NVIC EXI1_15
 *
 * ==========================================================
 */

#include <stm32l_sdk/lowpower/lowpower.h>
#include <stm32l_sdk/rtc/rtc.h>
#include <it_sdk/config.h>
#include <it_sdk/lowpower/lowpower.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/wrappers.h>
#include "stm32l0xx_hal.h"
#include "usart.h"
#include "gpio.h"


#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0
	void __LP_GPIO_IRQHandler(uint16_t GPIO_Pin);
	gpio_irq_chain_t __lowpwer_gpio_irq = {
			__LP_GPIO_IRQHandler,
			0,
			NULL
	};

#endif


/**
 * Setup the STM32L Low Power mode
 */
void stm32l_lowPowerSetup() {

	if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_STOP ) {
		// -------------------------------------------------------------
		// Configure the STM32L0x1 for switching to low power stop mode
		// -------------------------------------------------------------
		HAL_SuspendTick();
		#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_RTC )
			rtc_configure4LowPower(ITSDK_LOWPOWER_RTC_MS);		// Setup RTC wake Up
		#endif
	    __HAL_RCC_PWR_CLK_ENABLE();				// Enable Power Control clock
 	    HAL_PWREx_EnableUltraLowPower();		// Ultra low power mode
 	    HAL_PWREx_EnableFastWakeUp();			// Fast wake-up for ultra low power mode

		#if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
	        // make sure that no UART transfer is on-going
	        while(__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_BUSY) == SET){};
		#endif

		#if ( ITSDK_WITH_UART & __UART_USART1 ) > 0
			// make sure that no UART transfer is on-going
			while(__HAL_UART_GET_FLAG(&huart1, USART_ISR_BUSY) == SET){};
		#endif

		#if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
			// make sure that no UART transfer is on-going
			while(__HAL_UART_GET_FLAG(&huart2, USART_ISR_BUSY) == SET){};
		#endif

		#if  ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_LPUART ) > 0
	        // make sure that UART is ready to receive
	        while(__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_REACK) == RESET){};

			UART_WakeUpTypeDef wakeup;
    	    wakeup.WakeUpEvent=UART_WAKEUP_ON_STARTBIT; // UART_WAKEUP_ON_READDATA_NONEMPTY
		 	HAL_UARTEx_StopModeWakeUpSourceConfig(&hlpuart1,wakeup);
		 	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_WUF);
		 	HAL_UARTEx_EnableStopMode(&hlpuart1);
		#endif

		_stm32l_disableGpios();					// Disable GPIOs based on configuration

		#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0
		  // Register interrupt handler
		  gpio_registerIrqAction(&__lowpwer_gpio_irq);	// Install the action as the irq can be activated outside this code
		#endif

 	    // Switch to STOPMode
		__lowPower_wakeup_reason=LOWPWR_WAKEUP_UNDEF;
 	    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	}


}


void stm32l_lowPowerResume() {

	if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_STOP ) {
		// ------------------------------------------------------------
		// Restore from STOP MODE
		// ------------------------------------------------------------
		SystemClock_Config();
		#if  ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_RTC ) > 0
			rtc_disable4LowPower();
		#endif
		stm32l_lowPowerRestoreGpioConfig();
		#if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
			// Reinit LPUart
			HAL_UART_MspInit(&hlpuart1);
			MX_LPUART1_UART_Init();
		#endif
		#if ( ITSDK_WITH_UART & __UART_USART1 ) > 0
			HAL_UART_MspInit(&huart1);
			MX_USART1_UART_Init();
		#endif
		#if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
			HAL_UART_MspInit(&huart2);
			MX_USART2_UART_Init();
		#endif
		HAL_ResumeTick();
	}
//  useful line of code to identify the wakeup cause when needed...
//	if (__lowPower_wakeup_reason != LOWPWR_WAKEUP_UNDEF ) {
//		log_info("-%d-",__lowPower_wakeup_reason);
//	} else {
//		log_info("|");
//	}

}

/**
 * Restore the GPIO Configuration after waking up
 * Basically call the MX_GPIO_Init(); function
 * this can be overided in the main program when the
 * gpio is dynamically modified in the code.
 */
__weak void stm32l_lowPowerRestoreGpioConfig() {
	MX_GPIO_Init();
}

/**
 * Switch the GPIO to Low Power
 */
void __GpioAnalog(GPIO_TypeDef  *GPIOx, uint16_t pins)
{
  uint32_t position = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t temp = 0x00U;

  // Configure the port pins //
  while ((pins >> position) != 0)
  {
    iocurrent = (pins) & (1U << position);
    if(iocurrent)
    {
      // Configure IO Direction mode (Input, Output, Alternate or Analog)
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODE0 << (position * 2U));
      temp |= ((GPIO_MODE_ANALOG) << (position * 2U));
      GPIOx->MODER = temp;

      // Activate the Pull-up or Pull down resistor for the current IO
      temp = GPIOx->PUPDR;
      temp &= ~(GPIO_PUPDR_PUPD0 << (position * 2U));
      temp |= ((GPIO_NOPULL) << (position * 2U));
      GPIOx->PUPDR = temp;
    }
    position++;
  }
}

/**
 * Disable GPIOS for Low Power switching
 */
void _stm32l_disableGpios() {
//	GPIO_InitTypeDef GPIO_InitStructure = {0};

	/* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) 	*/
    /* Note: Debug using ST-Link is not possible during the execution of this   		*/
    /*       example because communication between ST-link and the device       		*/
    /*       under test is done through UART. All GPIO pins are disabled (set   		*/
    /*       to analog input mode) including  UART I/O pins.           					*/

#ifdef GPIOA

	__GpioAnalog(GPIOA,(~ITSDK_LOWPOWER_GPIO_A_KEEP) & (GPIOA_PIN_AVAILABLE));
	if( ITSDK_LOWPOWER_GPIO_A_KEEP == __LP_GPIO_NONE ) {
		__HAL_RCC_GPIOA_CLK_DISABLE();
	}
#endif

#ifdef GPIOB
	__GpioAnalog(GPIOB,(~ITSDK_LOWPOWER_GPIO_B_KEEP) & (GPIOB_PIN_AVAILABLE));
	if( ITSDK_LOWPOWER_GPIO_B_KEEP == __LP_GPIO_NONE ) {
		__HAL_RCC_GPIOB_CLK_DISABLE();
	}
#endif

#ifdef GPIOC
	__GpioAnalog(GPIOC,(~ITSDK_LOWPOWER_GPIO_C_KEEP) & (GPIOC_PIN_AVAILABLE));
	if( ITSDK_LOWPOWER_GPIO_C_KEEP == __LP_GPIO_NONE ) {
		__HAL_RCC_GPIOC_CLK_DISABLE();
	}
#endif

#ifdef GPIOD
	__GpioAnalog(GPIOD,(~ITSDK_LOWPOWER_GPIO_D_KEEP) & (GPIOD_PIN_AVAILABLE));
	if( ITSDK_LOWPOWER_GPIO_D_KEEP == __LP_GPIO_NONE ) {
		__HAL_RCC_GPIOD_CLK_DISABLE();
	}
#endif

#ifdef GPIOE
	__GpioAnalog(GPIOE,(~ITSDK_LOWPOWER_GPIO_E_KEEP) & (GPIOE_PIN_AVAILABLE));
	if( ITSDK_LOWPOWER_GPIO_E_KEEP == __LP_GPIO_NONE ) {
		__HAL_RCC_GPIOE_CLK_DISABLE();
	}
#endif

#ifdef GPIOH
	__GpioAnalog(GPIOH,(~ITSDK_LOWPOWER_GPIO_H_KEEP) & (GPIOH_PIN_AVAILABLE));
	if( ITSDK_LOWPOWER_GPIO_H_KEEP == __LP_GPIO_NONE ) {
		__HAL_RCC_GPIOH_CLK_DISABLE();
	}
#endif

}

/**
 * IRQ Handler
 */

#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0

void __LP_GPIO_IRQHandler(uint16_t GPIO_Pin) {

  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  __lowPower_wakeup_reason=LOWPWR_WAKEUP_GPIO;

}

#endif


/**
 * Configure the STM32L0x1 for switching to low power sleep mode
 * This is an activation of the sleep mode with activation of the
 * low power mode of the regulator. Wakeup time 32uS
 * FHclk = 262 KHz
 * In this mode we have the following
 * Peripheral OFF
 *  - CPU
 *  - HSI
 *  - I2C
 *  - ADC
 * Peripheral ON
 *  - RAM
 *  - Backup Register
 *  - Power On/Down Reset
 *  - MSI
 *  - Interconnect controler
 * Peripheral OPTIONNALLY ON (x = selected)
 *  - Flash
 *  - EEprom
 *  - BOR
 *  - DMA
 *  - PVD
 *  - HSE
 *  - LSI
 *  - LSE
 *  - RTC				(x)
 *  - RTC Tamper
 *  - AutoWakeUp
 *  - USART
 *  - LPUART			(x)	1,6uA
 *  - SPI
 *  - TEMP
 *  - CMP
 *  - TIMER
 *  - IWDG
 *  - WWDG				(x) 0,48uA
 *  - Systick Timer
 *  - GPIOs	A			(x)	0,64uA
 *  - GPIOs B			(x) 0,64uA
 *  - GPIOs C			(?) 1,70uA
 *  - GPIOs H			(?) 0,12uA
 */
//void stm32l_lowPowerInit2() {


//}

