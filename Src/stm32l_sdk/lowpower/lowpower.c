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
 * - LPUART WakeUp => clock mode need to be HSI or LSE, max speed 9600 for LSE
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
#if ( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_I2C2 ) > 0 || ( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_I2C1 )
#include "i2c.h"
#endif

#ifndef ITSDK_LOWPOWER_MISC_HALT
#error "You must set ITSDK_LOWPOWER_MISC_HALT in itsdk/config.h"
#endif

#if (ITSDK_DEVICE == __DEVICE_STM32L072XX) && (ITSDK_LOWPOWER_MOD &__LOWPWR_MODE_WAKE_LPUART) > 0
#error "STM32L0172 does not support LPUART WakeUp (or tells me what's wrong)"
#endif

#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0
	void __LP_GPIO_IRQHandler(uint16_t GPIO_Pin);
	gpio_irq_chain_t __lowpwer_gpio_irq = {
			__LP_GPIO_IRQHandler,
			0,
			NULL
	};
	uint16_t __lowPower_wakeup_pin =0;
#endif


/**
 * Setup the STM32L Low Power mode for the given amount of ms
 * 0xFFFFFFFF ms when no time limit
 */
stm32l_lowPowerReturn_e __attribute__((optimize("O3"))) stm32l_lowPowerSetup(uint32_t durationMs, stm32_lowPowerMode_e mode) {

	if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_STOP ) {
		// -------------------------------------------------------------
		// Configure the STM32L0x1 for switching to low power stop mode
		// -------------------------------------------------------------
		#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_RTC ) > 0
			if ( durationMs == __INFINITE_32B ) {
				durationMs = STM32L_LOWPOWER_MAXDURATION_MS;
				#if ITSDK_WDG_MS > 0
				  if ( durationMs > ITSDK_WDG_MS ) {
					  durationMs = ITSDK_WDG_MS - 5;
				  }
				#endif
			}
		    if ( durationMs > STM32L_MINIMUM_SLEEPDURATION_MS ) {
			   rtc_configure4LowPower(durationMs);						// Setup RTC wake Up
		    } else {
			   return STM32L_LOWPOWER_TOOSHORT;
			}
		#endif
		HAL_SuspendTick();
	    __HAL_RCC_PWR_CLK_ENABLE();				// Enable Power Control clock
 	    HAL_PWREx_EnableUltraLowPower();		// Ultra low power mode
 	    HAL_PWREx_EnableFastWakeUp();			// Fast wake-up for ultra low power mode

 	    if ( mode == STM32L_LOWPOWER_NORMAL_STOP ) {

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

			#if  ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_ALLUART ) > 0
				UART_WakeUpTypeDef wakeup;
			#endif

			#if  ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_LPUART ) > 0
				// make sure that UART is ready to receive
				while(__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_REACK) == RESET){}

				wakeup.WakeUpEvent=UART_WAKEUP_ON_READDATA_NONEMPTY; // UART_WAKEUP_ON_STARTBIT
				HAL_UARTEx_StopModeWakeUpSourceConfig(&hlpuart1,wakeup);
				__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_WUF);
				HAL_UARTEx_EnableStopMode(&hlpuart1);
			#else
			  #if (ITSDK_WITH_UART & __UART_LPUART1) > 0
				__HAL_RCC_LPUART1_CLK_DISABLE();
			  #endif
			#endif


			#if  ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_UART2 ) > 0
				// make sure that UART is ready to receive
				while(__HAL_UART_GET_FLAG(&huart2, USART_ISR_REACK) == RESET){}

				wakeup.WakeUpEvent=UART_WAKEUP_ON_READDATA_NONEMPTY; // UART_WAKEUP_ON_STARTBIT
				HAL_UARTEx_StopModeWakeUpSourceConfig(&huart2,wakeup);
				__HAL_UART_ENABLE_IT(&huart2, UART_IT_WUF);
				HAL_UARTEx_EnableStopMode(&huart2);
			#else
			  #if (ITSDK_WITH_UART & __UART_USART2) > 0
				__HAL_RCC_USART2_CLK_DISABLE();
			  #endif
			#endif

			#if  ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_UART1 ) > 0
				// make sure that UART is ready to receive
				while(__HAL_UART_GET_FLAG(&huart1, USART_ISR_REACK) == RESET){}

				wakeup.WakeUpEvent=UART_WAKEUP_ON_READDATA_NONEMPTY; // UART_WAKEUP_ON_STARTBIT
				HAL_UARTEx_StopModeWakeUpSourceConfig(&huart1,wakeup);
				__HAL_UART_ENABLE_IT(&huart1, UART_IT_WUF);
				HAL_UARTEx_EnableStopMode(&huart1);
			#else
				#if (ITSDK_WITH_UART & __UART_USART1) > 0
					__HAL_RCC_USART1_CLK_DISABLE();
				#endif
			#endif

			_stm32l_disableGpios();					// Disable GPIOs based on configuration

			#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0
			  // Register interrupt handler
			  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); 				// Clear wakeUp flag
			  gpio_registerWakeUpAction(&__lowpwer_gpio_irq);	// Install the wakeup handler
																// (the previously existing handler will be bypassed)
			#else
			  gpio_interruptDisableAll();						// Disable GPIOs interrupts
			#endif

			#if ( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_I2C1 ) > 0
				__HAL_RCC_I2C1_CLK_DISABLE();
			#endif
			#if ( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_I2C2 ) > 0
				__HAL_RCC_I2C2_CLK_DISABLE();
			#endif
			#if ( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_SPI1 ) > 0
				__HAL_RCC_SPI1_CLK_DISABLE();
			#endif
			#if ( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_SPI2 ) > 0
				__HAL_RCC_SPI1_CLK_DISABLE();
			#endif
			#if( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_TIM21 ) > 0
				__HAL_RCC_TIM21_CLK_DISABLE();
			#endif
			#if( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_ADC1 ) > 0
				__HAL_RCC_ADC1_CLK_DISABLE();
			#endif
 	    }
		// ??? add ?? Not yet tested
		//__HAL_FLASH_SLEEP_POWERDOWN_ENABLE();


 	    // Switch to STOPMode
		__lowPower_wakeup_reason=LOWPWR_WAKEUP_UNDEF;
		#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0
			__lowPower_wakeup_pin=0xFFFF;
		#endif
 	    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	}
	return STM32L_LOWPOWER_SUCCESS;
}


stm32l_lowPowerReturn_e __attribute__((optimize("O3"))) stm32l_lowPowerResume(stm32_lowPowerMode_e mode) {

	if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_STOP ) {
		// ------------------------------------------------------------
		// Restore from STOP MODE
		// ------------------------------------------------------------

		SystemClock_Config();
		HAL_SuspendTick();
		#if  ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_RTC ) > 0
			rtc_disable4LowPower();
		#endif

		if ( mode == STM32L_LOWPOWER_NORMAL_STOP ) {

			stm32l_lowPowerRestoreGpioConfig();

			#if ( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_I2C1 ) > 0
				HAL_I2C_MspInit(&hi2c1);
			#endif
			#if ( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_I2C2 ) > 0
				HAL_I2C_MspInit(&hi2c2);
			#endif
			#if ( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_SPI1 ) > 0
				HAL_SPI_MspInit(&hspi1);
			#endif
			#if ( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_SPI2 ) > 0
				HAL_SPI_MspInit(&hspi2);
			#endif
			#if( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_TIM21 ) > 0
				HAL_TIM_Base_MspInit(&htim21);
			#endif
			#if( ITSDK_LOWPOWER_MISC_HALT & __LP_HALT_ADC1 ) > 0
				__HAL_RCC_ADC1_CLK_ENABLE();
				HAL_ADCEx_EnableVREFINT();
			#endif
			#if (( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_LPUART ) == 0) && (( ITSDK_WITH_UART & __UART_LPUART1 ) > 0)
				// Reinit LPUart
				HAL_UART_MspInit(&hlpuart1);
				MX_LPUART1_UART_Init();
			#endif
			#if (( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_UART1 ) == 0) && (( ITSDK_WITH_UART & __UART_USART1 ) > 0 )
				HAL_UART_MspInit(&huart1);
				MX_USART1_UART_Init();
			#endif
			#if (( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_UART2 ) == 0) && (( ITSDK_WITH_UART & __UART_USART2 ) > 0 )
				HAL_UART_MspInit(&huart2);
				MX_USART2_UART_Init();
			#endif

			#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0
			 // remove the WakeUp Handler and fire the pending irq to the normal IRQ handler
			 gpio_removeWakeUpAction();
			 if ( __lowPower_wakeup_reason == LOWPWR_WAKEUP_GPIO && __lowPower_wakeup_pin != 0xFFFF ) {
				HAL_GPIO_EXTI_Callback(__lowPower_wakeup_pin);
			 }
			#endif
		}

	}
	//  useful line of code to identify the wakeup cause when needed...
	#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_LOWPOWER ) > 0
	if (__lowPower_wakeup_reason != LOWPWR_WAKEUP_UNDEF ) {
		if ( __lowPower_wakeup_reason == LOWPWR_WAKEUP_GPIO )
         #if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0
			log_info("-%d-(%d)-",__lowPower_wakeup_reason,__lowPower_wakeup_pin);
         #else
		    log_info("!");
         #endif
		else
			log_info("-%d-",__lowPower_wakeup_reason);
	} else {
		log_info("|");
	}
	#endif

	HAL_ResumeTick();
	return STM32L_LOWPOWER_SUCCESS;
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
 * We store the pin, reason of the wake up because we are going to callback the irq handler
 * once the mcu is reconfigured & ready
 */

#if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0

void __LP_GPIO_IRQHandler(uint16_t GPIO_Pin) {

  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  __lowPower_wakeup_reason=LOWPWR_WAKEUP_GPIO;
  __lowPower_wakeup_pin = GPIO_Pin;

}

#endif

#if  ( ITSDK_LOWPOWER_MOD & ( __LOWPWR_MODE_WAKE_LPUART | __LOWPWR_MODE_WAKE_UART2 | __LOWPWR_MODE_WAKE_UART1 )  ) > 0
void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart) {
   __lowPower_wakeup_reason=LOWPWR_WAKEUP_UART;
}

#endif

