/* ==========================================================
 * misc_wrapper.c - 
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 sept. 2018
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
 * Wrapper for different usage
 *
 * ==========================================================
 */

#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0

#include <it_sdk/wrappers.h>
#include "stm32l0xx_hal.h"

/**
 * Reset the device
 */
void itsdk_reset() {
	NVIC_SystemReset();
}

/**
 * Reset Cause
 */
itsdk_reset_cause_t itsdk_getResetCause() {
	if ( RCC->CSR & RCC_CSR_LPWRRSTF ) return RESET_CAUSE_LOWPOWER;
	if ( RCC->CSR & RCC_CSR_WWDGRSTF ) return RESET_CAUSE_WWDG;
	if ( RCC->CSR & RCC_CSR_IWDGRSTF ) return RESET_CAUSE_IWDG;
	if ( RCC->CSR & RCC_CSR_SFTRSTF ) return RESET_CAUSE_SOFTWARE;
	if ( RCC->CSR & RCC_CSR_PORRSTF ) return RESET_CAUSE_POWER_ON;
	if ( RCC->CSR & RCC_CSR_PINRSTF ) return RESET_CAUSE_RESET_PIN;
	if ( RCC->CSR & RCC_CSR_OBLRSTF ) return RESET_CAUSE_LOWPOWER;
	else return RESET_CAUSE_UNKNONW;
}

void itsdk_cleanResetCause() {
	RCC->CSR |= RCC_CSR_RMVF;
}

/**
 * Delay in ms
 */
void itsdk_delayMs(uint32_t ms) {
	HAL_Delay(ms);
}


#endif
