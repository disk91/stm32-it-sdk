/* ==========================================================
 * lowpower.h - stm32L0x1 low power header
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
 * 
 *
 * ==========================================================
 */

#ifndef STM32L_SDK_LOWPOWER_LOWPOWER_H_
#define STM32L_SDK_LOWPOWER_LOWPOWER_H_
#include <stdint.h>

typedef enum {
	STM32L_LOWPOWER_SUCCESS = 0,
	STM32L_LOWPOWER_TOOSHORT,		// The sleep delay sound too short it is better to keep awake

	STM32L_LOWPOWER_ERROR
} stm32l_lowPowerReturn_e;

typedef enum {
	STM32L_LOWPOWER_NORMAL_STOP = 0,	// the normal mode for deep sleep with a lot of wake up and optim posssibilties
	STM32L_LOWPOWER_RTCONLY_STOP,		// The simplified low power mode with no wakeup other than GPS and all peripheral unchanged


	STM32_LOWPOWER_END
} stm32_lowPowerMode_e;

#define STM32L_MINIMUM_SLEEPDURATION_MS		5
#define STM32L_LOWPOWER_MAXDURATION_MS		(8*3600*1000)		// wake up every 8h

// Public functions
stm32l_lowPowerReturn_e stm32l_lowPowerSetup(uint32_t durationMs,stm32_lowPowerMode_e mode);
stm32l_lowPowerReturn_e stm32l_lowPowerResume(stm32_lowPowerMode_e mode);
uint32_t lowPower_delayMs(uint32_t duration);
void stm32l_lowPowerRestoreGpioConfig();

// Private functions
void _stm32l_disableGpios();


#endif /* STM32L_SDK_LOWPOWER_LOWPOWER_H_ */
