/* ==========================================================
 * lowpower.c - Lowpower sdk functions
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 2 sept. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018  IngeniousThings and Disk91
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
#include <it_sdk/config.h>
#include <it_sdk/lowpower/lowpower.h>
#include <it_sdk/time/time.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0x1 || ITSDK_PLATFORM == __PLATFORM_STM32L0x3
	#include <stm32l_sdk/lowpower/lowpower.h>
	#include <stm32l_sdk/rtc/rtc.h>
#endif


/**
 * Switch to low power mode selected for the expected platform
 */
void lowPower_switch() {

	#if ITSDK_PLATFORM == __PLATFORM_STM32L0x1 || ITSDK_PLATFORM == __PLATFORM_STM32L0x3
		stm32l_lowPowerSetup();
		// sleeping
		stm32l_lowPowerResume();
	#endif

}




