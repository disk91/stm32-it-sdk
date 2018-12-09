/* ==========================================================
 * systick.c - Manage time with systicks
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 12 sept. 2018
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
 * The systick timer is configured by default to be updated
 * every ms.
 *
 * ==========================================================
 */
#include <it_sdk/itsdk.h>
#include <it_sdk/time/time.h>
#include <it_sdk/lowpower/lowpower.h>
#include <stm32l0xx_hal.h>

bool __enable_systick = true;

/**
 * Action to be executed on Systick
 * The name vary depending on the Firwmare version ... ST, you make me crazy !
 */
void HAL_IncTick(void) {
	// add 1ms to the global counter
	if (__enable_systick) itsdk_time_add_us(1000);
	uwTick++;
	__lowPower_wakeup_reason = LOWPWR_WAKEUP_SYSTICK;
}
void HAL_SYSTICK_Callback(void) {
	// add 1ms to the global counter
	if (__enable_systick) itsdk_time_add_us(1000);
	__lowPower_wakeup_reason = LOWPWR_WAKEUP_SYSTICK;
}
