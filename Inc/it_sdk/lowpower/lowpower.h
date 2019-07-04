/* ==========================================================
 * lowpower.h - header to manage lowpower
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

#ifndef IT_SDK_LOWPOWER_LOWPOWER_H_
#define IT_SDK_LOWPOWER_LOWPOWER_H_

typedef enum {
	LOWPWR_WAKEUP_RTC = 0,
	LOWPWR_WAKEUP_GPIO,
	LOWPWR_WAKEUP_SYSTICK,
	LOWPWR_WAKEUP_UART,
	LOWPWR_WAKEUP_UNDEF = 255,
} lowPower_wu_reason_t;
extern lowPower_wu_reason_t __lowPower_wakeup_reason;

typedef enum {
	LOWPRW_ENABLE = 0,
	LOWPRW_DISABLE
} lowPower_state_e;

void lowPower_switch();
uint32_t lowPower_delayMs(uint32_t duration);

// ------------------------------------------------------------------------
// LowPower enable / disable
void lowPower_enable();
void lowPower_disable();

#endif /* IT_SDK_LOWPOWER_LOWPOWER_H_ */
