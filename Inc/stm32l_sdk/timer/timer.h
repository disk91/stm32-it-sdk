/* ==========================================================
 * timer.h - stm32L0x1 timer header
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 26 nov. 2018
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
#ifndef STM32L_SDK_TIMER_TIMER_H_
#define STM32L_SDK_TIMER_TIMER_H_

#include <it_sdk/config.h>
#include <it_sdk/time/timer.h>

itsdk_timer_return_t stm32l_hwtimer_sync_run(
		uint32_t ms,								// time to wait
		void (*callback_func)(uint32_t value),		// function to call at end (NULL for no call)
		uint32_t value								// value to pass to called function
);

itsdk_timer_return_t stm32l_hwtimer_background_start();
uint64_t stm32l_hwtimer_getDurationUs(itsdk_bool_e stop);


#endif /* STM32L_SDK_TIMER_TIMER_H_ */
