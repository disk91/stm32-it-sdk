/* ==========================================================
 * timer.c - Manage hardware timer functions
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
 *
 * ==========================================================
 */
#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
#if ITSDK_WITH_HW_TIMER != __TIMER_NONE
#include <it_sdk/debug.h>
#include <it_sdk/time/timer.h>
#include <stm32l_sdk/timer/timer.h>
#include "tim.h"


void __timer_enable_clk() {
	#if ITSDK_HW_TIMER1_ID == 21
		__HAL_RCC_TIM21_CLK_ENABLE();
	#elif ITSDK_HW_TIMER1_ID == 22
		__HAL_RCC_TIM22_CLK_ENABLE();
	#elif ITSDK_HW_TIMER1_ID == 1
		__HAL_RCC_TIM1_CLK_ENABLE();
	#elif  ITSDK_HW_TIMER1_ID == 2
		__HAL_RCC_TIM2_CLK_ENABLE();
	#elif  ITSDK_HW_TIMER1_ID == 3
		__HAL_RCC_TIM3_CLK_ENABLE();
	#elif  ITSDK_HW_TIMER1_ID == 4
		__HAL_RCC_TIM4_CLK_ENABLE();
	#elif  ITSDK_HW_TIMER1_ID == 6
		__HAL_RCC_TIM6_CLK_ENABLE();
	#elif  ITSDK_HW_TIMER1_ID == 7
		__HAL_RCC_TIM7_CLK_ENABLE();
	#endif
}

void __timer_disable_clk() {
	#if ITSDK_HW_TIMER1_ID == 21
		__HAL_RCC_TIM21_CLK_DISABLE();
	#elif ITSDK_HW_TIMER1_ID == 22
		__HAL_RCC_TIM22_CLK_DISABLE();
	#elif ITSDK_HW_TIMER1_ID == 1
		__HAL_RCC_TIM1_CLK_DISABLE();
	#elif  ITSDK_HW_TIMER1_ID == 2
		__HAL_RCC_TIM2_CLK_DISABLE();
	#elif  ITSDK_HW_TIMER1_ID == 3
		__HAL_RCC_TIM3_CLK_DISABLE();
	#elif  ITSDK_HW_TIMER1_ID == 4
		__HAL_RCC_TIM4_CLK_DISABLE();
	#elif  ITSDK_HW_TIMER1_ID == 6
		__HAL_RCC_TIM6_CLK_DISABLE();
	#elif  ITSDK_HW_TIMER1_ID == 7
		__HAL_RCC_TIM7_CLK_DISABLE();
	#endif
}

/**
 * Run the timer for the given time. Sync mode, return only after timer execution
 * At end the callback_func is called with value as parameter. If NULL the function
 * just return.
 */
itsdk_timer_return_t stm32l_hwtimer_sync_run(
		uint32_t ms,
		void (*callback_func)(uint32_t value),
		uint32_t value
) {
	__timer_enable_clk();

	// loop number
	uint32_t tics  = (ITSDK_HW_TIMER1_FREQ/1000) * ms;
	if ( tics == 0 ) return TIMER_TOO_SHORT;
	uint32_t loops = tics / ITSDK_HW_TIMER1_MAX;
	uint32_t initial = (tics - loops*ITSDK_HW_TIMER1_MAX);

	// first iteration
	ITSDK_HW_TIMER1_HANDLE.Init.Prescaler = 0;
	ITSDK_HW_TIMER1_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
	ITSDK_HW_TIMER1_HANDLE.Init.Period = initial;
	ITSDK_HW_TIMER1_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&ITSDK_HW_TIMER1_HANDLE);
	__HAL_TIM_CLEAR_FLAG(&ITSDK_HW_TIMER1_HANDLE, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE(&ITSDK_HW_TIMER1_HANDLE);
	while(__HAL_TIM_GET_FLAG(&ITSDK_HW_TIMER1_HANDLE,TIM_FLAG_UPDATE) == 0);
	__HAL_TIM_DISABLE(&ITSDK_HW_TIMER1_HANDLE);

	// loop for timer max duration
	ITSDK_HW_TIMER1_HANDLE.Init.Period = ITSDK_HW_TIMER1_MAX-1;
	HAL_TIM_Base_Init(&ITSDK_HW_TIMER1_HANDLE);
	__HAL_TIM_ENABLE(&ITSDK_HW_TIMER1_HANDLE);
	while ( loops > 0) {
		__HAL_TIM_CLEAR_FLAG(&ITSDK_HW_TIMER1_HANDLE, TIM_FLAG_UPDATE);
		while(__HAL_TIM_GET_FLAG(&ITSDK_HW_TIMER1_HANDLE,TIM_FLAG_UPDATE) == 0);
		loops--;
	}
	__HAL_TIM_DISABLE(&ITSDK_HW_TIMER1_HANDLE);

	__timer_disable_clk();
	if ( callback_func != NULL ) {
		callback_func(value);
	}

	return TIMER_INIT_SUCCESS;
}




#endif
#endif
