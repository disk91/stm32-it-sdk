/* ==========================================================
 * misc_wrapper.c - 
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 15 sept. 2018
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
 * Wrapper for different usage
 *
 * ==========================================================
 */

#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0x1

#include <it_sdk/wrappers.h>
#include "stm32l0xx_hal.h"

/**
 * Reset the device
 */
void itsdk_reset() {
	NVIC_SystemReset();
}

#endif
