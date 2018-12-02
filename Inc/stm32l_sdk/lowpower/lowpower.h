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

// Public functions
void stm32l_lowPowerSetup();
void stm32l_lowPowerResume();
void stm32l_lowPowerRestoreGpioConfig();

// Private functions
void _stm32l_disableGpios();


#endif /* STM32L_SDK_LOWPOWER_LOWPOWER_H_ */
