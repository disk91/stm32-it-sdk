/* ==========================================================
 * time.h - 
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
 *
 * ==========================================================
 */
#ifndef IT_SDK_TIME_TIME_H_
#define IT_SDK_TIME_TIME_H_
#include <stdint.h>

void itsdk_time_add_us(uint32_t us);
void itsdk_time_set_ms(uint64_t ms);
uint64_t itsdk_time_get_ms();
void itsdk_time_reset();
void itsdk_time_init();

#endif /* IT_SDK_TIME_TIME_H_ */
