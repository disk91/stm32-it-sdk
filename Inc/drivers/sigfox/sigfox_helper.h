/* ==========================================================
 * sigfox_helper.h - Prototypes for Sigfox helper functions
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 9 nov. 2018
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

#ifndef IT_SDK_DRIVERS_SIGFOX_HELPER_H_
#define IT_SDK_DRIVERS_SIGFOX_HELPER_H_

#include <it_sdk/config.h>
#include <stdbool.h>
#include <drivers/s2lp/s2lp.h>

bool sigfox_init(s2lp_config_t * conf);
void sigfox_cifferKey(s2lp_config_t * conf);
void sigfox_unCifferKey(s2lp_config_t * conf);

#endif /* IT_SDK_DRIVERS_SIGFOX_HELPER_H_ */
