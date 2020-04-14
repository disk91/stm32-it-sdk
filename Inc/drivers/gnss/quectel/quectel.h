/* ==========================================================
 * quectel.h - headers for quectel GNSS
 *   tested with L80 and L86
 * ----------------------------------------------------------
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
 * ---------------------------------------------------------
 *  Created on: 13 avr. 2020
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2020
 * ==========================================================
 */
#include <it_sdk/config.h>
#include <it_sdk/gnss/gnss.h>

#ifndef INC_DRIVERS_GNSS_QUECTEL_QUECTEL_H_
#define INC_DRIVERS_GNSS_QUECTEL_QUECTEL_H_


#define DRIVER_GNSS_QUECTEL_MODEL_L86	0
#define DRIVER_GNSS_QUECTEL_MODEL_L80	0

gnss_ret_e quectel_lxx_initLowPower();


#endif /* INC_DRIVERS_GNSS_QUECTEL_QUECTEL_H_ */