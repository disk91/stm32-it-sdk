/* ==========================================================
 * max17205.h -  Maxim 17205 - Gauge 3 cells
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 24 févr. 2019
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2019 Disk91
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

#ifndef DRIVERS_GAUGE_MAX17205_MAX17205_H_
#define DRIVERS_GAUGE_MAX17205_MAX17205_H_

// ====================================================================
// API
// ====================================================================

typedef enum {
	MAX17205_MODE_DEFAULT = 0,


} drivers_max17205_mode_e;


typedef enum __attribute__ ((__packed__)) {
	MAX17205_TYPE_172X1 = 1,
	MAX17205_TYPE_172X5 = 5
} drivers_max17205_type_e;

typedef struct {
	drivers_max17205_mode_e 		mode;		// Setup mode
	drivers_max17205_type_e			devType;	// 172X1 or 172X5


} drivers_max17205_conf_t;

typedef enum {
	MAX17205_SUCCESS =0,
	MAX17205_NOTFOUND,

	MAX17205_FAILED

} drivers_max17205_ret_e;

drivers_max17205_ret_e drivers_max17205_setup(drivers_max17205_mode_e mode);


// ====================================================================
// Registers
// ====================================================================

#define ITSDK_DRIVERS_MAX17205_ADDRESS_000_0FF	0x36			// Non shifted address for memory 0x000 -> 0x0FF
#define ITSDK_DRIVERS_MAX17205_ADDRESS_100_17F	0x0B			// Non shifted address for memory 0x100	-> 0x17F



#define ITSDK_DRIVERS_MAX17205_REG_DEVNAME_ADR			0x21	// Device type identification
#define ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MAX172X1		0x01	//  17201 / 17211 chip
#define ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MAX172X5		0x05	//  17205 / 17215 chip

#define ITSDK_DRIVERS_MAX17205_REG_NRSENSE				0x1CF	//  setup the Rsense value




#endif /* DRIVERS_GAUGE_MAX17205_MAX17205_H_ */
