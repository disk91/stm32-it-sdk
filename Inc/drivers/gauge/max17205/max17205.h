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
	MAX17205_TYPE_SINGLE_CELL = 1,
	MAX17205_TYPE_MULTI_CELL2 = 4,			// I don't know why ut this is the returned value ...
	MAX17205_TYPE_MULTI_CELL = 5,
} drivers_max17205_type_e;

typedef enum {
	MAX17205_CELL1 = 0,
	MAX17205_CELL2,
	MAX17205_CELL3,
	MAX17205_CELLX,
	MAX17205_VBAT,
} drivers_max17205_cell_select_e;

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
drivers_max17205_ret_e drivers_max17205_getTemperature(int32_t * mTemp);
drivers_max17205_ret_e drivers_max17205_getVoltage(drivers_max17205_cell_select_e cell, uint16_t * mVolt);

// ====================================================================
// Registers
// ====================================================================

#define ITSDK_DRIVERS_MAX17205_ADDRESS_000_0FF	0x36			// Non shifted address for memory 0x000 -> 0x0FF
#define ITSDK_DRIVERS_MAX17205_ADDRESS_100_1FF	0x0B			// Non shifted address for memory 0x100	-> 0x17F



#define ITSDK_DRIVERS_MAX17205_REG_DEVNAME_ADR			0x21	// Device type identification
#define ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MAX172X1		0x01	//  17201 / 17211 chip (single Cell)
#define ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MAX172X5		0x05	//  17205 / 17215 chip (multi Cell)

#define ITSDK_DRIVERS_MAX17205_REG_TEMP_ADR				0x08	// Temperature

#define ITSDK_DRIVERS_MAX17205_REG_CELL1_VOLT_ADR		0xD8	// CELL1 Voltage
#define ITSDK_DRIVERS_MAX17205_REG_CELL2_VOLT_ADR		0xD7	// CELL2 Voltage
#define ITSDK_DRIVERS_MAX17205_REG_CELL3_VOLT_ADR		0xD7	// CELL3 Voltage
#define ITSDK_DRIVERS_MAX17205_REG_CELLX_VOLT_ADR		0xD9	// CELLX Voltage
#define ITSDK_DRIVERS_MAX17205_REG_VBAT_VOLT_ADR		0XDA	// VBAT Voltage

#define ITSDK_DRIVERS_MAX17205_REG_COMMAND_ADR			0x60	// Command register
#define ITSDK_DRIVERS_MAX17205_REG_CONFIG2_ADR			0xBB	// Config 2 register


#define ITSDK_DRIVERS_MAX17205_REG_NRSENSE				0x1CF	//  setup the Rsense value




#endif /* DRIVERS_GAUGE_MAX17205_MAX17205_H_ */
