/* ==========================================================
 * max44009.h - Maximum Light sensor driver
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 19 févr. 2019
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

#ifndef DRIVERS_LIGHT_MAX44009_MAX44009_H_
#define DRIVERS_LIGHT_MAX44009_MAX44009_H_

// ====================================================================
// API
// ====================================================================

typedef enum {
	MAX44009_MODE_ONREQUEST = 0,				// Basic mode :  measure periodicly, read on request
												//               no interrupt


} drivers_max44009_mode_e;


typedef struct {
	drivers_max44009_mode_e 		mode;		// Setup mode
} drivers_max44009_conf_t;

typedef enum {
	MAX44009_SUCCESS =0,
	MAX44009_NOTFOUND,

	MAX44009_FAILED

} drivers_max44009_ret_e;


drivers_max44009_ret_e drivers_max44009_setup(drivers_max44009_mode_e mode);
drivers_max44009_ret_e drivers_max44009_getSensors(
		uint32_t  * mlux					// Lux value
);

// ====================================================================
// REGISTERS
// ====================================================================
#define DRIVER_MAX44009_DEVICE_ADR 0x4A			// I2C address not shifted.

#define DRIVER_MAX44009_REG_INT_STATUS_ADR		0x00		// Interrupt status
#define DRIVER_MAX44009_REG_INT_ENABLE_ADR		0x01		// Interrupt enable
#define DRIVER_MAX44009_REG_CONFIG_ADR			0x02
#define DRIVER_MAX44009_REG_LUX_HIGH_ADR		0x03		// Lux response High bits
#define DRIVER_MAX44009_REG_LUX_LOW_ADR			0x04		// Lux response Low bits
#define DRIVER_MAX44009_REG_UP_THRESHOLD_ADR	0x05		// Upper threshold high bytes
#define DRIVER_MAX44009_REG_LOW_THRESHOLD_ADR	0x06		// Lower threshold hight bytes
#define DRIVER_MAX44009_REG_THRESHOLD_TIMER_ADR 0x07		// Threshold timer parameter

#define DRIVER_MAX44009_REG_INTSTATUS_INTS_MSK	0x01
#define DRIVER_MAX44009_REG_INTENABLE_INTS_MSK	0x01

#define DRIVER_MAX44009_REG_CONFIG_CONT_MSK		0x80		// Continuous mode
#define DRIVER_MAX44009_REG_CONFIG_CONT_DEFAULT 0x00		//  low power - run every 800ms w/o considering integration time
#define DRIVER_MAX44009_REG_CONFIG_CONT_CONT	0x80		//  run continuously at integration time from 6,25ms to 800ms
#define DRIVER_MAX44009_REG_CONFIG_MANUAL_MSK	0x40		// Manual mode
#define DRIVER_MAX44009_REG_CONFIG_MANUAL_AUTO  0x00		//  automatic mode : the sensor auto adapt timing to light
#define DRIVER_MAX44009_REG_CONFIG_MANUAL_ON	0x40		//  manual mode : you have to set the precision of CDR and TIM according to light

#define DRIVER_MAX44009_REG_LUXHIGH_EXP_MSK		0xF0		// Exponent
#define DRIVER_MAX44009_REG_LUXHIGH_EXP_SHIFT	4
#define DRIVER_MAX44009_REG_LUXHIGH_MAN_MSK		0x0F		// Mantissa
#define DRIVER_MAX44009_REG_LUXHIGH_MAN_SHIFT	0
#define DRIVER_MAX44009_REG_LUXLOW_MAN_MSK		0x0F		// Mantissa
#define DRIVER_MAX44009_REG_LUXLOW_MAN_SHIFT	0

#endif /* DRIVERS_LIGHT_MAX44009_MAX44009_H_ */
