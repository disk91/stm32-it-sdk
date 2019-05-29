/* ==========================================================
 * sl353.h -  Honeywell SL353 - Hall Effect magnetic sensor
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 28 may. 2019
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

#ifndef DRIVERS_HALL_SL353_SL353_H_
#define DRIVERS_HALL_SL353_SL353_H_

// ====================================================================
// API
// ====================================================================

typedef enum {
	SL353_SUCCESS=0,
	SL353_FAILED

} drivers_sl353_ret_e;

typedef enum {
	SL353_FIELD_NOT_DETECTED=0,
	SL353_FIELD_DETECTED
} drivers_sl353_state_e;

typedef struct {
	drivers_sl353_state_e 			status;			// magnetic field status
	uint8_t							hasChanged:1;	// status has changed since last read
	void (*onFieldSet)(void);						// when not NULL call when magnetic field is appearing
	void (*onFieldReset)(void);						// when not NULL call when magnetic field is disappearing
	void (*onFieldChange)(drivers_sl353_state_e state);	// when not NULL call on any field change
} drivers_sl353_state_t;

drivers_sl353_ret_e drivers_sl353_setup(
		void (*onFieldSet)(void),
		void (*onFieldReset)(void),
		void (*onFieldChange)(drivers_sl353_state_e state)
);
drivers_sl353_ret_e drivers_sl353_getImmediateState( drivers_sl353_state_e * state );



#endif /* DRIVERS_HALL_SL353_SL353_H_ */
