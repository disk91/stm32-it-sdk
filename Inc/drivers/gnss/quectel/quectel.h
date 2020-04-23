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


#define DRIVER_GNSS_QUECTEL_MODEL_L86	0				// Support GPS & GLONASS + wakeup with FORCE_ON pin
#define DRIVER_GNSS_QUECTEL_MODEL_L80	1				// Supports GPS, backup mode by cutting VCC

gnss_ret_e quectel_lxx_initLowPower();

typedef struct {
	volatile uint8_t				hasboot:1; 			// flag indicating the device has reboot (and is responding)
			 uint8_t				hasBackupMode:1;	// The ultra low power backup mode is supported
			 uint8_t				isInBackupMode:1;	// The device is currently in backup Mode
			 uint8_t				isInStopMode:1;		// Stop mode request (not existing, ultra cold restart will be performed instead)
			 uint8_t				isRunning:1;		// Position search in progress
			 uint8_t				nmeaProcessed;		// Number of nmea messages processed (%256) to determine error rate
			 uint8_t				nmeaErrors;			// Number of error on serial line since the nmeaProcessed has been 0
	volatile uint8_t				hasAckedSuccess:1;	// previous command acked success
	volatile uint8_t				hasAckedFailed:1;	// previous command acked with a failure
	volatile uint16_t				lastAckedCode;		// Last command code acked

} quectel_status_t;

#define DRIVER_GNSS_QUECTEL_CMD_MAXZ				80

#define DRIVER_GNSS_QUECTEL_CMD_RESTART				0xFFFD
#define DRIVER_GNSS_QUECTEL_CMD_PQTXT				0xFFFE
#define DRIVER_GNSS_QUECTEL_CMD_NOACK				0xFFFF
#define DRIVER_GNSS_QUECTEL_CMD_SET_NMEA_OUTPUT		314
#define DRIVER_GNSS_QUECTEL_CMD_SET_GNSS_SEARCH		353
#define DRIVER_GNSS_QUECTEL_CMD_SET_PERIODIC_MODE	225
#define DRIVER_GNSS_QUECTEL_CMD_STANDBY_MODE		161
#define DRIVER_GNSS_QUECTEL_CMD_HOT_START			101
#define DRIVER_GNSS_QUECTEL_CMD_TEST				000


#endif /* INC_DRIVERS_GNSS_QUECTEL_QUECTEL_H_ */
