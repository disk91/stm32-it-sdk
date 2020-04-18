/* ==========================================================
 * nmea.h - standard NMEA driver (not implementing vendor specific)
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
 * 
 *  Created on: 13 avr. 2020
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2020
 * ==========================================================
 */

#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_WITH_GNSS_DRIVER == __ENABLE

#ifndef INC_IT_SDK_GNSS_NMEA_H_
#define INC_IT_SDK_GNSS_NMEA_H_

typedef enum {
	NMEA_RMC = 0x0001,		// list of message type a device is supporting
	NMEA_GGA = 0x0002,
	NMEA_GSA = 0x0004,
	NMEA_GSV = 0x0008,
	NMEA_GLL = 0x0010,
	NMEA_VTG = 0x0020,
	NMEA_ZDA = 0x0040

} nmea_supported_e;

gnss_ret_e nmea_selectNMEAMessages(gnss_config_t * config, nmea_supported_e supported);
gnss_ret_e nmea_processNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz);
gnss_ret_e nmea_verifyChecksum(uint8_t * line, uint16_t sz);
gnss_ret_e nmea_addChecksum(uint8_t * line, uint16_t sz);

gnss_ret_e nmea_getDecimalField(uint8_t * line, uint16_t * number);
gnss_ret_e nmea_goNextField(uint8_t ** line);
gnss_ret_e nmea_getRationalField(uint8_t * line, uint16_t * number);
gnss_ret_e nmea_getUTCTimeDateField(gnss_date_t *pTime, uint8_t * timePt, uint8_t * datePt);
gnss_ret_e nmea_getLatLngField(uint8_t * l, int32_t * degrees, uint8_t orientation);



#endif /* INC_IT_SDK_GNSS_NMEA_H_ */

#endif // ITSDK_DRIVERS_WITH_GNSS_DRIVER
#endif // ITSDK_WITH_DRIVERS
