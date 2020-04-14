/* ==========================================================
 * quectel.c - headers for quectel GNSS
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
#if ITSDK_DRIVERS_GNSS_QUECTEL == __ENABLE
#include <drivers/gnss/quectel/quectel.h>

#include <it_sdk/wrappers.h>
#include <it_sdk/gnss/nmea.h>

// --------------------------------------------------------------------------------
// Internal
static gnss_ret_e __quectelNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz);

/**
 * Init the quectel device then setup it stopped to avoid consuming energy
 */
gnss_ret_e quectel_lxx_initLowPower(gnss_config_t * config) {

	// Configure the Reset pin and use it
	if  ( ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN,GPIO_OUTPUT_PP);
		gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
		itsdk_delayMs(10);
		gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
	}

	// Setup the gnss configuration
	config->withNmeaDecodeur = 1;
	config->driver.nmea.nmeaParser = &__quectelNMEA;


	return GNSS_SUCCESS;
}


/**
 * Quectel NMEA driver
 */
static gnss_ret_e __quectelNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz) {


	gnss_ret_e ret = nmea_processNMEA(data, line, sz);
	switch (ret) {
		case GNSS_PROPRIETARY: {
			// The line has been verified by the NMEA parser, format and checksum is valid
			if( sz > 8 && line[1] == 'M' && line[2] == 'T' && line[3] == 'K' ) {
				if ( line[4] == '0' ) {
					if ( line[5] == '1' ) {
						if ( line[6] == '1' ) {
							// PMTK011 - System message - identification
							// $PMTK011,MTKGPS*08
						}
						if ( line[6] == '0' ) {
							// PMTK010 - System message
							// $PMTK010,001*2E - is indicating a startup
							// $PMTK010,002*2D - host aiding EPO
						}
					}
				}


			}



			}
			break;
		default:
			break;
	}
	return ret;
}



#endif // ITSDK_DRIVERS_QUECTEL
#endif // ITSDK_WITH_DRIVERS
