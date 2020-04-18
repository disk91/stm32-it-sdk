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

#include <stdio.h>

#include <it_sdk/wrappers.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/gnss/nmea.h>

// --------------------------------------------------------------------------------
// Internal
static gnss_ret_e __quectelNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz);
static gnss_ret_e __quectelWaitForAck(uint16_t commandCode);
static gnss_ret_e __quectelSwitchToStopWithMemoryRetention();
static quectel_status_t __quectel_status;


/**
 * Init the quectel device then setup it stopped to avoid consuming energy
 */
gnss_ret_e quectel_lxx_initLowPower(gnss_config_t * config) {

	// configure the NEMA MESSAGE OUTPUT
	if ( nmea_selectNMEAMessages(config, (NMEA_RMC|NMEA_GGA|NMEA_GSA|NMEA_GSV|NMEA_GLL|NMEA_VTG|NMEA_ZDA) ) != GNSS_SUCCESS ) {
		log_error("Quectel driver is not supporting the expected GNSS informations\r\n");
		return GNSS_NOTSUPPORTED;
	}


	// Configure the Reset pin and use it
	__quectel_status.hasboot = 0;
	if  ( ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN,GPIO_OUTPUT_PP);
		gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
		itsdk_delayMs(10);
		gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
	}

	// Setup the gnss configuration
	config->withNmeaDecodeur = 1;
	config->driver.nmea.nmeaParser = &__quectelNMEA;

	// check Quectel presence
	if( __quectelWaitForAck(DRIVER_GNSS_QUECTEL_CMD_RESTART) != GNSS_SUCCESS ) {
		log_error("restart failed\r\n");
		return GNSS_NOTFOUND;
	}

	// Setup NMEA output
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];
	sprintf(cmd,"$PMTK314,%d,%d,%d,%d,%d,%d,0,0,0,0,0,0,0,0,0,0,0,%d,0*",
			config->driver.nmea.expectedGLL,
			config->driver.nmea.expectedRMC,
			config->driver.nmea.expectedVTG,
			config->driver.nmea.expectedGGA,
			config->driver.nmea.expectedGSA,
			2*config->driver.nmea.expectedGSV,		// sat status update rate lower
			config->driver.nmea.expectedZDA
	);
	nmea_addChecksum((uint8_t*)cmd, DRIVER_GNSS_QUECTEL_CMD_MAXZ);
	__gnss_printf("%s",cmd);
	if( __quectelWaitForAck(DRIVER_GNSS_QUECTEL_CMD_SET_NMEA_OUTPUT) != GNSS_SUCCESS ) {
		log_error("Failed to setup the NMEA output\r\n");
		// as all is better than cycle reboot... continue
	}

	// Setup the desired GNSS constellation
	sprintf(cmd,"$PMTK353,%d,%d,%d*",
			((ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE)?1:0),
			((ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE)?1:0),
			((0 && ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE)?1:0)
	);
	nmea_addChecksum((uint8_t*)cmd, DRIVER_GNSS_QUECTEL_CMD_MAXZ);
	__gnss_printf("%s",cmd);
	if( __quectelWaitForAck(DRIVER_GNSS_QUECTEL_CMD_SET_GNSS_SEARCH) != GNSS_SUCCESS ) {
		log_error("Failed to setup the GNSS constelationt\r\n");
		// as all is better than cycle reboot... continue
	}

	// Switch to low power
	__quectelSwitchToStopWithMemoryRetention();


	return GNSS_SUCCESS;
}


static gnss_ret_e __quectelSwitchToStopWithMemoryRetention() {
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];

	// Switch to Perpetual Backup mode
	// Consumption 7uA
	sprintf(cmd,"$PMTK225,4*");
	nmea_addChecksum((uint8_t*)cmd, DRIVER_GNSS_QUECTEL_CMD_MAXZ);
	__gnss_printf("%s",cmd);
	if( __quectelWaitForAck(DRIVER_GNSS_QUECTEL_CMD_SET_PERIODIC_MODE) != GNSS_SUCCESS ) {
		log_error("Failed to get backup confirmation\r\n");
		return GNSS_FAILED;
	}
	return GNSS_SUCCESS;
}



/**
 * Wait for a command ack
 */

static gnss_ret_e __quectelWaitForAck(uint16_t commandCode) {

	gnss_ret_e ret = GNSS_SUCCESS;
	uint16_t time = 0;
	uint16_t timeout = 1000;
	itsdk_bool_e response = BOOL_FALSE;

	__quectel_status.hasAckedFailed = 0;
	__quectel_status.hasAckedSuccess = 0;
	__quectel_status.lastAckedCode = 0;
	__quectel_status.hasboot = 0;
	do {
		switch (commandCode) {
			case DRIVER_GNSS_QUECTEL_CMD_RESTART:
				if ( __quectel_status.hasboot == 1 ) response = BOOL_TRUE;
				break;
			default:
				if ( __quectel_status.lastAckedCode != 0 ) {
					if ( __quectel_status.lastAckedCode == commandCode ) {
						if ( __quectel_status.hasAckedFailed == 1 ) {
							ret = GNSS_FAILED;
						}
						response = BOOL_TRUE;
					} else {
						log_debug("Received another command ack : %d\r\n",__quectel_status.lastAckedCode);
						__quectel_status.lastAckedCode = 0;
					}
				}
				break;
		}
		gnss_process_loop(BOOL_TRUE);
		#if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
		   wdg_refresh();
		#endif
		itsdk_delayMs(5);
		time += 5;
	} while (response == BOOL_FALSE && time < timeout);
	if ( response == BOOL_FALSE ) {
		log_error("Failing getting a reponsonse from command %d\r\n",commandCode);
		return GNSS_TIMEOUT;
	}
	return ret;
}


/**
 * Quectel NMEA driver
 */
static gnss_ret_e __quectelNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz) {


	gnss_ret_e ret = nmea_processNMEA(data, line, sz);
	switch (ret) {
		case GNSS_PROPRIETARY: {
			ret = GNSS_NOTSUPPORTED;
			// The line has been verified by the NMEA parser, format and checksum is valid
			if( sz > 8 && line[2] == 'M' && line[3] == 'T' && line[4] == 'K' ) {
				if ( line[5] == '0' ) {
					if ( line[6] == '0' ) {
						if ( line[7] == '1' ) {
							// ACK message - confirm reception of the last command
							// $PMTK001,CommandCode,Status (0,1,2 failed, 3 success)
							uint8_t * pt = &line[7];
							if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
							if ( nmea_getDecimalField(pt,(uint16_t *)&__quectel_status.lastAckedCode) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
							if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
							uint16_t reason;
							if ( nmea_getDecimalField(pt,&reason) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
							if (reason == 3) {
								__quectel_status.hasAckedSuccess = 1;
								__quectel_status.hasAckedFailed = 0;
							} else {
								__quectel_status.hasAckedSuccess = 0;
								__quectel_status.hasAckedFailed = 1;
							}
							ret = GNSS_SUCCESS;
						}
					} else if ( line[6] == '1' ) {
						if ( line[7] == '1' ) {
							// PMTK011 - System message - identification
							// $PMTK011,MTKGPS*08
							ret = GNSS_SUCCESS;
						}
						if ( line[7] == '0' ) {
							// PMTK010 - System message
							// $PMTK010,001*2E - is indicating a startup
							// $PMTK010,002*2D - host aiding EPO
							uint8_t * pt = &line[7];
							uint16_t reason;
							if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
							if ( nmea_getDecimalField(pt,&reason) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
							switch(reason) {
							case 1:
								__quectel_status.hasboot = 1;
								break;
							default:
								break;
							}
							ret = GNSS_SUCCESS;
						}
					}
				}

			}
			log_info("[PROP] %s \r\n",line);

			}
			break;
		case GNSS_SUCCESS:
			break;

		default:
			log_error("## error from nmea decoder %d\r\n",ret);
			log_error("ON : %s\r\n",line);
			break;
	}
	return ret;
}



#endif // ITSDK_DRIVERS_QUECTEL
#endif // ITSDK_WITH_DRIVERS
