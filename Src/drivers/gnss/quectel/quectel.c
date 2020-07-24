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
 * Basically the L80/L86 is MediaTek based GPS
 * User Manual V1.2 with extended features - https://www.quectel.com/UploadImage/Downlad/Quectel_GNSS_SDK_Commands_Manual_V1.2.pdf
 * SDK Command V1.4 https://www.quectel.com/UploadImage/Downlad/Quectel_GNSS_SDK_Commands_Manual_V1.4.pdf
 * Proto specif V1.4 https://www.quectel.com/UploadImage/Downlad/Quectel_L86_GNSS_Protocol_Specification_V1.4.pdf
 * Advanced low power mode (not supported) - https://www.quectel.com/UploadImage/Downlad/Quectel_GNSS_Low_Power_Mode_Application_Note_V2.0.pdf
 * FOTA - https://www.quectel.com/UploadImage/Downlad/Quectel_GNSS_FOTA_User_Guide_V1.0.pdf
 * --
 * PMTK documentation ( not sure it fits 100% with the chip - https://www.rhydolabz.com/documents/25/PMTK_A11.pdf
 * NMEA doc - http://www.plaisance-pratique.com/IMG/pdf/NMEA0183-2.pdf
 * --
 * Mediatek 3333 is a ARM7 @ 158MHz with 8MB flash
 * Fix up to 10Hz
 * see - https://labs.mediatek.com/en/chipset/MT3333
 * datasheet - https://s3-ap-southeast-1.amazonaws.com/mediatek-labs-imgs/downloads/d54c0d5ffdffbc55f6c592c6a804d4f2.pdf?response-content-disposition=inline%3B%20filename%3DMT3333_Datasheet.pdf&X-Amz-Content-Sha256=UNSIGNED-PAYLOAD&X-Amz-Security-Token=IQoJb3JpZ2luX2VjENT%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FwEaDmFwLXNvdXRoZWFzdC0xIkgwRgIhAKnPPzqSkQDDPVQTzY6thnSWkCemwXivUkEXSnBTeonoAiEAgJRpLElPW67W1W0OZkPO8pp5KcPNjnco029WerZ2SQcqxwMI7f%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FARABGgw0NDk2OTY5OTIwNjYiDDKVjBhZVgYaTh4I9CqbA7xGd%2B4e5jgqID%2FgMWp2Z5G2Hb9EE3bGy36xZa%2BkRxcsiL7mWpn1bWWxMNvlKFe6VTAYe5Su10beeyDFsM14qzNacSb07GTEFt1sigofFnSUoxTKHg7NFusk6549jcfklCMMg2E6LvMqZXYBrldfaq%2F4Cn%2BuUJDqvD%2FEXkB0cU5sHQfOsnTtJW471vvG8Mc%2Bkl3rBP%2FCXcrb3piWoENqZLZS17uxeC23Inl3Jleiad7COSMX79J%2BTrH20ttwxzHrDNLNGsiJCW5WP%2B44l2JU9SVabA298hfwPEAXCeWUZ0d44QuE5FuHLDYk0QwzKy33wd9YUM53ODh6ARcJ4QAcMkqH5J6BWcoxOBx1IjJimra%2FWMZnePTSx42%2FIrU%2F20%2Bb%2BxpdEgyZKPJ0TJzy49ptjM2ZvcEPzUmHlNzBEGRQ80R%2BIgiv3DuGMq0NqmFdbcNxc5dW003N%2FYv%2FmqE%2Fkufu9zizuIgM33O9GiBJRcu6iGlsEBkT8S5XLD%2BPlv4WYaeDhx%2BRFui7h%2BvBZzZ8fCkjEOJ4xuEVCEDle4EmMzD7%2F%2FD0BTrqAXr4%2BwU8ik2kz2bTW%2FoKrlje%2BlFSdBGhs8kGJ%2FJjqsIbRnEkce%2BuZfpRmiPaMm%2F05H%2FbEU8hLnjn2YJo20ay5SPSzEvHIjyFe3ncuoYGGUCmcj4AI%2F5xzI8FT%2FwHIrysAfFAIc5iFutQ1S3IiipRbKObNTnl9%2FF%2FkdX1ldqoHCij0KehsKBpjMjv4ZOYQ2eZkU%2BJhEzeFXT3bNuStoOsjQyK7s%2Fgv37YLAaKTGf8NPbxtNIrzQ38vw7ZzqbMYK6FA19y0dcsXw1KYsLRlSwS%2FOhNCNfemj0fqWcqBQ6iyAs79eJa9%2Be91eUgSw%3D%3D&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=ASIAWRNAHRNBG5Z7SB7R%2F20200419%2Fap-southeast-1%2Fs3%2Faws4_request&X-Amz-Date=20200419T122122Z&X-Amz-SignedHeaders=host&X-Amz-Expires=600&X-Amz-Signature=dc4a5af290a9a85a491b1dfd1801eb33958cda1e2d5158fcf0baecb187a74f95
 * -----------------------------------------------------------
 * Measured power consumption
 * - standby mode - 1.15mA on Ivcc+Ibackup @ 3.3V
 * - stop mode - 840uA (see hardware design V1.2 - previous version are wrong it is recommanded to shut the power down externally
 * -----------------------------------------------------------
 * L86 version use for the tests
 * $PQVERNO,R,L86NR02A02S,2018/05/15,21:13*6E
 * $PMTK705,AXN_5.1.6_3333_18050800,0002,Quectel-L86,1.0*13
 */

/**
 * Command 607 => EPO_INFO -> rep 707
 * PMTK707,Set,FWN,FTOW,LWN,LTOW,FCWN,FCTOW,LCWN,LCTOW
 * Set: Total number sets of EPO data stored in the GPS chip
 * FWN & FTOW : GPS week number and TOW of the first set of EPO data stored in chip respectively
 * LWN & LTOW : GPS week number and TOW of the last set of EPO data stored in chip respectively
 * FCWN & FCTOW : GPS week number and TOW of the first set of EPO data that are currently used respectively
 * LCWN & LCTOW : GPS week number and TOW of the last set of EPO data that are currently used respectively
 *
 */


/*
 *
 * ** Fridge ...
 * Not clear how to retreive the command mode once the sat search is ongoing....
 * is that a switch to binary mode ... unclear
 * by-the-way, now I know how to chnage the baudrate properly

	itsdk_delayMs(2000);
	static const uint8_t bin_off[] = "\x24\x0E\x00\xFD\x00\x00\x00\x00\x00\x00\xF3\x0D\x0A";

	for ( int db = 0 ; db <= SERIAL_SPEED_115200 ; db++) {
		log_info(">>> %d\r\n",db);
		__gnss_changeBaudRate(db);
		itsdk_delayMs(10);
		serial1_write(bin_off,13);
		itsdk_delayMs(20);
		sprintf(cmd,"$PMTK000*");
		if ( __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_TEST) == GNSS_SUCCESS )
			break;
	}
	serial1_changeBaudRate(SERIAL_SPEED_9600);
 *
 */

#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_GNSS_QUECTEL == __ENABLE
#include <drivers/gnss/quectel/quectel.h>

#include <stdio.h>

#include <it_sdk/wrappers.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/gnss/nmea.h>

// --------------------------------------------------------------------------------
// Internal
static gnss_ret_e __quectelNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz, gnss_nmea_driver_t * driver);
static gnss_ret_e __quectedSendCommand(char * cmd, uint8_t sz, uint16_t icmd);
static gnss_ret_e __quectelWaitForAck(uint16_t commandCode);
static gnss_ret_e __quectelSetRunMode(gnss_run_mode_e mode);
static gnss_ret_e __quectelSwitchToStopWithMemoryRetention();
static gnss_ret_e __quectelSwitchBackfromStopMode();
static gnss_ret_e __quectelSwitchToStandbyWithMemoryRetention();
static gnss_ret_e __quectelSwitchToHotStart();
static gnss_ret_e __quectelSwitchToWarmStart();
static gnss_ret_e __quectelSwitchToColdStart();
static gnss_ret_e __quectelSwitchToFullColdStart();
static gnss_ret_e __quectelStop();
static quectel_status_t __quectel_status;


/**
 * Init the quectel device then setup it stopped to avoid consuming energy
 */
gnss_ret_e quectel_lxx_initLowPower(gnss_config_t * config) {

	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];


	// configure the NEMA MESSAGE OUTPUT
	if ( nmea_selectNMEAMessages(config, (NMEA_RMC|NMEA_GGA|NMEA_GSA|NMEA_GSV|NMEA_GLL|NMEA_VTG|NMEA_ZDA/*|MTK_CHN*/) ) != GNSS_SUCCESS ) {
		#if (ITSDK_LOGGER_MODULE & __LOG_MOD_GNSS) > 0
 		  log_error("QUECTEL_L8X - Unsupported requested GNSS information\r\n");
		#endif
		return GNSS_NOTSUPPORTED;
	}

	__quectel_status.hasboot = 0;
	__quectel_status.isInBackupMode = 0;
	__quectel_status.isRunning = 0;
	__quectel_status.nmeaProcessed = 0;
	__quectel_status.nmeaErrors = 0;

	// Setup the gnss configuration
	config->withNmeaDecodeur = 1;
	config->setRunMode = &__quectelSetRunMode;
	config->driver.nmea.nmeaParser = &__quectelNMEA;
	config->driver.nmea.firstMessage = NMEA_NONE;
	config->driver.nmea.currentMessage = NMEA_NONE;
	config->driver.nmea.triggeringMessage = NMEA_NONE;


	#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L80
	if ( ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN,GPIO_OUTPUT_PP);
		#if ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_POL == __HIGH
			gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN);
		#else
			gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN);
		#endif
		__quectel_status.hasBackupMode = 1;
	} else {
		__quectel_status.hasBackupMode = 0;
	}
	#endif

	// Ensure all the conditions are ok for powering
	#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L86
	if (   ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN != __LP_GPIO_NONE
		|| ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN != __LP_GPIO_NONE
	) {
		if ( ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN != __LP_GPIO_NONE  ) {
			gpio_configure(ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN,GPIO_OUTPUT_PP);
			gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN);
		}
		if ( ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN != __LP_GPIO_NONE ) {
			gpio_configure(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN,GPIO_OUTPUT_PP);
			#if ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_POL == __HIGH
			  gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN);
			#else
			  gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN);
			#endif
		}
		__quectel_status.hasBackupMode = 1;
	} else {
		__quectel_status.hasBackupMode = 0;
	}
	#endif

	// Configure the Reset pin and use it
	if  ( ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN,GPIO_OUTPUT_PP);
		gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
		itsdk_delayMs(30); // 10 ms min according to doc
		gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
	}

	// check Quectel presence and wait for boot
	if( __quectelWaitForAck(DRIVER_GNSS_QUECTEL_CMD_RESTART) != GNSS_SUCCESS ) {
		#if (ITSDK_LOGGER_MODULE & __LOG_MOD_GNSS) > 0
			log_error("QUECTEL_L8X - Failed to detect\r\n");
		#endif
		__quectelStop();
		return GNSS_NOTFOUND;
	}

	#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L80
	itsdk_delayMs(100);	// extra delay needed for L80
	#endif

	// Check with a no action code, failed if failed
	sprintf(cmd,"$PMTK000*");
	if ( __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_TEST) != GNSS_SUCCESS ) {
		__quectelStop();
		return GNSS_FAILED;
	}

	// Get the firmware version (Quectel)
	//sprintf(cmd,"$PQVERNO,R*");
	//__quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_NOACK);

	// Get the firmware version (Mediatek)
	//sprintf(cmd,"$PMTK605*");
	//__quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_NOACK);


	// Setup the desired GNSS constellation
	// This makes the device to restart
    #if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L86
		sprintf(cmd,"$PMTK353,%d,%d,%d,%d,0*",
				((ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE)?1:0),
				((ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE)?1:0),
				((ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE)?1:0),	// Galileo Enable
				0													// Galileo FULL_Enable - This mode is not anymore supported and must not be using
																	//   https://forums.quectel.com/t/l86-what-is-the-difference-between-galileo-enable-and-galileo-full-enable/4231/3
		);
	__quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_SET_GNSS_SEARCH);
	#endif

	// Setup NMEA output
    // @TODO test le dernier qui est GPS channel status msg MCHN -> PMTKCHN
	sprintf(cmd,"$PMTK314,%d,%d,%d,%d,%d,%d,0,0,0,0,0,0,0,0,0,0,0,%d,%d*",
			config->driver.nmea.expectedGLL,
			config->driver.nmea.expectedRMC,
			config->driver.nmea.expectedVTG,
			config->driver.nmea.expectedGGA,
			config->driver.nmea.expectedGSA,
			config->driver.nmea.expectedGSV,
			config->driver.nmea.expectedZDA,
			config->driver.nmea.expectedCHN
	);
	__quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_SET_NMEA_OUTPUT);


	// Disable $GPTXT message (version 1.2)
	sprintf(cmd,"$PQTXT,W,0,1*");
	__quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_PQTXT);

	// Switch to low power
	__quectel_status.isInStopMode = 1;				// force clean COLD start on reset
	__quectelSwitchToStopWithMemoryRetention();
	return GNSS_SUCCESS;
}

/**
 * Switch the device to the given mode
 */
static gnss_ret_e __quectelSetRunMode(gnss_run_mode_e mode) {
	switch (mode) {
		default:
		case GNSS_STOP_MODE:
			__quectel_status.isInStopMode = 1;
			return __quectelSwitchToStopWithMemoryRetention();
		case GNSS_STOP_FORCE:
			// Sometime the stop command is not correctly proceeded when we do not have an external circuit to power it on/off
			// So in a such case, reseting the device will allow to be back is a known mode to stop it properly.
			__quectel_status.isInStopMode = 1;
			gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
			itsdk_delayMs(20);
			gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
			__gnss_connectSerial();
	  		__gnss_initSerial();
	  	    __quectelWaitForAck(DRIVER_GNSS_QUECTEL_CMD_RESTART);
			return __quectelSwitchToStopWithMemoryRetention();
		case GNSS_BACKUP_MODE:
			__quectel_status.isInStopMode = 0;
			return __quectelSwitchToStopWithMemoryRetention();
		case GNSS_SLEEP_MODE:
			__quectel_status.isInStopMode = 0;
			return __quectelSwitchToStandbyWithMemoryRetention();
		case GNSS_RUN_COLD:
			if ( __quectel_status.isInStopMode == 1 ) {
				__quectel_status.isInStopMode = 0;
				return __quectelSwitchToFullColdStart();
			} else {
				return __quectelSwitchToColdStart();
			}
		case GNSS_RUN_WARM:
			if ( __quectel_status.isInStopMode == 1 ) {
				__quectel_status.isInStopMode = 0;
				return __quectelSwitchToFullColdStart();
			} else {
				return __quectelSwitchToWarmStart();
			}
		case GNSS_RUN_HOT:
			if ( __quectel_status.isInStopMode == 1 ) {
				__quectel_status.isInStopMode = 0;
				return __quectelSwitchToFullColdStart();
			} else {
				return __quectelSwitchToHotStart();
			}
	}
}

/**
 * Power off the device if the feature is supported
 */
static gnss_ret_e __quectelStop() {
	if ( ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN != __LP_GPIO_NONE ) {
		// violent stop, no care
		#if ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_POL == __HIGH
		  gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN);
		#else
		  gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN);
		#endif
		#if ITSDK_DRIVERS_GNSS_QUECTEL_L8X_SERIAL_DISC == __ENABLE
		__gnss_disconnectSerial();
		#endif
		__quectel_status.isInBackupMode = 1;	// Not really but needed for restarting
		__quectel_status.isRunning = 0;
		return GNSS_SUCCESS;
	}
	return GNSS_FAILED;
}


/**
 * Switch the GPS in backup mode. in the mode only V_Backup is still activated
 * to preserve the ephemeris. The power consumption is 7uA.
 */
static gnss_ret_e __quectelSwitchToStopWithMemoryRetention() {
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];
	// Switch to Perpetual Backup mode
	// L86 Consumption 840uA - sounds like we do not have a ack on this command every time

	if ( __quectel_status.hasBackupMode == 1 ) {

		// Choice 1 : we have a Hard VCC disconnect pin so we just disconnect it
		if ( ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN != __LP_GPIO_NONE ) {
			#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L80
				// request for perpetual backup mode
				sprintf(cmd,"$PMTK225,4*");
				__quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_NOACK);
				itsdk_delayMs(100);
			#endif

			// violent stop, no care
			#if ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_POL == __HIGH
			  gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN);
			#else
			  gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN);
			#endif
		} else {
			// we have no VCC stop we can try the official method
			#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L86
			  if ( ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN != __LP_GPIO_NONE ) {
				 gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN);
			  }
			#endif
			itsdk_delayMs(100);
			sprintf(cmd,"$PMTK225,4*");
			//DRIVER_GNSS_QUECTEL_CMD_NOACK
			if ( __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_NOACK) != GNSS_SUCCESS ) {
				return GNSS_FAILED;
			}
			__gnss_initSerial();
		}
		__quectel_status.isInBackupMode = 1;
		__quectel_status.isRunning = 0;
		#if ITSDK_DRIVERS_GNSS_QUECTEL_L8X_SERIAL_DISC == __ENABLE
		__gnss_disconnectSerial();
		#endif
		return GNSS_SUCCESS;
	} else {
		// fallback in standby mode
		return __quectelSwitchToStandbyWithMemoryRetention();
	}
}

/**
 * Wake up the device then switch it to stand by otherwise it restart
 */
static gnss_ret_e __quectelSwitchBackfromStopMode() {
	if (__quectel_status.isInBackupMode == 1) {
		// wake up if needed

   	    #if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L86
		 if ( ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN != __LP_GPIO_NONE ) {
		     gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L86_FORCEON_PIN);
		 }
		#endif
		if ( ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN != __LP_GPIO_NONE ) {
		   // Power VCC on from external circuitery
			#if ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_POL == __HIGH
			  gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN);
			#else
			  gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_L8X_POWERON_PIN);
			#endif

		}

  		// A reset should not be needed but it seems it is not working w/o it.
		gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
		itsdk_delayMs(30); // 10 ms min according to doc
		gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);

		__gnss_connectSerial();
  		__gnss_initSerial();
  	    if ( __quectelWaitForAck(DRIVER_GNSS_QUECTEL_CMD_RESTART) == GNSS_TIMEOUT) {
  	    	// We failed to wake up - reset !
  			gpio_reset(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
  			itsdk_delayMs(20); // 10 ms min according to doc
  			gpio_set(ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_BANK,ITSDK_DRIVERS_GNSS_QUECTEL_NRESET_PIN);
  			__gnss_initSerial();
  	  		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_QUECTEL8X_FORCERST,0);
  			// Retrying
  			if ( __quectelWaitForAck(DRIVER_GNSS_QUECTEL_CMD_RESTART) == GNSS_TIMEOUT) {
  	  	  		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_QUECTEL8X_RSTFAILD,0);
  				return GNSS_FAILED;
  			}
  	    }
		#if ITSDK_DRIVERS_GNSS_QUECTEL_MODEL == DRIVER_GNSS_QUECTEL_MODEL_L80
  		  itsdk_delayMs(100);	// extra delay needed for L80
		#endif

  	    __quectel_status.isInBackupMode = 0;
  	    return GNSS_SUCCESS;
	}
	return GNSS_SUCCESS;
}

/**
 * Switch the GPS in standaby mode. in thiw mode the MCU is activated but low power
 * and the sat search is stopped. It is possible to wake up the GPS using the UART
 * power consumption is 1mA
 */
static gnss_ret_e __quectelSwitchToStandbyWithMemoryRetention() {
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];

	if ( __quectelSwitchBackfromStopMode() == GNSS_SUCCESS ) {
		// Switch to Perpetual Backup mode
		// Consumption 1mA
		sprintf(cmd,"$PMTK161,0*");
		if ( __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_STANDBY_MODE) == GNSS_SUCCESS ) {
			__quectel_status.isRunning = 0;
			return GNSS_SUCCESS;
		} else return GNSS_FAILED;
	} else return GNSS_FAILEDRESTARTING;
}


/**
 * Switch the GPS in HOT start -> epehemris should be still valid to get a fix
 */
static gnss_ret_e __quectelSwitchToHotStart() {
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];
	if ( __quectel_status.isRunning == 1 ) return GNSS_ALLREADYRUNNNING;
	if ( __quectelSwitchBackfromStopMode() == GNSS_SUCCESS ) {
		sprintf(cmd,"$PMTK101*");
		if ( __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_NOACK) == GNSS_SUCCESS ) {
			__quectel_status.isRunning = 1;
			return GNSS_SUCCESS;
		} else return GNSS_FAILED;
	} else return GNSS_FAILEDRESTARTING;
}

/**
 * Switch the GPS in WARM start -> Time and position a approximately known but ephemeris need to
 * be obtained to get the position
 */
static gnss_ret_e __quectelSwitchToWarmStart() {
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];
	if ( __quectel_status.isRunning == 1 ) return GNSS_ALLREADYRUNNNING;
	if ( __quectelSwitchBackfromStopMode() == GNSS_SUCCESS ) {
		sprintf(cmd,"$PMTK102*");
		if ( __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_NOACK) == GNSS_SUCCESS ) {
			__quectel_status.isRunning = 1;
			return GNSS_SUCCESS;
		} else return GNSS_FAILED;
	} else return GNSS_FAILEDRESTARTING;
}

/**
 * Switch the GPS in WARM start -> Force to restart not using any of the prior information known
 */
static gnss_ret_e __quectelSwitchToColdStart() {
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];
	if ( __quectel_status.isRunning == 1 ) return GNSS_ALLREADYRUNNNING;
	if ( __quectelSwitchBackfromStopMode() == GNSS_SUCCESS ) {
		sprintf(cmd,"$PMTK103*");
		if ( __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_NOACK) == GNSS_SUCCESS ) {
			__quectel_status.isRunning = 1;
			return GNSS_SUCCESS;
		} else return GNSS_FAILED;
	} else return GNSS_FAILEDRESTARTING;
}

/**
 * Switch the GPS in WARM start -> Force to restart not using any of the prior information known
 * Basically this is a reset of the chip, all me memory and settings are cleared
 */
static gnss_ret_e __quectelSwitchToFullColdStart() {
	char cmd[DRIVER_GNSS_QUECTEL_CMD_MAXZ];
	if ( __quectel_status.isRunning == 1 ) return GNSS_ALLREADYRUNNNING;
	if ( __quectelSwitchBackfromStopMode() == GNSS_SUCCESS ) {
		sprintf(cmd,"$PMTK104*");
		if ( __quectedSendCommand(cmd,DRIVER_GNSS_QUECTEL_CMD_MAXZ,DRIVER_GNSS_QUECTEL_CMD_NOACK) == GNSS_SUCCESS ) {
			__quectel_status.isRunning = 1;
			return GNSS_SUCCESS;
		} else return GNSS_FAILED;
	} else return GNSS_FAILEDRESTARTING;
}

/**
 * Send a command to the device and verify processing
 * Add the checksum at end of the command
 * Some of the command are not acked
 * - PMTK_CMD_HOT_START
 * - PMTK_CMD_WARM_START
 * - PMTK_CMD_COLD_START
 * - PMTK_CMD_FULL_COLD_START
 * - PMTK_SET_NMEA_BAUDRATE
 */
static gnss_ret_e __quectedSendCommand(char * cmd, uint8_t sz, uint16_t icmd) {
	nmea_addChecksum((uint8_t*)cmd, sz);
	__gnss_printf("%s",cmd);
	if( icmd != DRIVER_GNSS_QUECTEL_CMD_NOACK && __quectelWaitForAck(icmd) != GNSS_SUCCESS ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_QUECTEL8X_RSPFAILD,icmd);
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
	uint16_t timeout;
	itsdk_bool_e response = BOOL_FALSE;

	__quectel_status.hasAckedFailed = 0;
	__quectel_status.hasAckedSuccess = 0;
	__quectel_status.lastAckedCode = DRIVER_GNSS_QUECTEL_CMD_NOACK;
	__quectel_status.hasboot = 0;

	// boot messages comes after a first 2s cycle, so timeout is around 2020
	timeout = ( commandCode == DRIVER_GNSS_QUECTEL_CMD_RESTART )?3000:500;
	do {
		switch (commandCode) {
			case DRIVER_GNSS_QUECTEL_CMD_RESTART:
				if ( __quectel_status.hasboot == 1 ) response = BOOL_TRUE;
				break;
			default:
				if ( __quectel_status.lastAckedCode != DRIVER_GNSS_QUECTEL_CMD_NOACK ) {
					if ( __quectel_status.lastAckedCode == commandCode ) {
						if ( __quectel_status.hasAckedFailed == 1 ) {
							ret = GNSS_FAILED;
						}
						response = BOOL_TRUE;
					} else {
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
		return GNSS_TIMEOUT;
	}
	return ret;
}


/**
 * Quectel NMEA driver
 */

static gnss_ret_e __quectelNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz, gnss_nmea_driver_t * driver) {

	#if (ITSDK_LOGGER_MODULE & __LOG_MOD_GNSS) > 0
		log_info(".");
	#endif
	__quectel_status.nmeaProcessed++;
	if ( __quectel_status.nmeaProcessed == 0 ) {
		__quectel_status.nmeaErrors = 0;
	}
	nmea_supported_e previous = driver->currentMessage;
	//log_info("[%X]",previous);
	gnss_ret_e ret = nmea_processNMEA(data, line, sz,driver);

	// To print the received messages for debug
	//char __l[4];
	//strncpy(__l,&line[3],3);
	//__l[3]=0;
	//log_info("[%s]",__l);

	//log_info("[%s]",line);
	switch (ret) {
		case GNSS_PROPRIETARY: {
			ret = GNSS_NOTSUPPORTED;
			// The line has been verified by the NMEA parser, format and checksum is valid
			if ( sz > 10 && line[2] == 'Q' && line[3] == 'T' && line[4] == 'X' && line[5] == 'T' ) {
				// Ack the PQTXT command
				// $PQTXT,W,OK* or $PQTXT,W,ERROR*
				__quectel_status.lastAckedCode = DRIVER_GNSS_QUECTEL_CMD_PQTXT;
				if ( line[9] == 'O' ) {
					__quectel_status.hasAckedSuccess = 1;
					__quectel_status.hasAckedFailed = 0;
				} else {
					__quectel_status.hasAckedSuccess = 0;
					__quectel_status.hasAckedFailed = 1;
				}
				ret = GNSS_SUCCESS;
			} else if( sz > 8 && line[2] == 'M' && line[3] == 'T' && line[4] == 'K' ) {
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
							// $PMTK010 - System message
							// $PMTK010,001*2E - is indicating a startup
							// $PMTK010,002*2D - host aiding EPO - Last message got at boot time
							uint8_t * pt = &line[7];
							uint16_t reason;
							if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
							if ( nmea_getDecimalField(pt,&reason) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
							switch(reason) {
							case 1:
								// Before this stage the GNSS do not accept to receive command.
								// After the next one it seems not either
								__quectel_status.hasboot = 1;
								break;
							case 2:
								// Not clear but when this status is reached the
								// GNSS does not accept to receive any command
								// More over, any command on the serial port kills the GNSS
								__quectel_status.hasboot = 1;
								break;
							default:
								break;
							}
							ret = GNSS_SUCCESS;
						}
					}
				} else if ( line[5] == '7' && line[6] == '0' && line[7] == '7' ) {
					// Firmware version
					// In response to "$PMTK605*
					// $PMTK705,AXN_5.1.6_3333_18050800,0002,Quectel-L86,1.0*13
					//           |                       |     +----------+------> Product model / SDK version
					//           |                       + Buid ID
					//           + Release string AXN => MT 3329-3339-3333 chip - detailed in the string
					//                            Mcore => MT 3318 chip
					ret = GNSS_SUCCESS;
				} else if ( line[5] == 'C' && line[6] == 'H' && line[7] == 'N' ) {
					// PMTKCHN message => satellite status with following format
					// PPNNT
					//   PP - Sat Number
					//   NN - SNR
					//   T - state : 0-idle 1-searching 2-tracking
					// Problem is that this message size is really long with 6chars per satellites
					// so it requires a lot of memory to process or when need to have a char by char processing
					// this could be an improvement for later ...
					// forget it now.
					ret = GNSS_SUCCESS;
				}
			  }
			}
			break;
		case GNSS_SUCCESS:
			// processed by NMEA driver
			// We are searching to identifying the last message of the message flow
			// this message can change over time
			// Only important point First message need to stay the first message and do not disappear

			if ( driver->firstMessage == NMEA_NONE ) {
				// we found the first of the messages
				driver->firstMessage = driver->currentMessage;
				driver->numOfMessages = 0;
			} else if ( driver->firstMessage == driver->currentMessage && driver->triggeringMessage == NMEA_NONE ) {
				// we found the last of the messages
				driver->triggeringMessage = previous;
				driver->numOfMessages = driver->numOfMessCurShot;
			}

			if ( driver->firstMessage != NMEA_NONE  && driver->currentMessage == driver->firstMessage ) {
				// First message detected
				if ( driver->numOfMessages > 0 && driver->numOfMessCurShot < driver->numOfMessages) {
					// but the previous message flow was some missing messages
					driver->triggeringMessage = NMEA_NONE;
					driver->numOfMessages = 0;
				}
				driver->numOfMessCurShot=1;
			} else {
				driver->numOfMessCurShot++;
			}

			if (       driver->triggeringMessage != NMEA_NONE
					&& driver->currentMessage == driver->triggeringMessage
					&& driver->numOfMessages == driver->numOfMessCurShot
			) {
				// end of the NMEA message flow we can place the callback
				// @TODO - if RMC is the last message it can change from GP to GN when the fix has been performed...
				driver->onDataRefreshed();
		    } else if ( (driver->numOfMessages > 0 && driver->numOfMessCurShot >= driver->numOfMessages ) ) {
		    	// The message flow has changed, less or more messages are now sent over the line
				// we need to find the new last message from the list
				driver->triggeringMessage = NMEA_NONE;
				driver->numOfMessages = 0;
		    }


			break;

		default:
			#if (ITSDK_LOGGER_MODULE & __LOG_MOD_GNSS) > 0
			//log_error("## error from nmea decoder %d\r\n",ret);
			//log_error("ON : %s\r\n",line);
			#endif
			// Accepting an error rate of 20%, reporting an error otherwise
			__quectel_status.nmeaErrors++;
			if ( __quectel_status.nmeaProcessed > 20  && ((100*(uint16_t)__quectel_status.nmeaErrors) / __quectel_status.nmeaProcessed) > 20 ) {
				ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_QUECTEL8X_COMERROR,__quectel_status.nmeaErrors);
				// avoid to spam the log with this message too fast
				__quectel_status.nmeaProcessed = 0;
				__quectel_status.nmeaErrors = 0;
				log_error("G#");
			}
			break;
	}
	return ret;
}



#endif // ITSDK_DRIVERS_QUECTEL
#endif // ITSDK_WITH_DRIVERS
