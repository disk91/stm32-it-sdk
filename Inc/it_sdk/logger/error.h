/* ==========================================================
 * error.c - Store error code in a persistant storage for later
 *           analysis
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 17 feb. 2019
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 Disk91
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
 * ==========================================================
 */
#ifndef IT_SDK_LOGGER_ERROR_H_
#define IT_SDK_LOGGER_ERROR_H_

#include <stdbool.h>
#include <stdint.h>
#include <it_sdk/config.h>



#define ITSDK_ERROR_LASTBLOCK		0xFFFF
#define ITSDK_ERROR_FIRSTBLOCK		0xFFFE
#define ITSDK_ERROR_STRUCT_MAGIC	0xAE73		// random value for magic

typedef enum {
	ITSDK_ERROR_SUCCESS	= 0,
	ITSDK_ERROR_FAILED
} itsdk_error_ret_e;

// Structure of the error block in memory
typedef struct {
	uint16_t	magic;		// Magic to ensure we are at the right memory place
	uint16_t	readPt;		// Index in buffer for reading
	uint16_t	writePt;	// Index in buffer for writing
	uint16_t	later;		// Reserved for later use
} itsdk_error_head_t;

typedef struct {
	uint32_t	timeS;	// time of error in S
	uint32_t	error;	// error code
}itsdk_error_entry_t;

typedef struct {
	itsdk_error_head_t header;
	itsdk_error_entry_t errors[ITSDK_ERROR_BLOCKS];
} itsdk_error_t;


// Standard Error code
// Format for any error code:
// +----+---+---+--------------------+----------------+
// | XX | Y | Z | AAAA AAAA AAAA AAAA| KKKK KKKK KKKK |
// +----+---+---+--------------------+----------------+
//
// XX -> Level : 00 - Info / 01 - WARN / 10 - ERROR / 11 - FATAL
// Y  -> Category : 0 - SDK / 1 - Application
// Z  -> Value : 0 - No associated value / 1 - Associated value
// AAAA .... AAAA -> 16b associated value
// KKKK .... KKKK -> 12b error code
#define ITSDK_ERROR_LEVEL_INFO	0x00000000
#define ITSDK_ERROR_LEVEL_WARN	0x40000000
#define ITSDK_ERROR_LEVEL_ERROR	0x80000000
#define ITSDK_ERROR_LEVEL_FATAL	0xC0000000
#define ITSDK_ERROR_LEVEL_MASK	0xC0000000

#define ITSDK_ERROR_TYPE_SDK	0x00000000
#define ITSDK_ERROR_TYPE_APP	0x20000000
#define ITSDK_ERROR_TYPE_MASK	0x20000000

#define ITSDK_ERROR_WOUT_VALUE	0x00000000
#define ITSDK_ERROR_WITH_VALUE	0x10000000
#define ITSDK_ERROR_VALUE_SHIFT	12
#define ITSDK_ERROR_VALUE_MASK	0x0FFFF000

#define ITSDK_ERROR_ERROR_MASK  0x00000FFF
#define ITSDK_ERROR_ERROR_SHIFT 0


										// Error code under WWW symbol
										//  _______WWW
#define	ITSDK_ERROR_RESET					0x00000001 | (ITSDK_ERROR_LEVEL_INFO  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)
#define	ITSDK_ERROR_I2CFAIL					0x00000002 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)
#define ITSDK_ERROR_CONSOLE_NOTSETUP		0x00000003 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// The console is used before being setup
#define	ITSDK_ERROR_GPIO_UNSUPPORTED_BANK	0x00000004 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)
#define	ITSDK_ERROR_ADC_INIT_FAILED			0x00000005 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)
#define	ITSDK_ERROR_ADC_CALIBRATION_FAILED	0x00000006 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)
#define	ITSDK_ERROR_ADC_CONFCHANNEL_FAILED	0x00000007 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)
#define	ITSDK_ERROR_ADC_INVALID_PIN			0x00000008 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)
#define ITSDK_ERROR_STIMER_INIT_FLD			0x00000010 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)
#define ITSDK_ERROR_STIMER_ALREADY_SET		0x00000011 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)
#define ITSDK_ERROR_STIMER_LIST_FULL		0x00000012 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)  // The timer list is full, timer creation rejected
#define ITSDK_ERROR_ENCRYP_INVALID_DATALEN	0x00000020 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// The datalen is invalid should be 32B blocs
#define ITSDK_ERROR_ENCRYP_DATA_TOOLARGE    0x00000021 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// The datalen is too large
#define	ITSDK_ERROR_EEPROM_OUTOFBOUNDS		0x00000030 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Try to Read/Write out of eprom area
#define	ITSDK_ERROR_EEPROM_NOTALIGNED		0x00000031 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Try to Read/Write out of eprom area
#define	ITSDK_ERROR_WDG_OUTOFBOUNDS			0x00000040 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Watchdog duration setting os outof bounds
#define	ITSDK_ERROR_WDG_INIT_FAILED			0x00000041 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Watchdog init failed
#define ITSDK_ERROR_SCHED_DURATION_OVERFLOW 0x00000050 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Scheduler task period exceed the maximal duration
#define	ITSDK_ERROR_RTC_INVALID_CLKRATIO	0x00000060 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// RTC - Clk ratio for adjustment is invalid (too large)
#define ITSDK_ERROR_TICKS_INVALID_CLKRATIO  0x00000061 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// SYSTICKS - Clk ratio for adjustment is invalid (too large)
#define ITSDK_ERROR_CONFIG_OVERRIDE_MISS    0x00000070 | (ITSDK_ERROR_LEVEL_WARN  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// The config function has not been overrided - no factory default for the app part
#define ITSDK_ERROR_CONFIG_FACTORY_DEFAULT  0x00000071 | (ITSDK_ERROR_LEVEL_WARN  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// The config has been restored to factory default (value bit 0 => SDK / bit 1 => APP)
#define ITSDK_ERROR_CONFIG_COMMIT_NEW_CONF  0x00000072 | (ITSDK_ERROR_LEVEL_INFO  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// A New configuration has been commited
#define ITSDK_ERROR_CONFIG_SDKFACT_DEFAULT  0x00000073 | (ITSDK_ERROR_LEVEL_INFO  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// The config has been restored to factory default (value bit 0 => SDK / bit 1 => APP)
#define ITSDK_ERROR_CONFIG_SDKCNF_UPGRADED  0x00000074 | (ITSDK_ERROR_LEVEL_INFO  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// The SDK config has been Upgraded with version given as value.
#define ITSDK_ERROR_CONFIG_APPCNF_UPGRADED  0x00000075 | (ITSDK_ERROR_LEVEL_INFO  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// The APP config has been Upgraded with version given as value.
#define ITSDK_ERROR_CONFIG_CONFIG_BADMAGIC  0x00000076 | (ITSDK_ERROR_LEVEL_WARN  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// The config eeprom area do not start with magic, it has not yet been initialized or it has been scratched
#define ITSDK_ERROR_CONFIG_CONFIG_BADMNGV   0x00000077 | (ITSDK_ERROR_LEVEL_WARN  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// The config eeprom area management version has changed. Impossible to upgrade
#define ITSDK_ERROR_CONFIG_MIGRATE_FAILED   0x00000078 | (ITSDK_ERROR_LEVEL_WARN  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Config migration failed (size changed or unsupported migration path)(value bit 0 => SDK / bit 1 => APP)

#define ITSDK_ERROR_LORAWAN_INVALID_DR		0x00000100 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)
#define ITSDK_ERROR_LORAWAN_INVALID_TXPWR   0x00000101 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)
#define ITSDK_ERROR_LORAWAN_INVALID_REGION  0x00000102 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Selected region is not valid
#define ITSDK_ERROR_LORAWAN_INVALID_JOIN	0x00000103 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Join method is invalid
#define ITSDK_ERROR_LORAWAN_TIME_NOCALLBACK 0x00000104 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// TimerServer Callback function is null
#define ITSDK_ERROR_LORAWAN_TIME_INITFLD    0x00000105 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// TimerServer Timer init failed
#define ITSDK_ERROR_LORAWAN_SS_INVALID      0x00000106 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Impossible to access SecureStore Data

#define ITSDK_ERROR_SIGFOX_SS_INVALID       0x00000120 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Impossible to access SecureStore Data


#define ITSDK_ERROR_SIGFOX_RCZ_NOTSUPPORTED 0x00000200 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// This RCZ is not supported
#define ITSDK_ERROR_SIGFOX_OOB_NOTSUPPORTED 0x00000201 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// This OOB is not supported
#define ITSDK_ERROR_SIGFOX_TIMER_STARTERROR 0x00000202 | (ITSDK_ERROR_LEVEL_FATAL | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// A timer has not been able to start



#define ITSDK_ERROR_DRV_BME280_NOTFOUND		0x00000300 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Device not found on I2C
#define ITSDK_ERROR_DRV_BME280_I2CERROR		0x00000301 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// I2C Read failed
#define ITSDK_ERROR_DRV_O2SMPB_NOTFOUND		0x00000304 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Device not found on I2C
#define ITSDK_ERROR_DRV_O2SMPB_I2CERROR		0x00000305 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// I2C Read failed
#define ITSDK_ERROR_DRV_MAX44009_NOTFOUND	0x00000308 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Device not found on I2C
#define ITSDK_ERROR_DRV_MAX44009_I2CERROR	0x00000309 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// I2C Read failed
#define ITSDK_ERROR_DRV_ST25DV_NOTFOUND		0x00000310 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Device not found on I2C
#define ITSDK_ERROR_DRV_ST25DV_I2CERROR		0x00000311 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// I2C Read failed
#define ITSDK_ERROR_DRV_ST25DV_PASSCHGOK	0x00000312 | (ITSDK_ERROR_LEVEL_INFO  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// I2C Password change success
#define ITSDK_ERROR_DRV_ST25DV_PASSCHGKO	0x00000313 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// I2C Password change failed
#define ITSDK_ERROR_DRV_ST25DV_SERIALUZRD	0x00000314 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Serial User failed to Rd NFC
#define ITSDK_ERROR_DRV_ST25DV_SERIALUZWR	0x00000315 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Serial User failed to Wr NFC
#define ITSDK_ERROR_DRV_MAX17205_NOTFOUND	0x00000320 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Device not found on I2C
#define ITSDK_ERROR_DRV_MAX17205_I2CERROR	0x00000321 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// I2C Read failed
#define ITSDK_ERROR_DRV_MAX17205_UNDERVOLT	0x00000322 | (ITSDK_ERROR_LEVEL_WARN  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// The voltage is under the working limit. As a value the Voltage in dV
#define ITSDK_ERROR_DRV_MAX17205_NVUPDREM	0x00000323 | (ITSDK_ERROR_LEVEL_INFO  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Number of remaining NV Memory update is short
#define ITSDK_ERROR_DRV_MAX17205_NVFAILED	0x00000324 | (ITSDK_ERROR_LEVEL_INFO  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Failed to save the NV Memory
#define ITSDK_ERROR_DRV_LIS2DH_NOTFOUND		0x00000330 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Device not found on I2C
#define ITSDK_ERROR_DRV_LIS2DH_CAPCONFFAIL	0x00000331 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Configuration in capture mode failed
#define ITSDK_ERROR_DRV_LIS2DH_TRICONFFAIL	0x00000332 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Configuration in triggering mode failed
#define ITSDK_ERROR_DRV_QUECTEL8X_FORCERST	0x00000340 | (ITSDK_ERROR_LEVEL_INFO  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Device did not properly restart second reset needed
#define ITSDK_ERROR_DRV_QUECTEL8X_RSTFAILD	0x00000341 | (ITSDK_ERROR_LEVEL_WARN  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// Device did not properly apply second reset
#define ITSDK_ERROR_DRV_QUECTEL8X_RSPFAILD	0x00000342 | (ITSDK_ERROR_LEVEL_WARN  | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Failed to get a response from a serial command given as a parameter
#define ITSDK_ERROR_DRV_QUECTEL8X_COMERROR	0x00000343 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WITH_VALUE)	// Many errors during the serial transmission error rate reached > 20%
#define ITSDK_ERROR_DRV_GNSS_FAILSTOP		0x00000350 | (ITSDK_ERROR_LEVEL_ERROR | ITSDK_ERROR_TYPE_SDK | ITSDK_ERROR_WOUT_VALUE)	// GPS did not automatically stopped at end of the duration, stopped has been forced by garbage collection



#if ITSDK_WITH_ERROR_RPT == __ENABLE

#if ITSDK_WITH_ERROR_EXTENTION == __ENABLE
#include <it_sdk/configError.h>
#endif


// ==================================================================
// NVM Storage API (Weak functions)
// ==================================================================
itsdk_error_ret_e _itsdk_error_readHeader(itsdk_error_head_t * header);
itsdk_error_ret_e _itsdk_error_writeHeader(itsdk_error_head_t * header);
itsdk_error_ret_e _itsdk_error_write(uint16_t blockId, itsdk_error_entry_t * entry);
itsdk_error_ret_e _itsdk_error_read(uint16_t blockId,itsdk_error_entry_t * e);

// ==================================================================
// PUBLIC API
// ==================================================================
#define ITSDK_ERROR_REPORT(error,value)	itsdk_error_report(error,value)

itsdk_error_ret_e itsdk_error_setup();
itsdk_error_ret_e itsdk_error_report(uint32_t error,uint16_t value);
itsdk_error_ret_e itsdk_error_get(uint16_t * blockId,itsdk_error_entry_t * e);
itsdk_error_ret_e itsdk_error_clear();
itsdk_error_ret_e itsdk_error_getSize(uint32_t * size);


#else /* ITSDK_WITH_ERROR_RPT */
#define ITSDK_ERROR_LEVEL_FATAL	0xC0000000
#define ITSDK_ERROR_REPORT(error,value) itsdk_error_noreport(error)
itsdk_error_ret_e itsdk_error_noreport(uint32_t error);

#endif /* ITSDK_WITH_ERROR_RPT */


#endif /* IT_SDK_LOGGER_ERROR_H_ */
