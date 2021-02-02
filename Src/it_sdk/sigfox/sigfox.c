/* ==========================================================
 * sigfox.c - Abstraction layer for sigfox libraries
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 04 nov. 2018
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
 *
 * ==========================================================
 */
#include <stdbool.h>
#include <string.h>

#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/sigfox/sigfox.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/encrypt/encrypt.h>

#if ITSDK_WITH_SIGFOX_LIB > 0

#include <drivers/sigfox/sigfox_api.h>
#include <drivers/sigfox/se_nvm.h>

#if ITSDK_WITH_SECURESTORE == __ENABLE
  #include <it_sdk/eeprom/securestore.h>
#endif

#include <it_sdk/eeprom/sdk_state.h>
#include <it_sdk/eeprom/sdk_config.h>

#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
	#include <drivers/s2lp/s2lp.h>
	#include <drivers/s2lp/sigfox_helper.h>
	#include <drivers/s2lp/st_rf_api.h>
	#include <drivers/eeprom/m95640/m95640.h>
	#include <drivers/sigfox/sigfox_api.h>
#elif ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
	#include <drivers/sx1276/sigfox_sx1276.h>
#endif
#include <drivers/sigfox/mcu_api.h>

#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
	#include <it_sdk/eeprom/eeprom.h>
#endif


#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
s2lp_config_t __s2lpConf;
#endif

/**
 * Static definitions
 */
int8_t __itsdk_sigfox_getRealTxPower(int8_t reqPower);

/**
 * All operation needed to initialize the sigfox stack
 */
itsdk_sigfox_init_t itsdk_sigfox_setup() {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_setup\r\n"));

	itsdk_sigfox_init_t ret = SIGFOX_INIT_SUCESS;
#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
	eeprom_m95640_hwInit();
	s2lp_hwInit();
	eeprom_m95640_init(&ITSDK_DRIVERS_M95640_SPI);
	s2lp_init();
	s2lp_loadConfiguration(&__s2lpConf);
	s2lp_sigfox_init(&__s2lpConf);

	itsdk_state.sigfox.rcz = __s2lpConf.rcz;
#elif ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
	itsdk_sigfox_resetFactoryDefaults(false);		// store the key if not yet done
	ret = sx1276_sigfox_init();
#endif
	if (
#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
		    itsdk_config.sdk.sigfox.txPower == SIGFOX_DEFAULT_POWER ||
#endif
		    itsdk_state.sigfox.current_power == SIGFOX_DEFAULT_POWER
		) {
		switch (itsdk_state.sigfox.rcz) {
		case SIGFOX_RCZ1:
		case SIGFOX_RCZ5:
			  itsdk_state.sigfox.current_power = __itsdk_sigfox_getRealTxPower(14);
			break;
		case SIGFOX_RCZ2:
		case SIGFOX_RCZ4:
			  itsdk_state.sigfox.current_power = __itsdk_sigfox_getRealTxPower(24);
			break;
		case SIGFOX_RCZ3C:
			  itsdk_state.sigfox.current_power = __itsdk_sigfox_getRealTxPower(16);
			break;
		default:
			ITSDK_ERROR_REPORT(ITSDK_ERROR_SIGFOX_RCZ_NOTSUPPORTED,(uint16_t)itsdk_state.sigfox.rcz);
		}
	}
	if (
#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
			itsdk_config.sdk.sigfox.speed == SIGFOX_DEFAULT_SPEED ||
#endif
			itsdk_state.sigfox.current_speed == SIGFOX_DEFAULT_SPEED ) {
		switch (itsdk_state.sigfox.rcz) {
		case SIGFOX_RCZ1:
		case SIGFOX_RCZ3C:
		case SIGFOX_RCZ5:
			itsdk_state.sigfox.current_speed = SIGFOX_SPEED_100;
			break;
		case SIGFOX_RCZ2:
		case SIGFOX_RCZ4:
			itsdk_state.sigfox.current_speed = SIGFOX_SPEED_600;
			break;
		default:
			ITSDK_ERROR_REPORT(ITSDK_ERROR_SIGFOX_RCZ_NOTSUPPORTED,(uint16_t)itsdk_state.sigfox.rcz);
		}
	}

	// Set the default power
	itsdk_sigfox_setTxPower_ext(itsdk_state.sigfox.current_power,true);

	if ( ret == SIGFOX_INIT_SUCESS ) {
		itsdk_state.sigfox.initialized = true;
	}

	return ret;
}


/**
 * This function need to be called in the project_loop function
 * to manage the sigfox stack
 */
itsdk_sigfox_init_t itsdk_sigfox_loop() {
	//LOG_DEBUG_LORAWANSTK(("itsdk_sigfox_loop\r\n"));
	// Nothing yet to do
	return SIGFOX_INIT_SUCESS;
 }

/**
 * Stop the sigfox stack and be ready for activating another stack
 */
itsdk_sigfox_init_t itsdk_sigfox_deinit() {
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		#warning "Not yets implemented"
	#elif ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		sx1276_sigfox_deinit();
	#endif
	itsdk_state.sigfox.initialized = false;
	return SIGFOX_INIT_SUCESS;
}


/**
 * Send a frame on sigfox network
 * buf - the buffer containing the data to be transmitted
 * len - the frame len in byte
 * repeat - the number of repeat expected (from 0 to 2)
 * speed - the transmission speed itdsk_sigfox_speed_t
 * power - transmission power -1 for default value
 * encrypt - type en encryption to apply to the data frame
 * ack - when true a downlink transmission is requested
 * dwn - downlink buffer for downlink reception
 */
itdsk_sigfox_txrx_t itsdk_sigfox_sendFrame(
		uint8_t * buf,
		uint8_t len,
		uint8_t repeat,
		itdsk_sigfox_speed_t speed,
		int8_t power,
		itdsk_payload_encrypt_t encrypt,
		bool ack,
		uint8_t * dwn
) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_sendFrame\r\n"));

	// some basic checking...
	if ( len > 12) return SIGFOX_ERROR_PARAMS;
	if ( repeat > 2) repeat = 2;
	if ( power == SIGFOX_POWER_DEFAULT ) power = itsdk_state.sigfox.current_power;
	else power = __itsdk_sigfox_getRealTxPower(power);
	if ( speed == SIGFOX_SPEED_DEFAULT ) speed = itsdk_state.sigfox.current_speed;
	if ( ack && (dwn == NULL)) return SIGFOX_ERROR_PARAMS;

	#if ( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_SIGFOX)
	if ( (encrypt & PAYLOAD_ENCRYPT_SIGFOX) == 0 ) {
		log_error("[Sigfox] Sigfox ITSDK_SIGFOX_ENCRYPTION must be set as encryption has been activated\r\n");
	}
	#else
	if ( (encrypt & PAYLOAD_ENCRYPT_SIGFOX) != 0 ) {
		log_error("[Sigfox] Sigfox ITSDK_SIGFOX_ENCRYPTION can't be set until encryption has been activated\r\n");
	}
	#endif

	// encrypt the frame
	#if ( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_SPECK ) > 0
		if ( (encrypt & PAYLOAD_ENCRYPT_SPECK) > 0 ) {
			uint64_t masterKey;
			itsdk_encrypt_speck_getMasterKey(&masterKey);
			itsdk_speck_encrypt(
					buf,
					buf,
					len,
					masterKey
			);
		}
	#endif
	#if (ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_AESCTR) > 0
		if ( (encrypt & PAYLOAD_ENCRYPT_AESCTR) > 0 ) {
			uint32_t devId;
			itsdk_sigfox_getDeviceId(&devId);
			uint16_t seqId;
			itsdk_sigfox_getNextSeqId(&seqId);
			uint8_t nonce;
			itsdk_encrypt_aes_getNonce(&nonce);
			uint32_t sharedKey;
			itsdk_encrypt_aes_getSharedKey(&sharedKey);
			uint8_t masterKey[16];
			itsdk_encrypt_aes_getMasterKey(masterKey);

			itsdk_aes_ctr_encrypt_128B(
					buf,							// Data to be encrypted
					buf,							// Can be the same as clearData
					len,							// Size of data to be encrypted
					devId,							// 32b device ID
					seqId,							// 16b sequenceId (incremented for each of the frame)
					nonce,							// 8b  value you can update dynamically from backend
					sharedKey,						// 24b hardcoded value (hidden with ITSDK_PROTECT_KEY)
					masterKey						// 128B key used for encryption (hidden with ITSDK_PROTECT_KEY)
			);
		}
	#endif

	// Transmit the frame
	itsdk_sigfox_setTxPower(power);
	itsdk_sigfox_setTxSpeed(speed);

	itdsk_sigfox_txrx_t result;
#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
	uint16_t ret = SIGFOX_API_send_frame(buf,len,dwn,repeat,ack);
	switch (ret&0xFF) {
	case SFX_ERR_INT_GET_RECEIVED_FRAMES_TIMEOUT:
		result = SIGFOX_TXRX_NO_DOWNLINK;
		break;
	case SFX_ERR_NONE:
		if ( ack ) {
			#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
  			  s2lp_sigfox_retreive_rssi();
			#endif
			result = SIGFOX_TXRX_DOWLINK_RECEIVED;
		} else {
			result = SIGFOX_TRANSMIT_SUCESS;
		}
		break;
	default:
		result = SIGFOX_TXRX_ERROR;
		break;
	}

#endif


	return result;
}


/**
 * Send a frame on sigfox network
 * buf - the buffer containing the data to be transmitted
 * len - the frame len in byte
 * repeat - the number of repeat expected (from 0 to 2)
 * speed - the transmission speed itdsk_sigfox_speed_t
 * power - transmission power -1 for default value
 * encrypt - type en encryption to apply to the data frame
 * ack - when true a downlink transmission is requested
 * dwn - downlink buffer for downlink reception
 */
itdsk_sigfox_txrx_t itsdk_sigfox_sendBit(
		bool bitValue,
		uint8_t repeat,
		itdsk_sigfox_speed_t speed,
		int8_t power,
		bool ack,
		uint8_t * dwn
) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_sendBit\r\n"));

	// some basic checking...
	if ( repeat > 2) repeat = 2;
	if ( power == SIGFOX_POWER_DEFAULT ) power = itsdk_state.sigfox.current_power;
	else power = __itsdk_sigfox_getRealTxPower(power);
	if ( speed == SIGFOX_SPEED_DEFAULT ) speed = itsdk_state.sigfox.current_speed;
	if ( ack && dwn == NULL) return SIGFOX_ERROR_PARAMS;

	itsdk_sigfox_setTxPower(power);
	itsdk_sigfox_setTxSpeed(speed);

	itdsk_sigfox_txrx_t result = SIGFOX_TXRX_ERROR;
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		sfx_bool value = (bitValue)?SFX_TRUE:SFX_FALSE;
		uint16_t ret = SIGFOX_API_send_bit( value,dwn,repeat,ack);
		switch (ret&0xFF) {
		case SFX_ERR_INT_GET_RECEIVED_FRAMES_TIMEOUT:
			result = SIGFOX_TXRX_NO_DOWNLINK;
			break;
		case SFX_ERR_NONE:
			if ( ack ) {
				#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
					  s2lp_sigfox_retreive_rssi();
				#endif
				result = SIGFOX_TXRX_DOWLINK_RECEIVED;
			} else {
				result = SIGFOX_TRANSMIT_SUCESS;
			}
			break;
		default:
			result = SIGFOX_TXRX_ERROR;
			break;
		}

	#endif

	return result;
}

/**
 * Send an OOB message
 */
itdsk_sigfox_txrx_t itsdk_sigfox_sendOob(
		itdsk_sigfox_oob_t oobType,
		itdsk_sigfox_speed_t speed,
		int8_t power
) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_sendOob\r\n"));

	// some basic checking...
	if ( power == SIGFOX_POWER_DEFAULT ) power = itsdk_state.sigfox.current_power;
	else power = __itsdk_sigfox_getRealTxPower(power);
	if ( speed == SIGFOX_SPEED_DEFAULT ) speed = itsdk_state.sigfox.current_speed;
	itsdk_sigfox_setTxPower(power);
	itsdk_sigfox_setTxSpeed(speed);

	itdsk_sigfox_txrx_t result = SIGFOX_TXRX_ERROR;
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276

		uint16_t ret=0;
		switch (oobType) {
		case SIGFOX_OOB_SERVICE:
			ret = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
			break;
		case SIGFOX_OOB_RC_SYNC:
			ret = SIGFOX_API_send_outofband(SFX_OOB_RC_SYNC);
			break;
		default:
			ITSDK_ERROR_REPORT(ITSDK_ERROR_SIGFOX_OOB_NOTSUPPORTED,(uint16_t)oobType);
		}
		switch (ret&0xFF) {
		case SFX_ERR_NONE:
			result = SIGFOX_TRANSMIT_SUCESS;
			break;
		default:
			result = SIGFOX_TXRX_ERROR;
			break;
		}

	#endif

	return result;
}

/**
 * Get the current RCZ
 */
itsdk_sigfox_init_t itsdk_sigfox_getCurrentRcz(uint8_t * rcz) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_getCurrentRcz\r\n"));

	*rcz = itsdk_state.sigfox.rcz;
	if ( itsdk_state.sigfox.rcz > 0 ) return SIGFOX_INIT_SUCESS;
	return SIGFOX_INIT_PARAMSERR;
}


/**
 * Change the transmission power to the given value
 */
itsdk_sigfox_init_t itsdk_sigfox_setTxPower_ext(int8_t power, bool force) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_setTxPower_ext(%d)\r\n",power));

	if ( !force && power == itsdk_state.sigfox.current_power ) return SIGFOX_INIT_NOCHANGE;


	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		sfx_s16 delta = (power - itsdk_state.sigfox.current_power)*2;
		ST_RF_API_reduce_output_power(delta);
    #elif ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		sx1276_sigfox_setPower( power );
	#endif
	itsdk_state.sigfox.current_power = power;

	return SIGFOX_INIT_SUCESS;
}

/**
 * Change the current sigfox network speed
 */
itsdk_sigfox_init_t itsdk_sigfox_setTxPower(int8_t power) {
	LOG_DEBUG_SIGFOXSTK(("itsdk_sigfox_setTxPower\r\n"));
	return itsdk_sigfox_setTxPower_ext(power,false);
}

/**
 * Get the current sigfox trasnmision power
 */
itsdk_sigfox_init_t itsdk_sigfox_getTxPower(int8_t * power) {
	LOG_DEBUG_SIGFOXSTK(("itsdk_sigfox_getTxPower\r\n"));
	*power = itsdk_state.sigfox.current_power;
	return SIGFOX_INIT_SUCESS;
}


/**
 * Change the transmission speed
 */
itsdk_sigfox_init_t itsdk_sigfox_setTxSpeed(itdsk_sigfox_speed_t speed) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_setTxSpeed\r\n"));

	if ( speed == itsdk_state.sigfox.current_speed ) return SIGFOX_INIT_NOCHANGE;

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		// not yet supported
		LOG_WARN_SIGFOXSTK(("Sigfox speed change not yet supported"));
	#elif ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		// not yet supported
		LOG_WARN_SIGFOXSTK(("Sigfox speed change not yet supported"));
	#endif

	itsdk_state.sigfox.current_speed = speed;
	return SIGFOX_INIT_SUCESS;
}

/**
 * Get the current sigfox network speed
 */
itsdk_sigfox_init_t itsdk_sigfox_getTxSpeed(itdsk_sigfox_speed_t * speed) {
	LOG_DEBUG_SIGFOXSTK(("itsdk_sigfox_getTxSpeed\r\n"));
	*speed = (itdsk_sigfox_speed_t)itsdk_state.sigfox.current_speed;
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the deviceId into the given parameter
 */
itsdk_sigfox_init_t itsdk_sigfox_getDeviceId(itsdk_sigfox_device_is_t * devId) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_getDeviceId\r\n"));

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		if ( itsdk_state.sigfox.initialized ) {
		  *devId = __s2lpConf.id;
		} else return SIGFOX_INIT_FAILED;
    #else
      #if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
		 *devId = itsdk_config.sdk.sigfox.deviceId;
      #elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_CONFIG_STATIC
		 *devId = ITSDK_SIGFOX_ID;
      #else
		#error UNSUPPORTED ITSDK_SIGFOX_NVM_SOURCE VALUE
      #endif
	#endif
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the initial PAC into the given parameter
 * The PAC parameter is a 8 Bytes table
 */
itsdk_sigfox_init_t itsdk_sigfox_getInitialPac(uint8_t * pac) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_getInitialPac\r\n"));

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		if ( itsdk_state.sigfox.initialized ) {
		  for ( int i = 0 ; i < 8 ; i++ ) {
			  pac[i] = __s2lpConf.pac[i];
		  }
		} else return SIGFOX_INIT_FAILED;
   #else
     #if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
		 bcopy(itsdk_config.sdk.sigfox.initialPac,pac,8);
     #elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_CONFIG_STATIC
		 uint8_t _pac[8] = ITSDK_SIGFOX_PAC;
		 bcopy(_pac,pac,8);
     #else
		#error UNSUPPORTED ITSDK_SIGFOX_NVM_SOURCE VALUE
     #endif
    #endif

	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the last reception RSSI into the given parameter
 * S2LP_UNKNOWN_RSSI if unknow (0x0F00);
 */
itsdk_sigfox_init_t itsdk_sigfox_getLastRssi(int16_t * rssi) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_getLastRssi\r\n"));

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		*rssi = s2lp_sigfox_getLastRssiLevel();
    #elif ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		sx1276_sigfox_getRssi(rssi);
	#endif

	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the last used seqId
 */
itsdk_sigfox_init_t itsdk_sigfox_getLastSeqId(uint16_t * seqId) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_getLastSeqId\r\n"));

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		*seqId = _s2lp_sigfox_config->seqId;
    #elif ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		sx1276_sigfox_getSeqId(seqId);
	#endif

	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the next used seqId
 */
itsdk_sigfox_init_t itsdk_sigfox_getNextSeqId(uint16_t * seqId) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_getNextSeqId\r\n"));

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		*seqId = (_s2lp_sigfox_config->seqId+1) & 0x0FFF;
    #elif ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		sx1276_sigfox_getSeqId(seqId);
		*seqId = (*seqId+1) & 0x0FFF;
	#endif

	return SIGFOX_INIT_SUCESS;
}


/**
 * Switch to public key
 */
itsdk_sigfox_init_t itsdk_sigfox_switchPublicKey() {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_switchPublicKey\r\n"));

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP  || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		SIGFOX_API_switch_public_key(true);
	#endif
	return SIGFOX_INIT_SUCESS;
}

/**
 * Switch to private key
 */
itsdk_sigfox_init_t itsdk_sigfox_switchPrivateKey() {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_switchPrivateKey\r\n"));

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
	   SIGFOX_API_switch_public_key(false);
	#endif
	return SIGFOX_INIT_SUCESS;
}


/**
 * Switch to continuous transmission (certification)
 * Give the transmission frequency
 * Give the transmission power
 */
itsdk_sigfox_init_t itsdk_sigfox_continuousModeStart(
		uint32_t				frequency,
		itdsk_sigfox_speed_t 	speed,
		int8_t 					power
) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_continuousModeStart\r\n"));

	if ( power == SIGFOX_POWER_DEFAULT ) power = itsdk_state.sigfox.current_power;
	if ( speed == SIGFOX_SPEED_DEFAULT ) speed = itsdk_state.sigfox.current_speed;
	itsdk_sigfox_setTxPower(power);

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		switch (speed) {
		case SIGFOX_SPEED_100:
			SIGFOX_API_start_continuous_transmission (frequency, SFX_DBPSK_100BPS);
			break;
		case SIGFOX_SPEED_600:
			SIGFOX_API_start_continuous_transmission (frequency, SFX_DBPSK_600BPS);
			break;
		default:
			return SIGFOX_INIT_PARAMSERR;
		}
	#endif

	return SIGFOX_INIT_SUCESS;
}

/**
 * Stop continuous transmission (certification)
 */
itsdk_sigfox_init_t itsdk_sigfox_continuousModeStop() {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_continuousModeStop\r\n"));

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		SIGFOX_API_stop_continuous_transmission();
	#endif

	return SIGFOX_INIT_SUCESS;
}


/**
 * Change the RC Sync Period (sigfox payload encryption counter synchronization)
 * The default value is every 4096 frame when the seqId is back to 0
 * The given value is the number of frames
 */
itsdk_sigfox_init_t itsdk_sigfox_setRcSyncPeriod(uint16_t numOfFrame) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_setRcSyncPeriod\r\n"));

	if ( numOfFrame > 4096 ) return SIGFOX_INIT_PARAMSERR;
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
		SIGFOX_API_set_rc_sync_period(numOfFrame);
	#endif

	return SIGFOX_INIT_SUCESS;

}


/**
 * Get the Sigfox lib version in use
 * A string is returned terminated by \0
 */
itsdk_sigfox_init_t itsdk_sigfox_getSigfoxLibVersion(uint8_t ** version){
#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP || ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
	sfx_u8 __size;
	SIGFOX_API_get_version(version, &__size, VERSION_SIGFOX);
#endif
	return SIGFOX_INIT_SUCESS;
}

// ================================================================================
// MANAGE THE NVM STORAGE FOR SIGFOX LIBS
// ================================================================================


#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM

/**
 * Return the size of the sigfox Nvm memory to reserve
 */
itsdk_sigfox_init_t itsdk_sigfox_getNvmSize(uint32_t * sz) {
	*sz = (   sizeof(itsdk_sigfox_nvm_header_t)
			+ itdt_align_32b(SFX_NVMEM_BLOCK_SIZE)
			+ itdt_align_32b(SFX_SE_NVMEM_BLOCK_SIZE)
		  );
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the offset of the NVM area for Sigfox
 */
itsdk_sigfox_init_t itsdk_sigfox_getNvmOffset(uint32_t * offset) {
	itsdk_sigfox_getSigfoxNvmOffset(offset);
	*offset += sizeof(itsdk_sigfox_nvm_header_t);
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the offset of the NVM area for Sigfox Secure Element
 */
itsdk_sigfox_init_t itsdk_sigfox_getSeNvmOffset(uint32_t * offset) {
	itsdk_sigfox_getNvmOffset(offset);
	int size = itdt_align_32b(SFX_NVMEM_BLOCK_SIZE);
	*offset += size;
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the offset of the NVM area for Sigfox
 * Data including the Lib Nvm Offset followed by the
 * the SE offset
 */
itsdk_sigfox_init_t itsdk_sigfox_getSigfoxNvmOffset(uint32_t * offset) {

	uint32_t sstore=0, ssError=0;
	#if ITSDK_WITH_SECURESTORE == __ENABLE
	itsdk_secstore_getStoreSize(&sstore);
	#endif
	#if (ITSDK_WITH_ERROR_RPT == __ENABLE) && (ITSDK_ERROR_USE_EPROM == __ENABLE)
	itsdk_error_getSize(&ssError);
	#endif
	*offset = sstore + ssError;
	return SIGFOX_INIT_SUCESS;
}


/**
 * Configure the default values for the NVM Areas
 */
itsdk_sigfox_init_t __itsdk_sigfox_resetNvmToFactory(bool force) {
	LOG_INFO_SIGFOXSTK(("__itsdk_sigfox_resetNvmToFactory"));

	uint32_t offset;
	itsdk_sigfox_getSigfoxNvmOffset(&offset);

	itsdk_sigfox_nvm_header_t header;
	_eeprom_read(ITDT_EEPROM_BANK0, offset, (void *) &header, sizeof(itsdk_sigfox_nvm_header_t));
	uint32_t expecteSize;
	itsdk_sigfox_getNvmSize(&expecteSize);
	if ( force || header.magic != ITSDK_SIGFOX_NVM_MAGIC || header.size != expecteSize ) {
		LOG_INFO_SIGFOXSTK((".. Reset\r\n"));
		header.magic = ITSDK_SIGFOX_NVM_MAGIC;
		header.size = expecteSize;
		header.reserved = 0;
		_eeprom_write(ITDT_EEPROM_BANK0, offset, (void *) &header, sizeof(itsdk_sigfox_nvm_header_t));
		// force to reset the Sigfox NVM structure
		uint8_t se_nvm_default[SFX_SE_NVMEM_BLOCK_SIZE] = { 0, 0, 0, 0x0F, 0xFF };
		SE_NVM_set(se_nvm_default);
		uint8_t se_mcu_default[SFX_NVMEM_BLOCK_SIZE];
		bzero(se_mcu_default,SFX_NVMEM_BLOCK_SIZE);
		MCU_API_set_nv_mem(se_mcu_default);
	} else {
		LOG_INFO_SIGFOXSTK((".. Skiped\r\n"));
	}
	return SIGFOX_INIT_SUCESS;
}

#endif

// ===================================================================================
// Region conversion
// ===================================================================================
itsdk_sigfox_init_t itsdk_sigfox_getRczFromRegion(uint32_t region, uint8_t * rcz) {
	switch ( region ) {
	case __LPWAN_REGION_EU868:
	case __LPWAN_REGION_MEA868:
		*rcz = SIGFOX_RCZ1;
		break;
	case __LPWAN_REGION_US915:
	case __LPWAN_REGION_SA915:
		*rcz = SIGFOX_RCZ2;
		break;
	case __LPWAN_REGION_JP923:
		*rcz = SIGFOX_RCZ3C;		// to be verified as RCZ3a sound more relevant
		break;
	case __LPWAN_REGION_AU915:
	case __LPWAN_REGION_SA920:
	case __LPWAN_REGION_AP920:
		*rcz = SIGFOX_RCZ4;
		break;
	case __LPWAN_REGION_KR920:
		*rcz = SIGFOX_RCZ5;
		break;
	case __LPWAN_REGION_IN865:
		//*rcz = SIGFOX_RCZ6;
		*rcz = SIGFOX_UNSUPPORTED;
		break;
	default:
		*rcz = SIGFOX_UNSUPPORTED;
		break;
	}
	if ( *rcz == SIGFOX_UNSUPPORTED ) {
		return SIGFOX_INIT_FAILED;
	}
	return SIGFOX_INIT_SUCESS;
}

// ===================================================================================
// Tx Power management
// ===================================================================================
int8_t __itsdk_sigfox_getRealTxPower(int8_t reqPower) {
  LOG_INFO_SIGFOXSTK(("__itsdk_sigfox_getRealTxPower(%d)",reqPower));
  #ifdef ITSDK_RADIO_POWER_OFFSET
	reqPower += ITSDK_RADIO_POWER_OFFSET;
  #endif
  #ifdef ITSDK_RADIO_MAX_OUTPUT_DBM
    if ( ITSDK_RADIO_MAX_OUTPUT_DBM < reqPower ) {
 	  reqPower = ITSDK_RADIO_MAX_OUTPUT_DBM;
 	}
  #endif
  LOG_INFO_SIGFOXSTK(("(%d)\r\n",reqPower));
  return reqPower;

}

// ===================================================================================
// Overloadable functions
// ===================================================================================


/**
 * Configure the SecureStore with the Static values obtained from configSigfox.h
 * When force is false, the secure store will be refreshed only if there is no
 * configuration already setup.
 */
#if ITSDK_WITH_SECURESTORE == __ENABLE
itsdk_sigfox_init_t itsdk_sigfox_resetFactoryDefaults(bool force) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_resetFactoryDefaults\r\n"));

	uint8_t buffer[16];
	if ( force || itsdk_secstore_readBlock(ITSDK_SS_SIGFOXKEY, buffer) != SS_SUCCESS ) {
		uint8_t key[16] = ITSDK_SIGFOX_KEY;
		itsdk_encrypt_cifferKey(key,16);
		itsdk_secstore_writeBlock(ITSDK_SS_SIGFOXKEY, key);
		bzero(key,16);
	}
	bzero(buffer,16);
	return SIGFOX_INIT_SUCESS;
}
#else
itsdk_sigfox_init_t itsdk_sigfox_resetFactoryDefaults(bool force) {
	return SIGFOX_INIT_SUCESS;
}
#endif

/**
 * Get the sigfoxKey as a uint8_t[]
 */
__weak itsdk_sigfox_init_t itsdk_sigfox_getKEY(uint8_t * key) {
	LOG_INFO_SIGFOXSTK(("itsdk_sigfox_getKEY\r\n"));
#if ITSDK_WITH_SECURESTORE == __ENABLE
	uint8_t d[16];
	if ( itsdk_secstore_readBlock(ITSDK_SS_SIGFOXKEY, d) != SS_SUCCESS ) {
		#if ITSDK_WITH_ERROR_RPT == __ENABLE
			ITSDK_ERROR_REPORT(ITSDK_ERROR_SIGFOX_SS_INVALID,0);
		#endif
		bzero(key,16);
		return SIGFOX_INIT_FAILED;
	}
#else
	uint8_t d[16] = ITSDK_SIGFOX_KEY;
	itsdk_encrypt_cifferKey(d,16);
#endif
	memcpy(key,d,16);
	return SIGFOX_INIT_SUCESS;
}



#endif // ITSDK_WITH_SIGFOX_LIB

