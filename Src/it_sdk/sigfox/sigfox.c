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
#include <it_sdk/eeprom/securestore.h>

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



#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
s2lp_config_t __s2lpConf;
#endif

itsdk_sigfox_state __sigfox_state = {0};

/**
 * All operation needed to initialize the sigfox stack
 */
itsdk_sigfox_init_t itsdk_sigfox_setup() {

	itsdk_sigfox_init_t ret = SIGFOX_INIT_SUCESS;
#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
	eeprom_m95640_hwInit();
	s2lp_hwInit();
	eeprom_m95640_init(&ITSDK_DRIVERS_M95640_SPI);
	s2lp_init();
	s2lp_loadConfiguration(&__s2lpConf);
	s2lp_sigfox_init(&__s2lpConf);

	__sigfox_state.rcz = __s2lpConf.rcz;
#elif ITSDK_SIGFOX_LIB == __SIGFOX_SX1276
	ret = sx1276_sigfox_init(&__sigfox_state);
#endif

	switch (__sigfox_state.rcz) {
	case 1:
		__sigfox_state.default_power = 14;
		__sigfox_state.default_speed = 100;
		break;
	case 2:
		__sigfox_state.default_power = 24;
		__sigfox_state.default_speed = 600;
		break;
	case 3:
		__sigfox_state.default_power = 16;
		__sigfox_state.default_speed = 100;
		break;
	case 4:
		__sigfox_state.default_power = 24;
		__sigfox_state.default_speed = 600;
		break;
	case 5:
		__sigfox_state.default_power = 14;
		__sigfox_state.default_speed = 100;
		break;
	default:
		ITSDK_ERROR_REPORT(ITSDK_ERROR_SIGFOX_RCZ_NOTSUPPORTED,(uint16_t)__sigfox_state.rcz);
	}

	if ( ret == SIGFOX_INIT_SUCESS ) {
	   __sigfox_state.initialized = true;
	}
	return ret;
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

	// some basic checking...
	if ( len > 12) return SIGFOX_ERROR_PARAMS;
	if ( repeat > 2) repeat = 2;
	if ( power == SIGFOX_POWER_DEFAULT ) power = __sigfox_state.default_power;
	if ( speed == SIGFOX_SPEED_DEFAULT ) speed = __sigfox_state.default_speed;
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
			itsdk_sigfox_speck_getMasterKey(&masterKey);
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
			itsdk_sigfox_aes_getNonce(&nonce);
			uint32_t sharedKey;
			itsdk_sigfox_aes_getSharedKey(&sharedKey);
			uint8_t masterKey[16];
			itsdk_sigfox_aes_getMasterKey(masterKey);

			itsdk_aes_crt_encrypt_128B(
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
#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
	uint16_t ret = SIGFOX_API_send_frame(buf,len,dwn,repeat,ack);
	switch (ret&0xFF) {
	case SFX_ERR_INT_GET_RECEIVED_FRAMES_TIMEOUT:
		result = SIGFOX_TXRX_NO_DOWNLINK;
		break;
	case SFX_ERR_NONE:
		if ( ack ) {
			s2lp_sigfox_retreive_rssi();
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

	// some basic checking...
	if ( repeat > 2) repeat = 2;
	if ( power == SIGFOX_POWER_DEFAULT ) power = __sigfox_state.default_power;
	if ( speed == SIGFOX_SPEED_DEFAULT ) speed = __sigfox_state.default_speed;
	if ( ack && dwn == NULL) return SIGFOX_ERROR_PARAMS;

	itsdk_sigfox_setTxPower(power);
	itsdk_sigfox_setTxSpeed(speed);

	itdsk_sigfox_txrx_t result = SIGFOX_TXRX_ERROR;
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP

		uint16_t ret = SIGFOX_API_send_bit( bitValue,dwn,repeat,ack);
		switch (ret&0xFF) {
		case SFX_ERR_INT_GET_RECEIVED_FRAMES_TIMEOUT:
			result = SIGFOX_TXRX_NO_DOWNLINK;
			break;
		case SFX_ERR_NONE:
			if ( ack ) {
				s2lp_sigfox_retreive_rssi();
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

	// some basic checking...
	if ( power == SIGFOX_POWER_DEFAULT ) power = __sigfox_state.default_power;
	if ( speed == SIGFOX_SPEED_DEFAULT ) speed = __sigfox_state.default_speed;
	itsdk_sigfox_setTxPower(power);
	itsdk_sigfox_setTxSpeed(speed);

	itdsk_sigfox_txrx_t result = SIGFOX_TXRX_ERROR;
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP

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
	*rcz = __sigfox_state.rcz;
	if ( __sigfox_state.rcz > 0 ) return SIGFOX_INIT_SUCESS;
	return SIGFOX_INIT_PARAMSERR;
}


/**
 * Change the transmission power to the given value
 */
itsdk_sigfox_init_t itsdk_sigfox_setTxPower(uint8_t power) {
	if ( power == __sigfox_state.default_power ) return SIGFOX_INIT_NOCHANGE;

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP

		sfx_s16 delta = (power - __sigfox_state.default_power)*2;
		ST_RF_API_reduce_output_power(delta);

	#endif

	__sigfox_state.default_power = power;
	return SIGFOX_INIT_SUCESS;
}


/**
 * Change the transmission speed
 */
itsdk_sigfox_init_t itsdk_sigfox_setTxSpeed(itdsk_sigfox_speed_t speed) {
	if ( speed == __sigfox_state.default_speed ) return SIGFOX_INIT_NOCHANGE;

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		// not yet supported
		LOG_WARN_SIGFOX(("Sigfox speed change not yet supported"));
	#endif

	__sigfox_state.default_speed = speed;
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the deviceId into the given parameter
 */
itsdk_sigfox_init_t itsdk_sigfox_getDeviceId(itsdk_sigfox_device_is_t * devId) {
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		if ( __sigfox_state.initialized ) {
		  *devId = __s2lpConf.id;
		} else return SIGFOX_INIT_FAILED;
	#endif

	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the initial PAC into the given parameter
 * The PAC parameter is a 8 Bytes table
 */
itsdk_sigfox_init_t itsdk_sigfox_getInitialPac(uint8_t * pac) {
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		if ( __sigfox_state.initialized ) {
		  for ( int i = 0 ; i < 8 ; i++ ) {
			  pac[i] = __s2lpConf.pac[i];
		  }
		} else return SIGFOX_INIT_FAILED;
	#endif

	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the last reception RSSI into the given parameter
 * S2LP_UNKNOWN_RSSI if unknow (0x0F00);
 */
itsdk_sigfox_init_t itsdk_sigfox_getLastRssi(int16_t * rssi) {
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		*rssi = s2lp_sigfox_getLastRssiLevel();
	#endif

	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the last used seqId
 */
itsdk_sigfox_init_t itsdk_sigfox_getLastSeqId(uint16_t * seqId) {
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		*seqId = _s2lp_sigfox_config->seqId;
	#endif

	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the next used seqId
 */
itsdk_sigfox_init_t itsdk_sigfox_getNextSeqId(uint16_t * seqId) {
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		*seqId = (_s2lp_sigfox_config->seqId+1) & 0x0FFF;
	#endif

	return SIGFOX_INIT_SUCESS;
}



/**
 * Switch to public key
 */
itsdk_sigfox_init_t itsdk_sigfox_switchPublicKey() {
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		SIGFOX_API_switch_public_key(true);
	#endif
	return SIGFOX_INIT_SUCESS;
}

/**
 * Switch to private key
 */
itsdk_sigfox_init_t itsdk_sigfox_switchPrivateKey() {
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
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

	if ( power == SIGFOX_POWER_DEFAULT ) power = __sigfox_state.default_power;
	if ( speed == SIGFOX_SPEED_DEFAULT ) speed = __sigfox_state.default_speed;
	itsdk_sigfox_setTxPower(power);

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
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

	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
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

	if ( numOfFrame > 4096 ) return SIGFOX_INIT_PARAMSERR;
	#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
		SIGFOX_API_set_rc_sync_period(numOfFrame);
	#endif

	return SIGFOX_INIT_SUCESS;

}


/**
 * Get the Sigfox lib version in use
 * A string is returned terminated by \0
 */
itsdk_sigfox_init_t itsdk_sigfox_getSigfoxLibVersion(uint8_t ** version){
#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
	sfx_u8 __size;
	SIGFOX_API_get_version(version, &__size, VERSION_SIGFOX);
#endif
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the size of the sigfox Nvm memory to reserve
 */
itsdk_sigfox_init_t itsdk_sigfox_getNvmSize(uint32_t * sz) {
	*sz = ( SFX_NVMEM_BLOCK_SIZE + SFX_SE_NVMEM_BLOCK_SIZE );
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the offset of the NVM area for Sigfox
 */
itsdk_sigfox_init_t itsdk_sigfox_getNvmOffset(uint32_t * offset) {
	uint32_t sstore=0, ssError=0;
	#if ITSDK_WITH_SECURESTORE == __ENABLE
	itsdk_secstore_getStoreSize(&sstore);
	#endif
	#if (ITSDK_WITH_ERROR_RPT == __ENABLE) && (ITSDK_ERROR_USE_EPROM == __ENABLE)
	itsdk_error_getSize(&ssError);
	#endif
	*offset += sstore + ssError;
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return the offset of the NVM area for Sigfox Secure Element
 */
itsdk_sigfox_init_t itsdk_sigfox_getSeNvmOffset(uint32_t * offset) {
	itsdk_sigfox_getNvmOffset(offset);
	*offset += SFX_NVMEM_BLOCK_SIZE;
	return SIGFOX_INIT_SUCESS;
}

/**
 * Configure the default values for the NVM Areas
 */
itsdk_sigfox_init_t __itsdk_sigfox_resetNvmToFactory() {
	uint8_t se_nvm_default[SFX_SE_NVMEM_BLOCK_SIZE] = { 0xFF, 0, 0, 0x0F, 0xFF };
	SE_NVM_set(se_nvm_default);
	uint8_t se_mcu_default[SFX_NVMEM_BLOCK_SIZE] = { 0, 0, 0, 0, 0, 0, 0 };
	MCU_API_set_nv_mem(se_mcu_default);
	return SIGFOX_INIT_SUCESS;
}

// ===================================================================================
// Overloadable functions
// ===================================================================================

/**
 * Return default nonce, this function is overloaded in the main program
 * to return a dynamic value
 */
__weak  itsdk_sigfox_init_t itsdk_sigfox_aes_getNonce(uint8_t * nonce) {
	*nonce = ITSDK_SIGFOX_AES_INITALNONCE;
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return default sharedKey, this function is overloaded in the main program
 * to return a dynamic value
 */
__weak  itsdk_sigfox_init_t itsdk_sigfox_aes_getSharedKey(uint32_t * sharedKey) {
	*sharedKey = ITSDK_SIGFOX_AES_SHAREDKEY;
	return SIGFOX_INIT_SUCESS;
}


/**
 * Return default masterKey (protected by ITSDK_PROTECT_KEY), this function is overloaded in the main program
 * to return a dynamic value when needed
 */
__weak  itsdk_sigfox_init_t itsdk_sigfox_aes_getMasterKey(uint8_t * masterKey) {
#if ITSDK_SIGFOX_LIB ==	__SIGFOX_S2LP
	bcopy((void *)_s2lp_sigfox_config->key,(void *)masterKey,16);
#else
	uint8_t tmp[16] = ITSDK_SIGFOX_KEY;
	itsdk_encrypt_cifferKey(tmp,16);
	bcopy((void *)tmp,(void *)masterKey,16);
#endif
	return SIGFOX_INIT_SUCESS;
}

/**
 * Return default speck Key ( protected by ITSDK_PROTECT_KEY), this function is overloaded in the main program
 * ro return a dynamic value when needed
 */
__weak  itsdk_sigfox_init_t itsdk_sigfox_speck_getMasterKey(uint64_t * masterKey) {
	*masterKey = ITSDK_SIGFOX_SPECKKEY;
	return SIGFOX_INIT_SUCESS;
}

#endif // ITSDK_WITH_SIGFOX_LIB

