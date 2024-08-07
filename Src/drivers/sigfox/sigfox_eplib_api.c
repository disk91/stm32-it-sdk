/* ==========================================================
 * sigfox_eplib_api.c - implementation for sigfox library, board api
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
 *  Created on: 04 may 2024
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2024
 * ==========================================================
 */
#include <it_sdk/config.h>

#if  ITSDK_WITH_SIGFOX_LIB == __ENABLE && ITSDK_SIGFOX_LIB == __SIGFOX_SX126X
//#include <3rdParties/sigfox/sigfox-ep-lib/inc/sigfox_ep_flags.h>
//#include <3rdParties/sigfox/sigfox-ep-lib/inc/manuf/mcu_api.h>

#include "sigfox_ep_lib_version.h"
#include "manuf/mcu_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h" // 3rdParties/sigfox/sigfox-ep-lib/inc/sigfox_ep_flags.h
#endif

#include <string.h>
#include <stdio.h>
#include <it_sdk/eeprom/eeprom.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/time/timer.h>
#include <it_sdk/time/time.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/lowpower/lowpower.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <stm32l_sdk/rtc/rtc.h>

#include <drivers/sigfox/sigfox_eplib_api.h>
#include <sigfox_error.h>



// ===========================================================
// MCU API
// -----------------------------------------------------------
//
// ===========================================================

// -----------------------------------------------------------
// Init SIGFOX API
// -----------------------------------------------------------
#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
MCU_API_status_t MCU_API_open(MCU_API_config_t *config) {

    _LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_open()\r\n"));

    MCU_API_timer_stop(MCU_API_TIMER_1);
    #if defined BIDIRECTIONAL
    MCU_API_timer_stop(MCU_API_TIMER_2);
	#endif
	#if defined CERTIFICATION
    MCU_API_timer_stop(MCU_API_TIMER_3);
    #endif
	/*
   ADC_Init();
   NVM_Init();
	 */
#ifdef ERROR_CODES
    return MCU_API_SUCCESS;
#endif
}
#endif


// -----------------------------------------------------------
// Terminate SIGFOX API
// -----------------------------------------------------------
#ifdef LOW_LEVEL_OPEN_CLOSE
MCU_API_status_t MCU_API_close(void) {
    _LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_close()\r\n"));

    MCU_API_timer_stop(MCU_API_TIMER_1);
    #if defined BIDIRECTIONAL
     MCU_API_timer_stop(MCU_API_TIMER_2);
	#endif
	#if defined CERTIFICATION
     MCU_API_timer_stop(MCU_API_TIMER_3);
    #endif

	/*
    ADC_Uninit();
    NVM_Uninit();
    */
	#ifdef ERROR_CODES
      return MCU_API_SUCCESS;
	#endif
}
#endif

// -----------------------------------------------------------
// Call on critical error
// -----------------------------------------------------------
void MCU_API_error(void) {
    _LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_error()\r\n"));

    MCU_API_timer_stop(MCU_API_TIMER_1);
    #if defined BIDIRECTIONAL
    MCU_API_timer_stop(MCU_API_TIMER_2);
	#endif
	#if defined CERTIFICATION
    MCU_API_timer_stop(MCU_API_TIMER_3);
    #endif
    return;
}

// -----------------------------------------------------------
// What to be executed during Async process ???
// -----------------------------------------------------------
#ifdef ASYNCHRONOUS
MCU_API_status_t MCU_API_process(void)
{
	_LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_process()\r\n"));
	#ifdef ERROR_CODES
     return MCU_API_SUCCESS;
	#endif
}
#endif


// -----------------------------------------------------------
// Compensate latency from the timers: The sigfox library
// is automatically compensating the MCU latency based on
// the given API
// -----------------------------------------------------------
#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION) && (defined BIDIRECTIONAL)
MCU_API_status_t MCU_API_get_latency(MCU_API_latency_t latency_type, sfx_u32 *latency_ms) {
	#ifdef ERROR_CODES
    MCU_API_status_t status = MCU_API_SUCCESS;
	#endif
    switch (latency_type) {
        case MCU_API_LATENCY_GET_VOLTAGE_TEMPERATURE:
            *latency_ms = ITSDK_SFX_SX126X_LATENCYMS;
            break;
        default :
            break;
    }
    RETURN();
}
#endif

// ===========================================================
// Timers Management
// -----------------------------------------------------------
// Timer interface
// - 3 timer can be used
//   TIMER1 - for normal communication including
//				- inter frame
//				- ...
//	 TIMER2 - for downlink
//
//	 TIMER3 - for certifications
//
// A timer is identified by a Instance number in the structure
//
// ===========================================================



// -----------------------------------------------------------
// Start a timer, it can be synchronous or asynchronous
// -----------------------------------------------------------
#ifdef TIMER_REQUIRED

#ifdef ASYNCHRONOUS
	MCU_API_timer_cplt_cb_t __cplt_cb[MCU_API_TIMER_LAST];
	void __timer_cb(uint32_t v) {
		v-=ITSDK_SFX_SX126X_TMBASE;
	    _LOG_SFXEPLIB_DEBUG(("[SFX] timer callback (%d)\r\n",v));
	    if ( v < MCU_API_TIMER_LAST) {
	       __cplt_cb[v]();
	    }
	}
#endif
MCU_API_status_t MCU_API_timer_start(MCU_API_timer_t *timer) {
	#ifdef ERROR_CODES
    MCU_API_status_t status = MCU_API_SUCCESS;
	#endif
    itsdk_timer_return_t timerStatus = TIMER_INIT_SUCCESS;
	#ifdef ASYNCHRONOUS
		_LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_timer_start(%d,%d,0x%lX)\r\n",timer->instance, timer->duration_ms,timer->cplt_cb));
	#else
		_LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_timer_start(%d,%d)\r\n",timer->instance, timer->duration_ms));
	#endif
	switch (timer->instance) {
        case MCU_API_TIMER_1:
        	// see below why we do this change
        	if ( timer->duration_ms > 24000 ) timer->duration_ms += ITSDK_SFX_SX126X_RXWINEXT;
	  #if defined BIDIRECTIONAL
        case MCU_API_TIMER_2:
        	// this timer is a bit shorter than expected
        	// measured 18600ms for 20000ms expected
        	// The only explaination I found is a drift of the RTC clock
        	// during radio emission
        	//
        	// best solution is to extend the reception windows to cover this difference
	  #endif
	  #if defined CERTIFICATION
        case MCU_API_TIMER_3:
	  #endif
          #ifdef ASYNCHRONOUS
        	itsdk_time_set_ms(rtc_getTimestampMs());					// resync time
        	__cplt_cb[timer->instance] = timer->cplt_cb;
        	timerStatus = itsdk_stimer_register(
        			timer->duration_ms,									// duration
					__timer_cb,											// callback function on timer expire
					ITSDK_SFX_SX126X_TMBASE+(uint32_t)timer->instance,	// timer instance for identification
					TIMER_ACCEPT_LOWPOWER
        	);
		  #else
        	itsdk_time_set_ms(rtc_getTimestampMs());					// resync time
            timerStatus = itsdk_stimer_register(
        			timer->duration_ms,				// duration
					NULL,							// no call back function, polling
					ITSDK_SFX_SX126X_TMBASE+(uint32_t)timer->instance,	// identify timer
					TIMER_ACCEPT_LOWPOWER
        	);
		  #endif
            if (timerStatus != TIMER_INIT_SUCCESS)
                EXIT_ERROR(MCU_API_ERROR);
            break;
        default:
        	_LOG_SFXEPLIB_ERROR(("[ERROR] MCU_API_timer_start unexpected Timer(%d)\r\n",timer->instance));
            EXIT_ERROR(MCU_API_ERROR);
    }

    RETURN();
errors:
	_LOG_SFXEPLIB_ERROR(("[ERROR] MCU_API_timer_start failed for timer (%d): %d\r\n",timer->instance, timerStatus));
    RETURN();
}
#endif

// -----------------------------------------------------------
// Stop a running timer
// -----------------------------------------------------------
#ifdef TIMER_REQUIRED
MCU_API_status_t MCU_API_timer_stop(MCU_API_timer_instance_t timer_instance) {

	#ifdef ERROR_CODES
    MCU_API_status_t status = MCU_API_SUCCESS;
	#endif
	itsdk_timer_return_t timerStatus = TIMER_INIT_SUCCESS;

    _LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_timer_stop(%d)\r\n", timer_instance));
    switch (timer_instance) {
        case MCU_API_TIMER_1:
  	  #ifdef BIDIRECTIONAL
	    case MCU_API_TIMER_2:
	  #endif
	  #ifdef CERTIFICATION
	    case MCU_API_TIMER_3:
	  #endif
	    	timerStatus = itsdk_stimer_stop_1(
        		NULL,				// cbFunction not verified, so we don't care
				ITSDK_SFX_SX126X_TMBASE+(uint32_t)timer_instance,
				false
        	);
	    	if ( timerStatus != TIMER_NOT_FOUND ) {		// call on init when the timer is not initialized, not a problem
              if ( timerStatus != TIMER_INIT_SUCCESS)
                EXIT_ERROR(MCU_API_ERROR);
	    	}
            break;
        default:
            EXIT_ERROR(MCU_API_ERROR);
    }

    RETURN();
errors:
	_LOG_SFXEPLIB_ERROR(("[ERROR] MCU_API_timer_stop failed for timer (%d): %d\r\n",timer_instance, timerStatus));
	RETURN();
}
#endif

// -----------------------------------------------------------
// Get Timer Status return SFX_TRUE when the timer elapsed
// run the timer in background and refresh watchdog as it
// is called in a loop
// -----------------------------------------------------------
#if (defined TIMER_REQUIRED) && !(defined ASYNCHRONOUS)
MCU_API_status_t MCU_API_timer_status(MCU_API_timer_instance_t timer_instance, sfx_bool *timer_has_elapsed) {
	#ifdef ERROR_CODES
      MCU_API_status_t status = MCU_API_SUCCESS;
	#endif

    *timer_has_elapsed = SFX_FALSE;
	itsdk_stimer_run();
    switch (timer_instance) {
        case MCU_API_TIMER_1:
	  #if defined BIDIRECTIONAL
        case MCU_API_TIMER_2:
	  #endif
	  #if defined CERTIFICATION
        case MCU_API_TIMER_3:
	  #endif

        	if ( ! itsdk_stimer_isRunning_1(NULL,ITSDK_SFX_SX126X_TMBASE+(uint32_t)timer_instance,false) ) {
				*timer_has_elapsed = SFX_TRUE;
				_LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_timer_status elapsed (%d)\r\n", timer_instance));
			}

            break;
        default:
            EXIT_ERROR(MCU_API_ERROR);
    }

    #if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
		wdg_refresh();
	#endif

    return MCU_API_SUCCESS;
errors:
	_LOG_SFXEPLIB_ERROR(("[ERROR] MCU_API_timer_status unknown timer (%d)\r\n",timer_instance));
    RETURN();
}
#endif


// -----------------------------------------------------------
// Blocking timer wait
// -----------------------------------------------------------
#if (defined TIMER_REQUIRED) && !(defined ASYNCHRONOUS)
MCU_API_status_t MCU_API_timer_wait_cplt(MCU_API_timer_instance_t timer_instance) {

  	#ifdef ERROR_CODES
    MCU_API_status_t status = MCU_API_SUCCESS;
	#endif

    _LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_timer_wait_cplt(%d)\r\n", timer_instance));
    switch (timer_instance) {
        case MCU_API_TIMER_1:
	  #if defined BIDIRECTIONAL
        case MCU_API_TIMER_2:
	  #endif
	  #if defined CERTIFICATION
        case MCU_API_TIMER_3:
	  #endif
        	while( itsdk_stimer_isRunning_1(NULL,ITSDK_SFX_SX126X_TMBASE+(uint32_t)timer_instance,false) ) {
				#if ITSDK_SIGFOX_LOWPOWER == 1
        			lowPower_delayMs(1000); // this is a maximum, the delayMs returns on next timer expiration
				#else
        			itsdk_delayMs(10);
				#endif
				#if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
        			wdg_refresh();
				#endif
        		itsdk_stimer_run();
        	}
            break;
        default:
        	_LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_timer_wait_cplt invalid timer\r\n"));
            EXIT_ERROR(MCU_API_ERROR);
            break;
    }
	 _LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_timer_wait_cplt done\r\n"));

    return MCU_API_SUCCESS;
errors:
    RETURN();
}
#endif

// ===========================================================
// NVM & Credential API
// -----------------------------------------------------------
//
// ===========================================================


// -----------------------------------------------------------
// Get the device ID from the configuration
// as ID is a 32bits number we need to convert it in a byte array
// in the right order
// -----------------------------------------------------------
MCU_API_status_t MCU_API_get_ep_id(sfx_u8 *ep_id, sfx_u8 ep_id_size_bytes) {

	#ifdef ERROR_CODES
	  MCU_API_status_t status = MCU_API_SUCCESS;
	#endif

	uint32_t devId;
	#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
	   devId = itsdk_config.sdk.sigfox.deviceId;
	#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_CONFIG_STATIC
	   devId = ITSDK_SIGFOX_ID;
	#else
		#error UNSUPPORTED ITSDK_SIGFOX_NVM_SOURCE VALUE
	#endif

	// device ID should be 32b
	if (ep_id_size_bytes == 4) {
		// find the right order for bytes
		ep_id[0] = ( devId & 0xFF000000 ) >> 24;
		ep_id[1] = ( devId & 0x00FF0000 ) >> 16;
		ep_id[2] = ( devId & 0x0000FF00 ) >> 8;
		ep_id[3] = ( devId & 0x000000FF );
	} else EXIT_ERROR(MCU_API_ERROR);

	return MCU_API_SUCCESS;

errors:
    RETURN();

}

#if ITSDK_SFX_SX126X_NVMUPD > 1
// store in memory the nvm content to reduce the eeprom access
uint8_t __sx126x_sigfox_nvm[SIGFOX_NVM_DATA_SIZE_BYTES];
uint8_t __sx126x_sigfox_nvm_acc = 0xFF;
#endif

// -----------------------------------------------------------
// Read the NVM from sigfox NVM base and for the given number
// of bytes. This is a byte NVM access.
// Make sure the EEPROM implementation supports byte read and
// alignment on any addresses
//
// The NVM contains only 4 bytes and they are all read at once usually
// 2 bytes for random value (LSB then MSB)
// 2 bytes for SeqId value (LSB then MSB)
// The eeprom is not initialized on start, need to be made
// on eeprom init
// -----------------------------------------------------------
MCU_API_status_t MCU_API_get_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes) {

	#ifdef ERROR_CODES
      MCU_API_status_t status = MCU_API_SUCCESS;
	#endif

	#if ITSDK_SFX_SX126X_NVMUPD > 1
    	 if ( __sx126x_sigfox_nvm_acc == 0xFF ) {
   			uint32_t offset;
   			itsdk_sigfox_getNvmOffset(&offset);
   			_eeprom_read(ITDT_EEPROM_BANK0, offset, (void *) __sx126x_sigfox_nvm, SIGFOX_NVM_DATA_SIZE_BYTES);
   			__sx126x_sigfox_nvm_acc = ITSDK_SFX_SX126X_NVMUPD;
    	 }
    	 bcopy(__sx126x_sigfox_nvm,nvm_data,nvm_data_size_bytes);
	#else
		uint32_t offset;
		itsdk_sigfox_getNvmOffset(&offset);
		_eeprom_read(ITDT_EEPROM_BANK0, offset, (void *) nvm_data, nvm_data_size_bytes);
	#endif
    _LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_get_nvm(%d) [%02X,%02X,%02X,%02X]\r\n", nvm_data_size_bytes,nvm_data[0],nvm_data[1],nvm_data[2],nvm_data[3]));

    RETURN();
}

// -----------------------------------------------------------
// Write the NVM from sigfox NVM base and for the given number
// of bytes. This is a byte NVM access.
// Make sure the EEPROM implementation supports byte read and
// alignment on any addresses
//
// The NVM contains only 4 bytes and they are all read at once usually
// 2 bytes for random value (LSB then MSB)
// 2 bytes for SeqId value (LSB then MSB)
//
// On Init, the library does not init the NVM, the upper layer
// Needs to setup SeqId to 0 (by the way, any value is ok) and
// the random value to a random value.
//
// Currently in the lib, the MCU_API_set_nvm is a full refresh of the
// data on every frame. The Random value is used for selecting a frequency
// and the random Value is changed on every uplink frame with a new random
// value.
//
// TODO
// As the SeqId write can impact the Flash life duration
// we will hack it by reducing the write cycle committing the SeqId
// only on a power of two. meaning Read & write are modified to
// hide the real value of SeqId stored into NVM
// -----------------------------------------------------------
MCU_API_status_t MCU_API_set_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes) {

	#ifdef ERROR_CODES
    MCU_API_status_t status = MCU_API_SUCCESS;
	#endif

    _LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_set_nvm(%d) [%02X,%02X,%02X,%02X]\r\n", nvm_data_size_bytes,nvm_data[0],nvm_data[1],nvm_data[2],nvm_data[3]));
	#if ITSDK_SFX_SX126X_NVMUPD > 1
    	// Update structure in ram
    	bcopy(nvm_data,__sx126x_sigfox_nvm,nvm_data_size_bytes);
    	// Update structure in flash
    	if ( __sx126x_sigfox_nvm_acc >=  ITSDK_SFX_SX126X_NVMUPD ) {
    		__sx126x_sigfox_nvm_acc = 0;
    		// anticipate the next seqId
    		uint16_t sId = (((uint16_t)nvm_data[3]) << 8) + nvm_data[2];
    		sId = (sId + ITSDK_SFX_SX126X_NVMUPD) % ITSDK_SIGFOX_ROLLOVER;
    		nvm_data[2] = (sId & 0xFF);
    		nvm_data[3] = (sId >> 8);
    		// write the updated data in flash
    		uint32_t offset;
    		itsdk_sigfox_getNvmOffset(&offset);
    		_eeprom_write(ITDT_EEPROM_BANK0, offset, (void *) nvm_data, nvm_data_size_bytes);
    	} else __sx126x_sigfox_nvm_acc++;
	#else
		uint32_t offset;
		itsdk_sigfox_getNvmOffset(&offset);
		_eeprom_write(ITDT_EEPROM_BANK0, offset, (void *) nvm_data, nvm_data_size_bytes);
	#endif
    RETURN();
}


#ifdef VERBOSE

  // -----------------------------------------------------------
  // Return the SIGFOX LIB VERSION
  // The PAC is stored in the ItSDK secure store and not in the
  // Sigfox library. So let this make nothing as apparently it
  // is just printing and we have a better way to get it printed
  // -----------------------------------------------------------
  MCU_API_status_t MCU_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes) {
	#ifdef ERROR_CODES
    MCU_API_status_t status = MCU_API_SUCCESS;
	#endif
    RETURN();
  }

  static const sfx_u8 SIGFOX_MCU_API_VERSION[] = "ItSdk (" SIGFOX_EP_LIB_VERSION ")";
  // -----------------------------------------------------------
  // Return the SIGFOX LIB VERSION
  // -----------------------------------------------------------
  MCU_API_status_t MCU_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
	  #ifdef ERROR_CODES
	  MCU_API_status_t status = MCU_API_SUCCESS;
	  #endif

	  (*version) = (sfx_u8*) SIGFOX_MCU_API_VERSION;
	  (*version_size_char) = (sfx_u8) sizeof(SIGFOX_MCU_API_VERSION);
	  RETURN();
  }
#endif

// ===========================================================
// Encryption & CRC Management
// -----------------------------------------------------------
// This set of API allows to use hardware CRC & Primary Number
// Generator as AES218. The Sigfox library includes some soft
// Version of this, if the hardware has some capabilities they
// can be called here.
//
// The EAS128CBC is not provided by the Sigfox library and an
// external library must be used.
// ===========================================================


// -----------------------------------------------------------
// Encrypt with Sigfox Key, sigfox key is accessible from the
// Secure store of hardcoder based on configuration. the
// itsdk_sigfox_getKEY function gets it. the result of the encryption
// is returned in the same buffer
// -----------------------------------------------------------

MCU_API_status_t MCU_API_aes_128_cbc_encrypt(MCU_API_encryption_data_t *aes_data) {
    uint8_t aes_key[16];
    _LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_aes_128_cbc_encrypt\r\n"));
	#ifdef PUBLIC_KEY_CAPABLE
    if (aes_data->key == SIGFOX_EP_KEY_PRIVATE) {
    	itsdk_sigfox_getKEY(aes_key);
    	itsdk_aes_cbc_encrypt_128B(
    			aes_data->data,			// Data to be encrypted
				aes_data->data,			// Encryption result
				16,						// Datasize
				aes_key					// Encryption key
		);
    } else if ((aes_data->key == SIGFOX_EP_KEY_PUBLIC)) {
    	itsdk_aes_cbc_encrypt_128B(
    			aes_data->data,			// Data to be encrypted
				aes_data->data,			// Encryption result
				16,						// Datasize
				(uint8_t *)SIGFOX_EP_PUBLIC_KEY	 // Encryption key
		);
    }
	#else
	itsdk_sigfox_getKEY(aes_key);
	itsdk_aes_cbc_encrypt_128B(
			aes_data->data,			// Data to be encrypted
			aes_data->data,			// Encryption result
			16,						// Datasize
			aes_key					// Encryption key
	);
#endif
#ifdef ERROR_CODES
    return MCU_API_SUCCESS;
#endif
}


#ifdef CRC_HW
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_compute_crc16(sfx_u8 *crc_data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc)
 * \brief Compute a CRC16.
 * \param[in]  	data: Input data.
 * \param[in]	data_size: Number of bytes of the input data.
 * \param[in]	polynom: CRC polynom to use.
 * \param[out] 	crc: Pointer to the computed CRC16 value.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_compute_crc16(sfx_u8 *crc_data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc) {
#warning tobedone
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	return MCU_API_SUCCESS;
#endif
}
#endif

#if (defined CRC_HW) && (defined APPLICATION_MESSAGES) && (defined BIDIRECTIONAL)
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_compute_crc8(sfx_u8 *crc_data, sfx_u8 data_size, sfx_u16 polynom, sfx_u8 *crc)
 * \brief Compute a CRC8.
 * \param[in]  	data: Input data.
 * \param[in]	data_size: Number of bytes of the input data.
 * \param[in]	polynom: CRC polynom to use.
 * \param[out] 	crc: Pointer to the computed CRC8 value.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_compute_crc8(sfx_u8 *crc_data, sfx_u8 data_size, sfx_u16 polynom, sfx_u8 *crc) {
#warning tobedone
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	return MCU_API_SUCCESS;
#endif
}
#endif

#ifdef PN_HW
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_compute_pn11(sfx_u16 min_value, sfx_u16 max_value, sfx_u16 *next_value)
 * \brief Compute a new PN11 value.
 * \param[in]  	min_value: Minimum allowed value.
 * \param[in]	max_value: Maximum allowed value.
 * \param[out] 	next_value: Pointer to the computed new PN11 value.
 * \retval		none
 *******************************************************************/
MCU_API_status_t MCU_API_compute_pn11(sfx_u16 min_value, sfx_u16 max_value, sfx_u16 *next_value) {
#warning tobedone
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	return MCU_API_SUCCESS;
#endif
}
#endif

#if (defined PN_HW) && (defined SPECTRUM_ACCESS_FH)
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_compute_pn7(sfx_u16 min_value, sfx_u16 max_value, sfx_u16 *next_value)
 * \brief Compute a new PN7 value.
 * \param[in]  	min_value: Minimum allowed value.
 * \param[in]	max_value: Maximum allowed value.
 * \param[out] 	next_value: Pointer to the computed new PN7 value.
 * \retval		none
 *******************************************************************/
MCU_API_status_t MCU_API_compute_pn7(sfx_u16 min_value, sfx_u16 max_value, sfx_u16 *next_value) {
#warning tobedone
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	return MCU_API_SUCCESS;
#endif
}
#endif


// ===========================================================
// Voltage & Temperature
// -----------------------------------------------------------
// Sigfox have a OOB frame containing the Temperature and
// Voltage for Downlink Ack and Keep Alive. The lib needs these
// informations.
// Voltage is measured on idle period and during a frame TX.
// it means we need to capture the voltage during a frame
// transmission. Voltage is in mV and temp in 1/10°C
// ===========================================================

// -----------------------------------------------------------
// Returns Voltage and MCU temperature
// -----------------------------------------------------------
uint16_t	__sx126x_voltageInTx = 0;
#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
MCU_API_status_t MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle_mv, sfx_u16 *voltage_tx_mv, sfx_s16 *temperature_tenth_degrees)
{
	#ifdef ERROR_CODES
    MCU_API_status_t status = MCU_API_SUCCESS;
	#endif
    _LOG_SFXEPLIB_DEBUG(("[SFX] MCU_API_get_voltage_temperature()\r\n"));

    (*voltage_idle_mv)=adc_getVdd();
    (*voltage_tx_mv)=(__sx126x_voltageInTx!=0)?__sx126x_voltageInTx:adc_getVdd();
    (*temperature_tenth_degrees)=adc_getTemperature()/10;
    RETURN();
}
void sx126x_updateTxVoltate() {
	__sx126x_voltageInTx=adc_getVdd();
}
#endif




#if ITSDK_RADIO_CERTIF == __ENABLE

MCU_API_status_t MCU_API_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm)
{
	#ifdef ERROR_CODES
     MCU_API_status_t status = MCU_API_SUCCESS;
	#endif
    _itsdk_console_printf("data:%02X%02X%02X%02X%02X%02X%02X%02X rssi:%d",
		dl_payload[0],dl_payload[1],dl_payload[2],dl_payload[3],
		dl_payload[4],dl_payload[5],dl_payload[6],dl_payload[7],
		rssi_dbm);
    RETURN();
}
#elif defined CERTIFICATION
MCU_API_status_t MCU_API_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm) {
	#ifdef ERROR_CODES
     MCU_API_status_t status = MCU_API_SUCCESS;
	#endif
    RETURN();
}
#endif



#endif // ITSDK_SIGFOX_LIB
