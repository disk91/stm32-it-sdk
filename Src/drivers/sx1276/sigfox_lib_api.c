/* ==========================================================
 * st_lib_api.c - Sigfox mcu api
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 1 may 2019
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

#include <it_sdk/config.h>
#if ( ITSDK_WITH_SIGFOX_LIB == __ENABLE ) && (ITSDK_SIGFOX_LIB == __SIGFOX_SX1276)
#include <string.h>
#include <stdio.h>
#include <drivers/sigfox/mcu_api.h>
#include <drivers/sigfox/rf_api.h>
#include <drivers/sx1276/sigfox_sx1276.h>
#include <drivers/sx1276/sgfx_sx1276_driver.h>
#include <drivers/sx1276/sigfox_lowlevel.h>
#include <drivers/sigfox/sigfox_api.h>
#include <drivers/sigfox/se_nvm.h>
#include <it_sdk/sigfox/sigfox.h>
#include <it_sdk/eeprom/eeprom.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/time/timer.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/lowpower/lowpower.h>

// =============================================================================================
// MCU API
// =============================================================================================

// The new version of the compiler was protecting memory access against static elements
// Making a hardfault in the sigfox api init function when it tried to write into this zone.
// Non static declaration solve this issue
// Next issue was on alignement, if not aligned on 4 it is possible that the first address is out of the
// authorized area.
sfx_u8 __sigfox_mem[ITSDK_SIGFOX_MEM_SIZE] __attribute((aligned(4)));

/**
 * Static memory allocation
 */
sfx_u8 MCU_API_malloc(sfx_u16 size, sfx_u8 **returned_pointer)
{
  LOG_DEBUG_SFXSX1276(("Sigfox lib mem req: %dB \r\n",size));
  if(size>ITSDK_SIGFOX_MEM_SIZE) {
	  LOG_ERROR_SFXSX1276(("Requesting more memory than maximum allowable\r\n"));
	  return MCU_ERR_API_MALLOC;
  } else {
    (*returned_pointer)=__sigfox_mem;
  }
  return SFX_ERR_NONE;
}

/**
 * Fake memory deallocation
 */
sfx_u8 MCU_API_free(sfx_u8 *ptr)
{
	LOG_DEBUG_SFXSX1276((">> MCU_API_free\r\n"));
    return SFX_ERR_NONE;
}

/*
* @brief  Gets version.
* @param  version : pointer to the buffer
* @param  size    : Size
* @retval sfx_u8  : status
*/
sfx_u8 MCU_API_get_version(sfx_u8 **version, sfx_u8 *size)
{
	LOG_DEBUG_SFXSX1276((">> MCU_API_get_version\r\n"));
	static sfx_u8 mcu_api_version[]= SIGFOX_MCU_API_VER;
	(*size) = sizeof(mcu_api_version);
	(*version) = (sfx_u8*)mcu_api_version;
	return SFX_ERR_NONE;
}


/**
 * Get voltage & temperature
 */
uint16_t	__sx1276_voltageInTx = 0;
uint8_t		__sx1276_voltageInTxPending = 0;
sfx_u8 MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle, sfx_u16 *voltage_tx, sfx_s16 *temperature)
{

	LOG_DEBUG_SFXSX1276((">> MCU_API_get_voltage_temperature\r\n"));
    // get the idle voltage of the complete device
    // get the temperature of the device
    // if those values are not available : set it to 0x0000
    // return the voltage_idle in 1/1000 volt on 16bits and 1/10 degrees for the temperature */
    (*voltage_idle)=adc_getVdd();
    (*voltage_tx)=(__sx1276_voltageInTx!=0)?__sx1276_voltageInTx:adc_getVdd();
    (*temperature)=adc_getTemperature()/10;

    return SFX_ERR_NONE;

}

/**
 * Wait for the given delay_type..
 */
#warning "We could optimize long stop with going to sleep..."
sfx_u8 MCU_API_delay(sfx_delay_t delay_type)
{
	LOG_DEBUG_SFXSX1276((">> MCU_API_delay(%d)\r\n",delay_type));
    sfx_error_t err = SFX_ERR_NONE;
    uint8_t rcz;
    itsdk_sigfox_getCurrentRcz(&rcz);
    //SCH_WaitEvt( DELAY_EVT );
    switch(delay_type)
    {
    case SFX_DLY_INTER_FRAME_TRX :
        /* Delay  is 500ms  in FCC and ETSI
         * In ARIB : minimum delay is 50 ms */
        if( rcz == SIGFOX_RCZ3C ) {
        	itsdk_delayMs(ITSDK_SIGFOX_IF_TXRX_RCZ3);
        } else {
        	// Measure (frame to frame) is 721ms for 500ms requested
        	// due to code around assuming with 50ms TCXO wakeup
			#if ITSDK_SIGFOX_IF_TXRX_RCZ1 < ITSDK_SX1276_SFXWAKEUP_TIME
			#error "ITSDK_SIGFOX_IF_TXRX_RCZ1 can't be lower than ITSDK_SX1276_SFXWAKEUP_TIME"
			#endif
        	itsdk_delayMs(ITSDK_SIGFOX_IF_TXRX_RCZ1 - ITSDK_SX1276_SFXWAKEUP_TIME);	// spec is 500 - 525ms
        }
        break;

    case SFX_DLY_INTER_FRAME_TX :
        /* Start delay 0 seconds to 2 seconds in FCC and ETSI*/
        /* In ARIB : minimum delay is 50 ms */
        if( rcz == SIGFOX_RCZ3C ) {
        	itsdk_delayMs(50);
        } else {
			#if ITSDK_SIGFOX_IF_TX_RCZ1 < ITSDK_SX1276_SFXWAKEUP_TIME
			#error "ITSDK_SIGFOX_IF_TX_RCZ1 can't be lower than ITSDK_SX1276_SFXWAKEUP_TIME"
			#endif
        	// was 1s but many different devices like sensit are 100ms sounds more efficient
        	// but 100 ms do not work really good
        	itsdk_delayMs(ITSDK_SIGFOX_IF_TX_RCZ1 - ITSDK_SX1276_SFXWAKEUP_TIME);
        }
        break;

    case SFX_DLY_OOB_ACK :
        /* Start delay between 1.4 seconds to 4 seconds in FCC and ETSI */
    	itsdk_delayMs(1400);
       /*comment from sigfox iso 1400 was measured 1300, spec={1,4-4s}so added 200*/
        break;

    case SFX_DLY_CS_SLEEP :
    	itsdk_delayMs(500);
        break;

    default :
        err = MCU_ERR_API_DLY;
        break;
    }
    return err;
}



/**
 * Read Data to NVM (eeprom)
 */
sfx_u8 MCU_API_get_nv_mem(sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE])
{
	LOG_DEBUG_SFXSX1276((">> MCU_API_get_nv_mem\r\n"));

	uint32_t offset;
	itsdk_sigfox_getNvmOffset(&offset);

	uint8_t tab[SFX_NVMEM_BLOCK_SIZE+4] = {0};
	uint8_t sz = itdt_align_32b(SFX_NVMEM_BLOCK_SIZE);
	_eeprom_read(ITDT_EEPROM_BANK0, offset, (void *) tab, sz);
	bcopy(tab,read_data,SFX_NVMEM_BLOCK_SIZE);

	//log_info_array("MCU_NVM",read_data,SFX_NVMEM_BLOCK_SIZE);

	return SFX_ERR_NONE;
}

/**
 * Write Data to NVM (eeprom)
 */
sfx_u8 MCU_API_set_nv_mem(sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE])
{
	LOG_DEBUG_SFXSX1276((">> MCU_API_set_nv_mem\r\n"));
	//log_info_array("MCU_NVM",data_to_write,SFX_NVMEM_BLOCK_SIZE);

	uint32_t offset;
	itsdk_sigfox_getNvmOffset(&offset);

	uint8_t tab[SFX_NVMEM_BLOCK_SIZE+4] = {0};
	bcopy(data_to_write,tab,SFX_NVMEM_BLOCK_SIZE);
	uint8_t sz = itdt_align_32b(SFX_NVMEM_BLOCK_SIZE);
	_eeprom_write(ITDT_EEPROM_BANK0, offset, (void *) tab, sz);

	return SFX_ERR_NONE;
}


/**
 * Report test ?? not clear when is it called
 */
sfx_u8 MCU_API_report_test_result(sfx_bool status, sfx_s16 rssi)
{
	LOG_DEBUG_SFXSX1276((">> MCU_API_report_test_result\r\n"));

    if( status == SFX_TRUE) {
    // we received a frame for the device
    } else {
	 // we did not received a frame
    }
    return SFX_ERR_NONE;
}


/** ----------------------------------------------------------------------------
 * Timers
 */
void OnTimerTimeoutCsEvt( uint32_t v ) {
	LOG_DEBUG_SFXSX1276((">> OnTimerTimeoutCsEvt\r\n"));
	sx1276_sigfox_state.rxCarrierSenseFlag = STLL_SET;
}

sfx_u8 MCU_API_timer_start_carrier_sense(sfx_u16 time_duration_in_ms)
{
	LOG_DEBUG_SFXSX1276((">> MCU_API_timer_start_carrier_sense %d \r\n",time_duration_in_ms));
	if (   itsdk_stimer_register(
			time_duration_in_ms,
			OnTimerTimeoutCsEvt,
			0,
			TIMER_ACCEPT_LOWPOWER
		) != TIMER_INIT_SUCCESS ) {
			ITSDK_ERROR_REPORT(ITSDK_ERROR_SIGFOX_TIMER_STARTERROR,0);
		}
	return SFX_ERR_NONE;
}

sfx_u8 MCU_API_timer_stop_carrier_sense(void)
{
	LOG_DEBUG_SFXSX1276((">> MCU_API_timer_stop_carrier_sense \r\n"));
	itsdk_stimer_stop(
				OnTimerTimeoutCsEvt,
				0
			);
    return SFX_ERR_NONE;
}


void OnTimerTimeoutEvt( uint32_t v ) {
	LOG_DEBUG_SFXSX1276((">> OnTimerTimeoutEvt\r\n"));
	sx1276_sigfox_state.timerEvent = SIGFOX_EVENT_SET;
	//SCH_SetEvt( TIMOUT_EVT );
}

sfx_u8 MCU_API_timer_start(sfx_u32 time_duration_in_s) {

	LOG_DEBUG_SFXSX1276((">> MCU_API_timer_start(%d)\r\n",time_duration_in_s));
	if (   itsdk_stimer_register(
				(time_duration_in_s)*1000-500,
				OnTimerTimeoutEvt,
				0,
				TIMER_ACCEPT_LOWPOWER
			) != TIMER_INIT_SUCCESS ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_SIGFOX_TIMER_STARTERROR,1);
	} else {
	   sx1276_sigfox_state.timerEvent = SIGFOX_EVENT_CLEAR;
	}
	return SFX_ERR_NONE;
}

sfx_u8 MCU_API_timer_stop(void)
{
	LOG_DEBUG_SFXSX1276((">> MCU_API_timer_stop\r\n"));
	itsdk_stimer_stop(
			OnTimerTimeoutEvt,
			0
	);
	sx1276_sigfox_state.timerEvent = SIGFOX_EVENT_SET;
	return SFX_ERR_NONE;

}

/**
 * Rq : this function wait for 2x10s in RCZ2 communication, assuming to ensure the 20s duty cycle between transmissions
 * switch to low power mode with rtc wakeup only.
 */
sfx_u8 MCU_API_timer_wait_for_end(void)
{
   LOG_DEBUG_SFXSX1276((">> MCU_API_timer_wait_for_end\r\n"));
   while (sx1276_sigfox_state.timerEvent == SIGFOX_EVENT_CLEAR) {
	   lowPower_delayMs(1000);
	   if ( sx1276_sigfox_idle() == SX1276_SIGFOX_ERR_BREAK ) break;
	   #if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
 	      wdg_refresh();
	   #endif
   }
   return SFX_ERR_NONE;
}


// =============================================================================================
// RF API
// =============================================================================================

sfx_u8 RF_API_init(sfx_rf_mode_t rf_mode)
{
   LOG_DEBUG_SFXSX1276((">> RF_API_init\r\n"));

   // ------------------------------------------------------
   // PSEUDO code
   // ------------------------------------------------------
   // Put here all RF initialization concerning the RC IC you are using.
   // this function is dependant of SPI driver or equivalent.
   switch (rf_mode)
   {
    case SFX_RF_MODE_TX :
  	  LOG_DEBUG_SFXSX1276(("RF_API_init in TX\r\n"));
      SGFX_SX1276_tx_config( );
      // set the RF IC in TX mode : this could be DBPSK100 or 600:  this distinction will be done during the RF_API_send
      // you can add some code to switch on TCXO or warm up quartz for stabilization : in case of frequency drift issue
    break;
    case SFX_RF_MODE_RX :
      LOG_DEBUG_SFXSX1276(("RF_API_init in RX\r\n"));
      SGFX_SX1276_rx_config( );
      // set the RF IC in RX mode : GFSK 600bps is the only mode now
      // Enable interrupts on RX fifo
      // RF IC must configure the SYNCHRO BIT = 0xAAAAAAAAAA and synchro frame = 0xB227
    break;
    case SFX_RF_MODE_CS200K_RX :
      LOG_DEBUG_SFXSX1276(("RF_API_init in CS200\r\n"));
      SGFX_SX1276_rx_config( );
      SGFX_SX1276_rx_setbw(CS_BW_200KHZ);
      // configure the RF IC into sensing 200KHz bandwidth to be able to read out RSSI level
      // RSSI level will outputed during the RF_API_wait_for_clear_channel function
    break;
    case SFX_RF_MODE_CS300K_RX :
      LOG_DEBUG_SFXSX1276(("RF_API_init in CS300\r\n"));
      SGFX_SX1276_rx_config( );
      SGFX_SX1276_rx_setbw(CS_BW_300KHZ);
      // configure the RF IC into sensing 300KHz bandwidth to be able to read out RSSI level
      // This is poosible to make this carrier sense in 2 * 200Kz if you respect the regulation time for listening
      // RSSI level will outputed during the RF_API_wait_for_clear_channel function.
    break;

    default :
    break;
  }
  return SFX_ERR_NONE;
};

/**
 * Stop
 */
sfx_u8 RF_API_stop(void)
{
    LOG_DEBUG_SFXSX1276(("RF_API_stop\r\n"));
    SGFX_SX1276_stop();
    return SFX_ERR_NONE;
}

/**
 * Send
 */
sfx_u8 RF_API_send(sfx_u8 *stream, sfx_modulation_type_t type, sfx_u8 size)
{
  LOG_DEBUG_SFXSX1276(("RF_API_send (%d)(%d)\r\n",type,size));
  sfx_u8 status =SFX_ERR_NONE;
  mod_error_t err;
  switch (type)
  {
    case SFX_DBPSK_100BPS :
      if ( (err = SGFX_SX1276_tx( (uint8_t *)stream, (uint8_t) size, 100 )) != MOD_SUCCESS) {
    	  LOG_DEBUG_SFXSX1276(("100 - Send Error (%d)\r\n",err));
          status = RF_ERR_API_SEND;
      }
      break;
    case SFX_DBPSK_600BPS :
      if ( (err = SGFX_SX1276_tx( (uint8_t *)stream, (uint8_t) size, 600 )) != MOD_SUCCESS) {
    	  LOG_DEBUG_SFXSX1276(("600 - Send Error  (%d)\r\n",err));
          status = RF_ERR_API_SEND;
      }
      break;

    default :
      status = RF_ERR_API_SEND;
      break;
  }
  return status;
}


/**
 * ContinousTransmit
 */
sfx_u8 RF_API_start_continuous_transmission (sfx_modulation_type_t type)
{
  LOG_DEBUG_SFXSX1276(("RF_API_start_continuous_transmission\r\n"));

  sfx_u8 status = SFX_ERR_NONE;
   // ------------------------------------------------------
    // PSEUDO code
    // ------------------------------------------------------
  switch (type)
  {
    case SFX_NO_MODULATION :
      if (SGFX_SX1276_start_txtest_cw()!= MOD_SUCCESS) {
       status = RF_ERR_API_START_CONTINUOUS_TRANSMISSION;
      }
      // Configure the RF IC into pure carrier CW : no modulation
      // This mode is available on many RF ICs for type approval tests or manufacturing tests.
      // The frequency is chosen in the RF_API_change_frequency by the sigfox lib
      // i.e. : SPI_DRV_write(CW)
    break;
    case SFX_DBPSK_100BPS :
      if (SGFX_SX1276_start_txtest_prbs9( 100 )!= MOD_SUCCESS) {
       status = RF_ERR_API_START_CONTINUOUS_TRANSMISSION;
      }
      // Make an infinite DBPSK 100bps modulation on the RF IC at the frequency given by the RF_API_change_frequency()
      break;
    case SFX_DBPSK_600BPS :
      // Make an infinite DBPSK 600bps modulation on the RF IC at the frequency given by the RF_API_change_frequency()
      if (SGFX_SX1276_start_txtest_prbs9( 600 )!= MOD_SUCCESS) {
       status = RF_ERR_API_START_CONTINUOUS_TRANSMISSION;
      }
      break;
    default :
      status = RF_ERR_API_SEND;
      break;
  }
  return status;
}


/**
 * Stop continuous transmission
 */
sfx_u8 RF_API_stop_continuous_transmission (void)
{
   LOG_DEBUG_SFXSX1276(("RF_API_start_continuous_transmission\r\n"));
   // Stop the RC IC : you can switch off this part and use RF IC for other modulation / protocole : SFX LIB does not use it after this call
   // Stop also the TCXO if you have one or external PA.
   if (SGFX_SX1276_stop_txtest() != MOD_SUCCESS) {
     return RF_ERR_API_STOP_CONTINUOUS_TRANSMISSION;
   }
   return SFX_ERR_NONE;
}

sfx_u8 RF_API_change_frequency(sfx_u32 frequency)
{
  LOG_DEBUG_SFXSX1276(("RF_API_change_frequency(%d)\r\n",frequency));
  STLL_Radio_SetFreq(frequency);
  return SFX_ERR_NONE;
}

/**
 * wait frame
 */
sfx_u8 RF_API_wait_frame(sfx_u8 *frame, sfx_s16 *rssi, sfx_rx_state_enum_t * state)
{
  LOG_DEBUG_SFXSX1276(("RF_API_wait_frame\r\n"));
  sfx_error_t status;

  SGFX_SX1276_rx_start();
  // Wait that flag EOFTX_EVT is set by STLL_SetEndOfTxFrame
  //STLL_WaitEndOfTxFrame();
  // SCH_WaitEvt( EOFTX_EVT );

  if( STLL_WaitEndOfRxFrame( ) == STLL_SET ){
    SGFX_SX1276_rx_stop(frame);
    *rssi = STLL_SGFX_SX1276_GetSyncRssi();
    sx1276_sigfox_state.meas_rssi_dbm = *rssi;
    status = SFX_ERR_NONE;
    *state = DL_PASSED;
  } else {
    *rssi = (sfx_s8) 0;
    *state = DL_TIMEOUT;
    status = SFX_ERR_NONE;
  }
  return status;
}


/**
 * wait for clear channel
 */
sfx_u8 RF_API_wait_for_clear_channel(sfx_u8 cs_min, sfx_s8 cs_threshold, sfx_rx_state_enum_t * state )
{
  LOG_DEBUG_SFXSX1276(("RF_API_wait_for_clear_channel\r\n"));

  sfx_s16 rssiVec[SIGFOX_MAX_CS_RSSI_AVG];
  sfx_s16 rssi=0;
  int cur_idx=0;

  sfx_rx_state_enum_t cs_state  = DL_TIMEOUT;
  /*default in case below never true*/
  /*Init so that it waits at least 5*/
  for (int i=0; i<SIGFOX_MAX_CS_RSSI_AVG; i++) {
    rssiVec[i]=INT16_MAX;
  }


  if (cs_min >= SIGFOX_MAX_CS_RSSI_AVG) {
    return RF_ERR_API_WAIT_CLEAR_CHANNEL;
  }

  /*init the cs status to Reset*/
  STLL_RxCarrierSenseInitStatus();
   /*starts the receiver to sense channel*/
  LOG_DEBUG_SFXSX1276(("> CS Start\r\n"));
  SGFX_SX1276_rx_start();

  while (STLL_RxCarrierSenseGetStatus( ) == STLL_RESET ) {
	itsdk_delayMs(1);
    // integrate the rssi during cs_min miliseconds
    rssiVec[cur_idx] = STLL_RxCarrierSenseGetRssi();

    cur_idx++;
    if (cur_idx==cs_min) {
      cur_idx=0;
    }

    //Av in dB.. Should be done in linear to be exact.
    rssi = 0;
    for(int i=0; i<cs_min; i++) {
      rssi+=rssiVec[i];
    }
    rssi= rssi / (cs_min);

    if ( rssi < cs_threshold ) {
       cs_state= DL_PASSED;
       break;
    }

  }

  if (cs_state== DL_PASSED) {
    LOG_DEBUG_SFXSX1276(("> LBT Channel Free\r\n"));
  } else {
    LOG_DEBUG_SFXSX1276(("> LBT Channel Busy\r\n"));
  }
  *state= cs_state;
  return SFX_ERR_NONE;
}

/**
 * Get Version from the static ST library
 */
#define SIZE_OF_MODULATION_LIB_VERSION  30 /* MODULATION_LIB_VERSION defined in ST_SGFX_SX1276_xxx.lib */
static sfx_u8 rf_api_version[SIZE_OF_MODULATION_LIB_VERSION];
sfx_u8 RF_API_get_version(sfx_u8 **version, sfx_u8 *size)
{
  sfx_u8 ret = SFX_ERR_API_GET_VERSION;
  int len=  sprintf( (char*) rf_api_version, "%s", SGFX_SX1276_get_version() );

  *version = (sfx_u8*)rf_api_version;
  if(size != SFX_NULL)
  {
    *size = len;
    ret = SFX_ERR_NONE;
  }

  return ret;
}

// =============================================================================================
// Secure Element
// =============================================================================================


/**
 * Read Data from NVM (eeprom)
 */
sfx_u8 SE_NVM_get(sfx_u8 read_data[SFX_SE_NVMEM_BLOCK_SIZE])
{
	LOG_DEBUG_SFXSX1276((">> SE_NVM_get\r\n"));

	uint32_t offset;
	itsdk_sigfox_getSeNvmOffset(&offset);

	uint8_t tab[SFX_SE_NVMEM_BLOCK_SIZE+4] = {0};
	uint8_t sz = itdt_align_32b(SFX_SE_NVMEM_BLOCK_SIZE);
	_eeprom_read(ITDT_EEPROM_BANK0, offset, (void *) tab, sz);
	bcopy(tab,read_data,SFX_SE_NVMEM_BLOCK_SIZE);

	//log_info_array("SE_NVM",read_data,SFX_SE_NVMEM_BLOCK_SIZE);

    return SFX_ERR_NONE;
}

/**
 * Write Data to NVM (eeprom)
 * We need to be aligned on 32b size to access the EEPROM...
 */
sfx_u8 SE_NVM_set(sfx_u8 data_to_write[SFX_SE_NVMEM_BLOCK_SIZE])
{
	LOG_DEBUG_SFXSX1276((">> SE_NVM_set\r\n"));

	//log_info_array("SE_NVM",data_to_write,SFX_SE_NVMEM_BLOCK_SIZE);

	uint32_t offset;
	itsdk_sigfox_getSeNvmOffset(&offset);

	uint8_t tab[SFX_SE_NVMEM_BLOCK_SIZE+4] = {0};
	bcopy(data_to_write,tab,SFX_SE_NVMEM_BLOCK_SIZE);
	uint8_t sz = itdt_align_32b(SFX_SE_NVMEM_BLOCK_SIZE);
	_eeprom_write(ITDT_EEPROM_BANK0, offset, (void *) tab, sz);
    return SFX_ERR_NONE;
}

/**
 * Get the Type of Key (public / private)
 */
sfx_key_type_t SE_NVM_get_key_type( void ) {
	LOG_DEBUG_SFXSX1276((">> SE_NVM_get_key_type()\r\n"));

	uint8_t v = 0;
#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
	v= itsdk_config.sdk.sigfox.sgfxKey;
#else
	v= ITSDK_SIGFOX_KEY_TYPE;
#endif
	switch (v) {
		default:
		case SIGFOX_KEY_PRIVATE:
			return CREDENTIALS_KEY_PRIVATE;
			break;
		case SIGFOX_KEY_PUBLIC:
			return CREDENTIALS_KEY_PUBLIC;
			break;
	}
}

/**
 * Set the type of key - not really saved in the nvm as this operation will be
 * made in a different way, here we just update the memory
 */
void  SE_NVM_set_key_type( sfx_key_type_t keyType ) {
	LOG_DEBUG_SFXSX1276((">> SE_NVM_set_key_type()\r\n"));
#if ITSDK_CONFIGURATION_MODE != __CONFIG_STATIC
	switch (keyType) {
	default:
	case CREDENTIALS_KEY_PRIVATE:
		itsdk_config.sdk.sigfox.sgfxKey = SIGFOX_KEY_PRIVATE;
		break;
	case CREDENTIALS_KEY_PUBLIC:
		itsdk_config.sdk.sigfox.sgfxKey = SIGFOX_KEY_PUBLIC;
		break;
	}
#else
	return ;
#endif
}



#endif

