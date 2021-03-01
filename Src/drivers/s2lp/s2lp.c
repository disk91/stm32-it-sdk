/* ==========================================================
 * s2lp.c - S2LP (STm SubGhz transceiver) driver for sigfox
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
 * Some peaces of that code directly comes from ST Libraries
 * and identified with << COPYRIGHT(c) 2018 STMicroelectronics >>
 *
 * ==========================================================
 */

#include <it_sdk/config.h>
#if ITSDK_WITH_SIGFOX_LIB > 0 && ITSDK_SIGFOX_LIB == __SIGFOX_S2LP

#include <it_sdk/itsdk.h>
#include <it_sdk/sigfox/sigfox.h>
#include <drivers/s2lp/s2lp_spi.h>
#include <drivers/s2lp/s2lp.h>
#include <it_sdk/logger/logger.h>
#include <drivers/s2lp/st_rf_api.h>
#include <drivers/s2lp/st_lib_api.h>
#include <drivers/s2lp/sigfox_retriever.h>
#include <drivers/s2lp/sigfox_helper.h>

#if ITSDK_SIGFOX_NVM_SOURCE	== __SFX_NVM_M95640
	#include <drivers/eeprom/m95640/m95640.h>
#endif

s2lp_drivers_config_t s2lp_driver_config;

void s2lp_shutdown() {
	gpio_set(ITSDK_S2LP_SDN_BANK,ITSDK_S2LP_SDN_PIN);
}

void s2lp_wakeup() {
	gpio_reset(ITSDK_S2LP_SDN_BANK,ITSDK_S2LP_SDN_PIN);
	itsdk_delayMs(1);
}

Ò
itsdk_sigfox_init_t s2lp_sigfox_init() {
	uint8_t tmp;

 #if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640
	eeprom_m95640_hwInit();
 #endif
	gpio_configure_ext(ITSDK_S2LP_SDN_BANK,ITSDK_S2LP_SDN_PIN,GPIO_OUTPUT_PP,ITSDK_GPIO_SPEED_HIGH,ITSDK_GPIO_ALT_NONE);
	s2lp_spi_setCsHigh();		// disable CS
	s2lp_shutdown();			// The S2LP have a bug before V3 : when not shutdown it force value on SPI
								// and jam the other devices even when CS is not activ
	itsdk_delayMs(10);
 #if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640
	eeprom_m95640_init(&ITSDK_DRIVERS_M95640_SPI);
 #endif

	S2LP_SPI_StatusBytes status;
	s2lp_wakeup();

	#if ITSDK_S2LP_TARGET == __S2LP_HT32SX
	// Front End Module initialization
	// This is part of Ht32SX driver
	uint8_t tmp[]={
		(uint8_t)S2LP_GPIO_DIG_OUT_TX_RX_MODE | (uint8_t)S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
		(uint8_t)S2LP_GPIO_DIG_OUT_RX_STATE   | (uint8_t)S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
		(uint8_t)S2LP_GPIO_DIG_OUT_TX_STATE   | (uint8_t)S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP
	};
	s2lp_spi_writeRegisters(ITSDK_S2LP_SPI, S2LP_REG_GPIO0_CONF, sizeof(tmp), tmp);
	#endif

	do {
      // Delay for state transition
      for(volatile uint8_t i=0; i!=0xFF; i++);

      // Reads the MC_STATUS register
      status = s2lp_spi_readRegisters(&ITSDK_S2LP_SPI,S2LP_REG_MC_STATE0, 1, &tmp);
	} while(status.MC_STATE!=MC_STATE_READY);
	// Return 0 when ready

	// Get the version
    s2lp_spi_readRegisters(&ITSDK_S2LP_SPI,S2LP_REG_DEVICE_INFO0, 1, &tmp);
    log_debug("S2LP version 0x%0X\r\n",tmp);
    if(tmp==S2LP_VERSION_2_0 || tmp==S2LP_VERSION_2_1) {
    	// Sounds like a bug on the SPI ...
    	// preventing conflict when accessing the eeprom or other I2C
    	s2lp_shutdown();
    }
	s2lp_loadConfiguration();

	ST_RF_API_set_freq_offset(__S2LP__ITSDK_S2LP_CNF_FREQ);
	#if ITSDK_S2LP_TARGET == __S2LP_HT32SX
	 ST_RF_API_set_xtal_freq(__S2LP__ITSDK_S2LP_CNF_FREQ);
	 ST_RF_API_set_lbt_thr_offset(__S2LP__ITSDK_SIGFOX_LBTOFFSET);
	#endif

	ST_RF_API_set_tcxo(__S2LP__ITSDK_S2LP_CNF_TCX0);
	#if ITSDK_S2LP_CNF_RANGE == __SIGFOX_S2LP_PA_SKYWORKS_868
	  ST_RF_API_set_pa(1);
	  ST_RF_API_gpio_tx_rx_pin(0);
	  ST_RF_API_gpio_rx_pin(1);
	  ST_RF_API_gpio_tx_pin(2);
	  ST_RF_API_set_rssi_offset(ITSDK_SIGFOX_RSSICAL-16);
	#elif ITSDK_S2LP_CNF_RANGE == __SIGFOX_S2LP_PA_NONE
	  ST_RF_API_set_pa(0);
	  ST_RF_API_set_rssi_offset(ITSDK_SIGFOX_RSSICAL);
	#endif


	 //  !!! ICI !!
	  switch ( conf->rcz ) {
	  	case 1:
	  		SIGFOX_API_open(&(sfx_rc_t)RC1);
	  		break;
	  	case 2:
	  		SIGFOX_API_open(&(sfx_rc_t)RC2);
	  		// In FCC we can choose the macro channel to use by a 86 bits bitmask
	  	    //  In this case we use the first 9 macro channels
	  		sfx_u32 config_words1[3]={1,0,0};
	  		SIGFOX_API_set_std_config(config_words1,1);
	  		log_error("RCZ2 implementation is actually not working");
	  		ITSDK_ERROR_REPORT(ITSDK_ERROR_SIGFOX_RCZ_NOTSUPPORTED,(uint16_t)conf->rcz);
	  		break;
	  	case 3:
	  		SIGFOX_API_open(&(sfx_rc_t)RC3C);
	  		sfx_u32 config_words2[3]=RC3C_CONFIG;
	  		SIGFOX_API_set_std_config(config_words2,0);
	  		break;
	  	case 4:
	  		SIGFOX_API_open(&(sfx_rc_t)RC4);
	  		sfx_u32 config_words3[3]={0,0x40000000,0};
	  		SIGFOX_API_set_std_config(config_words3,1);
	  		log_error("RCZ4 implementation is actually not working");
	  		ITSDK_ERROR_REPORT(ITSDK_ERROR_SIGFOX_RCZ_NOTSUPPORTED,(uint16_t)conf->rcz);
	  		break;
	  	case 5:
	  		log_error("RCZ5 implementation is actually supported");
	  		ITSDK_ERROR_REPORT(ITSDK_ERROR_SIGFOX_RCZ_NOTSUPPORTED,(uint16_t)conf->rcz);
	  		break;

	  	}
	return SIGFOX_INIT_SUCESS;
}



/**
 * Load the configuration according to the source setting in the configuration file
 * ----------------------------------------------------------------------------------
 * enc_utils_retrieve_data
 *   use 3 zone in the nvram
 *      - a 32B block starting at 0x200 with following content
 *        [ DEVICEID LSByte first - 4B ] [ PAC MSByte first - 8B ] [ ? 4b ] [ RCZ 4b ] [ Unknown 19B ]
 *      - a 8B block starting at 0x6 with following content
 *        [ ?? - 4B ]  [ ?? - 4B ]
 *      - a 4B block starting at 0x1F0 - this is a checksum for two previous blocs ( checksum of the uint32_t values )
 *  The function creates an internal structure containing the ID and PAC but it also retrieve the device key
 *  The function copy the ID, PAC and RCZ values into the given pointers.
 *  The function verify the checksum validity, the device ID compliance (not 0x00...0 | 0xFF...F)
 *
 */
void s2lp_loadConfiguration() {

	#if ITSDK_SIGFOX_NVM_SOURCE	== __SFX_NVM_M95640
		s2lp_eprom_config_t eeprom_cnf;
		eeprom_m95640_read(&ITSDK_DRIVERS_M95640_SPI,0x0000, 32, (uint8_t *)&eeprom_cnf);

		s2lp_eprom_offset_t eeprom_offset;
		eeprom_m95640_read(&ITSDK_DRIVERS_M95640_SPI,0x0021, 4, (uint8_t *)&eeprom_offset);

		// S2lp_hw config
		switch(eeprom_cnf->xtalFreq) {
		   case 0: s2lp_driver_config.tcxoFreq = 24000000;
		     break;
		   case 1: s2lp_driver_config.tcxoFreq = 25000000;
		     break;
		   case 2: s2lp_driver_config.tcxoFreq = 26000000;
		     break;
		   case 3: s2lp_driver_config.tcxoFreq = 48000000;
		     break;
		   case 4: s2lp_driver_config.tcxoFreq = 50000000;
		     break;
		   case 5: s2lp_driver_config.tcxoFreq = 52000000;
		     break;
		   case 0xff:
		     // XTAL frequency is custom
		     for(uint8_t i=0;i<4;i++){
		       ((uint8_t*)&s2lp_driver_config.tcxoFreq)[i]=((uint8_t*)&eeprom_cnf->customFreq)[3-i];
		     }
		     break;
		   default:
			   log_warn("S2LP - Config - Freq not recognized\r\n");
			   s2lp_driver_config.tcxoFreq = 50000000;
		     break;
		}

		// cnf->tcxo = eeprom_cnf->tcxo; // now static in the headers
		// cnf->band = eeprom_cnf->band; // unclear what it is

		// cnf->range = S2LP_RANGE_EXT_NONE; // static in conf
		// if (eeprom_cnf->range == 2 ) cnf->range = S2LP_RANGE_SKYWORKS_868; // static in conf

		if(*(uint32_t*)eeprom_offset->offset != 0xffffffff) {
		   for(uint8_t i=0;i<4;i++) {
		     ((uint8_t*)&s2lp_driver_config.freqOffset)[i]=eeprom_offset->offset[3-i];
		   }
		   if(s2lp_driver_config.freqOffset < -100000 || s2lp_driver_config.freqOffset > 100000) {
			   s2lp_driver_config.freqOffset=0;
		   }
		} else {
		  s2lp_driver_config.freqOffset=0;
		}

		// Not clear why => when passing &(s2lpConf->id) it crash...
		// retrieve_data also extract the SigfoxID key and store it in RAM for internal use.
		enc_utils_retrieve_data(&s2lp_driver_config.deviceId, s2lp_driver_config.initialPac, &itsdk_state.sigfox.rcz);

		// Search for the private key in the memory to fill the structure
		s2lp_sigfox_retreive_key(s2lp_driver_config.deviceId, s2lp_driver_config.initialPac, s2lp_driver_config.key);
		s2lp_sigfox_cifferKey();

	#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
		itsdk_state.sigfox.rcz = itsdk_config.sdk.sigfox.rcz;
	#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_HEADERS
		int i;
		uint8_t pac[8] 		= ITSDK_SIGFOX_PAC;
		uint8_t key[16] 	= IDTSK_SIGFOX_KEY;
		uint8_t aux[16] 	= IDTSK_SIGFOX_AUX;
		s2lp_driver_config.deviceId		= ITSDK_SIGFOX_ID;
		itsdk_sigfox_getRczFromRegion(ITSDK_DEFAULT_REGION,&itsdk_state.sigfox.rcz);
		for ( i =  0 ; i < 8 ; i++ ) itsdk_state.sigfox.initialPac[i] = pac[i];
		for ( i =  0 ; i < 16 ; i++ ) {
			s2lp_driver_config.key[i] = key[i];
		}
		sigfox_cifferKey();
		#warning "__SFX_NVM_HEADERS Not to be used in production"
	#endif

	s2lp_driver_config.lastReadRssi = S2LP_UNKNOWN_RSSI;
	s2lp_driver_config.lastReceptionRssi = S2LP_UNKNOWN_RSSI;

}


#endif // ITSDK_SIGFOX_LIB test

