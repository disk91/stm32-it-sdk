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

void s2lp_shutdown() {
	gpio_set(ITSDK_S2LP_SDN_BANK,ITSDK_S2LP_SDN_PIN);
}

void s2lp_wakeup() {
	gpio_reset(ITSDK_S2LP_SDN_BANK,ITSDK_S2LP_SDN_PIN);
}

void s2lp_hwInit() {
	s2lp_spi_setCsHigh();		// disable CS
	s2lp_shutdown();			// The S2LP have a bug before V3 : when not shutdown it force value on SPI
								// and jam the other devices even when CS is not activ

}


void s2lp_init() {
	S2LP_SPI_StatusBytes status;
	uint8_t tmp;
	s2lp_wakeup();
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
    	// preventing conflict when accessing the eeprom
    	s2lp_shutdown();
    }

}

void s2lp_loadConfigFromEeprom(
		s2lp_eprom_config_t * eeprom_cnf,
		s2lp_eprom_offset_t * eeprom_offset,
		s2lp_config_t * cnf
) {

   switch(eeprom_cnf->xtalFreq) {
   case 0: cnf->xtalFreq = 24000000;
     break;
   case 1: cnf->xtalFreq = 25000000;
     break;
   case 2: cnf->xtalFreq = 26000000;
     break;
   case 3: cnf->xtalFreq = 48000000;
     break;
   case 4: cnf->xtalFreq = 50000000;
     break;
   case 5: cnf->xtalFreq = 52000000;
     break;
   case 0xff:
     // XTAL frequency is custom
     for(uint8_t i=0;i<4;i++){
       ((uint8_t*)&cnf->xtalFreq)[i]=((uint8_t*)&eeprom_cnf->customFreq)[3-i];
     }
     break;
   default:
	   log_warn("S2LP - Config - Freq not recognized\r\n");
	   cnf->xtalFreq = 50000000;
     break;
   }

   cnf->tcxo = eeprom_cnf->tcxo;
   cnf->band = eeprom_cnf->band;

   cnf->range = S2LP_RANGE_EXT_NONE;
   if (eeprom_cnf->range == 2 ) cnf->range = S2LP_RANGE_SKYWORKS_868;


   if(*(uint32_t*)eeprom_offset->offset != 0xffffffff) {
      for(uint8_t i=0;i<4;i++) {
        ((uint8_t*)&cnf->offset)[i]=eeprom_offset->offset[3-i];
      }

      if(cnf->offset < -100000 || cnf->offset > 100000) {
    	  cnf->offset=0;
      }
  } else {
	  cnf->offset=0;
  }

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
void s2lp_loadConfiguration(
		s2lp_config_t * s2lpConf
) {

	#if ITSDK_SIGFOX_NVM_SOURCE	== __SFX_NVM_M95640
		s2lp_eprom_config_t eepromConf1;
		eeprom_m95640_read(&ITSDK_DRIVERS_M95640_SPI,0x0000, 32, (uint8_t *)&eepromConf1);

		s2lp_eprom_offset_t eepromConf2;
		eeprom_m95640_read(&ITSDK_DRIVERS_M95640_SPI,0x0021, 4, (uint8_t *)&eepromConf2);

		// S2lp_hw config
		s2lp_loadConfigFromEeprom(
				&eepromConf1,
				&eepromConf2,
				s2lpConf
		);

		// Not clear why => when passing &(s2lpConf->id) it crash...
		// retrieve_data also extract the SigfoxID key and store it in RAM for internal use.
		uint32_t id;
		uint8_t rcz;
		enc_utils_retrieve_data(&id, s2lpConf->pac, &rcz);
		s2lpConf->id = id;
		s2lpConf->rcz = rcz;

		// Clean the key and aux not provided with this NVM source
		for (int i= 0 ; i < 16 ; i++) {
			s2lpConf->key[i]=0xFF;
			s2lpConf->aux[i]=0xFF;
		}

		// Search for the private key in the memory to fill the structure
		s2lp_sigfox_retreive_key(s2lpConf->id, s2lpConf->pac, s2lpConf->key);
		s2lp_sigfox_cifferKey(s2lpConf);

		s2lpConf->low_power_flag = ITSDK_SIGFOX_LOWPOWER;
		s2lpConf->payload_encryption = (( ITSDK_SIGFOX_ENCRYPTION & __PAYLOAD_ENCRYPT_SIGFOX) > 0)?1:0;

	#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
		#error "__SFX_NVM_LOCALEPROM Not yet implemented"
	#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_HEADERS
		int i;
		uint8_t pac[8] 		= ITSDK_SIGFOX_PAC;
		uint8_t key[16] 	= IDTSK_SIGFOX_KEY;
		uint8_t aux[16] 	= IDTSK_SIGFOX_AUX;

		s2lpConf->band 		= ITSDK_S2LP_CNF_BAND;
		s2lpConf->offset 	= ITSDK_S2LP_CNF_OFFSET;
		s2lpConf->range 	= ITSDK_S2LP_CNF_RANGE;
		s2lpConf->tcxo 		= ITSDK_S2LP_CNF_TCX0;
		s2lpConf->xtalFreq 	= ITSDK_S2LP_CNF_FREQ;
		s2lpConf->id		= ITSDK_SIGFOX_ID;
		itsdk_sigfox_getRczFromRegion(ITSDK_DEFAULT_REGION, &s2lpConf->rcz);
		for ( i =  0 ; i < 8 ; i++ ) s2lpConf->pac[i] = pac[i];
		for ( i =  0 ; i < 16 ; i++ ) {
			s2lpConf->key[i] = key[i];
			s2lpConf->aux[i] = aux[i];
		}
		sigfox_cifferKey(s2lpConf);
		s2lpConf->low_power_flag = ITSDK_SIGFOX_LOWPOWER;
		s2lpConf->payload_encryption = ITSDK_SIGFOX_ENCRYPTED;

		#warning "__SFX_NVM_HEADERS Not to be used in production"
	#endif

	// Init the seqId by reading the memory
	uint8_t tmp[SFX_NVMEM_BLOCK_SIZE];
	MCU_API_get_nv_mem(tmp);
	s2lpConf->seqId = (uint16_t)tmp[SFX_NVMEM_SEQ_NUM] + (((uint16_t)tmp[SFX_NVMEM_SEQ_NUM+1]) << 8);
	s2lpConf->lastReadRssi = S2LP_UNKNOWN_RSSI;
	s2lpConf->lastReceptionRssi = S2LP_UNKNOWN_RSSI;

	s2lp_applyConfig(s2lpConf);
}

void s2lp_printConfig(
		s2lp_config_t * s2lpConf
) {
	int i;
	log_info("------- Sigfox/S2LP configuration ----- \r\n");
	log_info("band: %d\r\n",s2lpConf->band);
	log_info("offset: %d\r\n",s2lpConf->offset);
	log_info("rssiOffset: %d\r\n",s2lpConf->rssiOffset);
	log_info("range: %d\r\n",s2lpConf->range);
	log_info("tcxo: %d\r\n",s2lpConf->tcxo);
	log_info("xtalFreq: %d\r\n",s2lpConf->xtalFreq);
	log_info("id: 0x%X\r\n",s2lpConf->id);
	log_info("rcz: %d\r\n",s2lpConf->rcz);
	log_info("pac: [ ");
	for ( i = 0 ; i < 8 ; i++ ) log_info("%02X",s2lpConf->pac[i]);
	log_info(" ]\r\n");

	s2lp_sigfox_unCifferKey(s2lpConf);
	log_info("key: [ ");
	for ( i = 0 ; i < 16 ; i++ ) log_info("%02X",s2lpConf->key[i]);
	log_info(" ]\r\n");
	s2lp_sigfox_cifferKey(s2lpConf);

	log_info("aux: [ ");
	for ( i = 0 ; i < 16 ; i++ ) log_info("%02X",s2lpConf->aux[i]);
	log_info(" ]\r\n");
	log_info("\r\n");

	uint8_t * versionStr;
	uint8_t sz;
	SIGFOX_API_get_version(&versionStr,&sz,VERSION_SIGFOX);
	log_info("Sigfox version : %s\r\n",versionStr);
	SIGFOX_API_get_version(&versionStr,&sz,VERSION_MCU);
	log_info("MCU version : %s\r\n",versionStr);
	SIGFOX_API_get_version(&versionStr,&sz,VERSION_RF);
	log_info("RF version : %s\r\n",versionStr);
	SIGFOX_API_get_version(&versionStr,&sz,VERSION_MONARCH);
	log_info("MONARCH version : %s\r\n",versionStr);
	SIGFOX_API_get_version(&versionStr,&sz,VERSION_DEVICE_CONFIG);
	log_info("Config version : %s\r\n",versionStr);

}


void s2lp_applyConfig(
	s2lp_config_t * cnf
){
	itsdk_sigfox_configInit(cnf);
	ST_RF_API_set_freq_offset(cnf->offset);
	ST_RF_API_set_tcxo(cnf->tcxo);
	if ( cnf->range ) {
		ST_RF_API_set_pa(1);
		ST_RF_API_gpio_tx_rx_pin(0);
		ST_RF_API_gpio_rx_pin(1);
		ST_RF_API_gpio_tx_pin(2);
		ST_RF_API_set_rssi_offset(-16);
	} else {
		ST_RF_API_set_pa(0);
		ST_RF_API_set_rssi_offset(0);
	}
}


#endif // ITSDK_SIGFOX_LIB test

