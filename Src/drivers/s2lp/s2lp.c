/* ==========================================================
 * s2lp.c - S2LP (STm SubGhz transceiver) driver for sigfox
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 04 nov. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018  IngeniousThings and Disk91
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
#if ITSDK_SIGFOX_LIB == __SIGFOX_S2LP

#include <it_sdk/itsdk.h>
#include <it_sdk/sigfox/sigfox.h>
#include <drivers/s2lp/s2lp_spi.h>
#include <drivers/s2lp/s2lp.h>
#include <it_sdk/logger/logger.h>
#include <drivers/s2lp/st_rf_api.h>


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
		log_info("Try to connect to S2LP\r\n");
	    // Delay for state transition
	    for(volatile uint8_t i=0; i!=0xFF; i++);

	    // Reads the MC_STATUS register
	    status = s2lp_spi_readRegisters(&ITSDK_S2LP_SPI,S2LP_REG_MC_STATE0, 1, &tmp);
	    log_info("...Returned 0x%X\r\n",status);
	} while(status.MC_STATE!=MC_STATE_READY);
	// Return 0 when ready
	log_info("S2LP connection success returned value 0x%0X\r\n",tmp);

	// Get the version
    s2lp_spi_readRegisters(&ITSDK_S2LP_SPI,S2LP_REG_DEVICE_INFO0, 1, &tmp);
    log_info("S2LP version 0x%0X\r\n",tmp);
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


void s2lp_loadConfigFromHeaders(
		s2lp_config_t * cnf
) {
	cnf->band =  ITSDK_S2LP_CNF_BAND;
	cnf->offset = ITSDK_S2LP_CNF_OFFSET;
	cnf->range = ITSDK_S2LP_CNF_RANGE;
	cnf->tcxo = ITSDK_S2LP_CNF_TCX0;
	cnf->xtalFreq = ITSDK_S2LP_CNF_FREQ;
}


void s2lp_applyConfig(
	s2lp_config_t * cnf
){
	  ST_RF_API_set_freq_offset(cnf->offset);
	  ST_RF_API_set_tcxo(cnf->tcxo);
}




#endif // ITSDK_SIGFOX_LIB test

