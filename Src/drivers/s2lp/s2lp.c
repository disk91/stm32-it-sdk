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


void s2lp_init() {
	S2LP_SPI_StatusBytes status;
	uint8_t tmp;
	do {
	    // Delay for state transition
	    for(volatile uint8_t i=0; i!=0xFF; i++);

	    // Reads the MC_STATUS register
	    status = s2lp_spi_readRegisters(&ITSDK_S2LP_SPI,0x8E, 1, &tmp);
	} while(status.MC_STATE!=MC_STATE_READY);

}

/*
void S2LPManagementIdentificationRFBoard(void)
{
  uint8_t tmp;
  StatusBytes status;

  do{
    // Delay for state transition
    for(volatile uint8_t i=0; i!=0xFF; i++);

    // Reads the MC_STATUS register
    status = RadioSpiReadRegisters(0x8E, 1, &tmp);
  }while(status.MC_STATE!=MC_STATE_READY);

  RadioSpiReadRegisters(0xF1, 1, &tmp);


  s_S2LPCut=(S2LPCutType)tmp;

  if((s_S2LPCut==S2LP_CUT_2_0) || (s_S2LPCut==S2LP_CUT_2_1))
  {
    RadioEnterShutdown();
  }

  RadioSetHasEeprom(EepromIdentification());

  if(!RadioGetHasEeprom()) // EEPROM is not present
  {
    RadioExitShutdown();
    if(S2LPManagementComputeXtalFrequency()==0)
    {
      // if it fails force it to 50MHz
      S2LPRadioSetXtalFrequency(50000000);
    }
  }
  else  // EEPROM found
  {
    //read the memory and set the variable
    uint8_t tmpBuffer[32];
    EepromRead(0x0000, 32, tmpBuffer);
    uint32_t xtal;
    float foffset=0;
    if(tmpBuffer[0]==0 || tmpBuffer[0]==0xFF) {
      // this one happens in production where the E2PROM is here but blank
      S2LPManagementEnableTcxo();
      if((s_S2LPCut==S2LP_CUT_2_0) || (s_S2LPCut==S2LP_CUT_2_1))
      {
      RadioExitShutdown();
      }
      S2LPManagementComputeXtalFrequency();
      return;
    }
    switch(tmpBuffer[1]) {
    case 0:
      xtal = 24000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 1:
      xtal = 25000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 2:
      xtal = 26000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 3:
      xtal = 48000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 4:
      xtal = 50000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 5:
      xtal = 52000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 0xff:
      // XTAL freqeuncy is custom
      for(uint8_t i=0;i<4;i++)
      {
        ((uint8_t*)&xtal)[i]=tmpBuffer[30-i];
      }
      S2LPRadioSetXtalFrequency(xtal);
      break;
    default:
      S2LPManagementComputeXtalFrequency();
      break;
    }

    // TCXO field
    if(tmpBuffer[31]==1)
    {
      S2LPManagementSetTcxo(1);
    }

    S2LPManagementSetBand(tmpBuffer[3]);

    RangeExtType range=RANGE_EXT_NONE;
    if(tmpBuffer[5]==2)
    {
      range = RANGE_EXT_SKYWORKS_868;
    }
    S2LPManagementSetRangeExtender(range);

    EepromRead(0x0021,4,tmpBuffer);

    if(*(uint32_t*)tmpBuffer != 0xffffffff)
    {
      for(uint8_t i=0;i<4;i++)
      {
        ((uint8_t*)&foffset)[i]=tmpBuffer[3-i];
      }

      if(foffset<-100000 || foffset>100000)
      {
        foffset=0;
      }
    }
    S2LPManagementSetOffset((int32_t)foffset);
  }

  if((s_S2LPCut==S2LP_CUT_2_0) || (s_S2LPCut==S2LP_CUT_2_1))
  {
    RadioExitShutdown();
  }

}
 */




#endif // ITSDK_SIGFOX_LIB test

