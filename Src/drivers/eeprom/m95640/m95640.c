/* ==========================================================
 * m95640p.c - Driver for SPI EEPROM M95640
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
#include <it_sdk/itsdk.h>
#include <it_sdk/logger/logger.h>
#include <drivers/eeprom/m95640/m95640.h>


#if ITSDK_DRIVERS_M95640 == 1


void eeprom_m95640_hwInit() {
	eeprom_m95640_chipUnSelected();
}

/**
 * Needed initializations
 * Assuming the GPIO & SPI are already configured
 */
bool eeprom_m95640_init(ITSDK_SPI_HANDLER_TYPE * spi) {
	uint8_t status;

	log_debug("M95640 Init\r\n");

	// Check the eeprom status and change configuration SRWD bit to ensure
	// the eeprom is present and ready for next steps
	status = eeprom_m95640_getStatus(spi, true);
    if( ( status & M95640_EEPROM_STATUS_MSQ ) == M95640_EEPROM_STATUS_SRWD ) {
 	    // if it is EEPROM_STATUS_SRWD, ok the EEPROM is present and ready to work
	    status=1;
	} else {
		log_debug(" > Enable Write & SRWD \r\n");
	    eeprom_m95640_enableWrite(spi, true);
	    itsdk_delayMs(50);

	    // the bit may be not set (first time we see this EEPROM), try to set it
	    eeprom_m95640_setSRWD(spi, true);
	    itsdk_delayMs(50);

	    // check again
	    status = eeprom_m95640_getStatus(spi, true);

	    if( ( status & M95640_EEPROM_STATUS_MSQ ) == M95640_EEPROM_STATUS_SRWD ) {
	        // if it is EEPROM_STATUS_SRWD, ok the EEPROM is present and ready to work
	        status=1;
	    } else {
	        // else no EEPROM is present
	    	log_debug(" > Not Found \r\n");
	        status = 0;
	    }
	}


    if ( status == 1 ) {
    	log_debug(" > Found \r\n");

    	return true;
    }
    return false;

}

/**
 * Read cNbBytes at nAddress from the eeprom
 */

void eeprom_m95640_read(
		ITSDK_SPI_HANDLER_TYPE * spi,
		uint16_t nAddress,
		uint8_t cNbBytes,
		uint8_t* pcBuffer
){
	eeprom_m95640_access(
			spi,
			nAddress,
			cNbBytes,
			pcBuffer,
			M95640_READ_DATA
	);
}

/**
 * Write cNbBytes at nAddress from the eeprom
 */
void eeprom_m95640_write(
		ITSDK_SPI_HANDLER_TYPE * spi,
		uint16_t nAddress,
		uint8_t cNbBytes,
		uint8_t* pcBuffer
){
	eeprom_m95640_access(
			spi,
			nAddress,
			cNbBytes,
			pcBuffer,
			M95640_WRITE_DATA
	);
}



uint8_t eeprom_m95640_getStatus(
		ITSDK_SPI_HANDLER_TYPE * spi,					// spi port to be used
		bool				 withChipSelect			// update the chip select signal when true
												    // when false, CS must be already activ
){
  uint8_t status[2];
  uint8_t cmd[] = {M95640_EEPROM_CMD_RDSR, 0xFF};

  if (withChipSelect) eeprom_m95640_chipSelected();

  // Send command Read Status Register
  spi_readRegister(
  		  spi,
          cmd,
		  status,
  		  2
    );

  if (withChipSelect) eeprom_m95640_chipUnSelected();

  return status[1];

}


/**
 * Read/Write cNbBytes at nAddress from the eeprom
 */
void eeprom_m95640_access(
		ITSDK_SPI_HANDLER_TYPE * spi,					// spi port to be used
		uint16_t nAddress,							// Address in the eeprom
		uint8_t cNbBytes,							// Size of the data to be accessed
		uint8_t* pcBuffer,							// Buffer where data are / where to store
		M95640_ACCESS access						// type of access Read or Write
){
  uint8_t cmd[3];
  uint8_t dummy[3];
  if ( access == M95640_READ_DATA ) {
    cmd[0] = M95640_EEPROM_CMD_READ;
  } else {
    cmd[0] = M95640_EEPROM_CMD_WRITE;
  }

  cmd[1] = (uint8_t)(nAddress >> 8);
  cmd[2] = (uint8_t)(nAddress & 0xFF);

  // Wait the end of a previous write operation
  eeprom_m95640_ensureEndOfWrite(spi,true);

  // Allow to write the eeprom
  if ( access == M95640_WRITE_DATA ) {
	  eeprom_m95640_enableWrite(spi,true);
  }

  // enable eeprom spi
  eeprom_m95640_chipSelected();

  // Write the header bytes and read the status bytes
  spi_readRegister(
  		  spi,
          cmd,
          dummy,
  		  3
    );

  // Read the registers according to the number of bytes
  for (int index = 0; index < cNbBytes; index++) {
	  if ( access==M95640_READ_DATA ) {
		  spi_rwRegister(
		 	  spi,
			  dummy,
		 	  &(pcBuffer)[index],
		 	  1
		  );
	  } else {
		  spi_rwRegister(
		 	  spi,
			  &(pcBuffer)[index],
			  dummy,
		 	  1
		  );
	  }
  }

  /* Put the SPI chip select high to end the transaction */
  eeprom_m95640_chipUnSelected();
/*
  log_info("Rd/Wr from Eeprom (m95640.c)[%d] @ 0x%X : [ ",cNbBytes,nAddress);
  for (int i=0; i< cNbBytes ; i++) {
	log_info("%02X ",pcBuffer[i]);
  }
  log_info("]\r\n");
*/
}




/**
 * Manage Chip Select Signal
 */
void eeprom_m95640_chipSelected() {
	gpio_reset(ITSDK_DRIVERS_M95640_SPI_CS_BANK,ITSDK_DRIVERS_M95640_SPI_CS_PIN);
}

void eeprom_m95640_chipUnSelected() {
	gpio_set(ITSDK_DRIVERS_M95640_SPI_CS_BANK,ITSDK_DRIVERS_M95640_SPI_CS_PIN);
}

/**
 * Manage Write process
 */
void eeprom_m95640_ensureEndOfWrite(
		ITSDK_SPI_HANDLER_TYPE * spi,					// spi port to be used
		bool				 	 withChipSelect			// update the chip select signal when true
														// when false, CS must be already activ
) {
  uint8_t cmd[1] = { M95640_EEPROM_CMD_RDSR };
  uint8_t dummy[1] = { 0xFF };
  uint8_t status[1] = { 0xFF };

  if (withChipSelect) eeprom_m95640_chipSelected();

  // Send command - Status Register
  spi_readRegister(
  		  spi,
          cmd,
		  status,
  		  1
    );

  // Polling on status register
  do{
	  spi_readRegister(
	  		  spi,
	          dummy,
			  status,
	  		  1
	    );
  } while( status[0] & M95640_EEPROM_STATUS_WIP);

  if (withChipSelect) eeprom_m95640_chipUnSelected();
}

/**
 * Allow to Write into the eeprom
 */
void eeprom_m95640_enableWrite(
		ITSDK_SPI_HANDLER_TYPE * spi,					// spi port to be used
		bool				 	 withChipSelect			// update the chip select signal when true
												    	// when false, CS must be already activ
){
  uint8_t cmd[1] = { M95640_EEPROM_CMD_WREN };
  uint8_t status[1] = { 0xFF };

  if (withChipSelect) eeprom_m95640_chipSelected();

  // Send command - Write enable
  spi_rwRegister(
  		  spi,
          cmd,
		  status,
  		  1
    );

  if (withChipSelect) eeprom_m95640_chipUnSelected();

}

/*
 * Status Register Write Protect set to 1 => Write protected
 */
uint8_t eeprom_m95640_setSRWD(
		ITSDK_SPI_HANDLER_TYPE * spi,					// spi port to be used
		bool				     withChipSelect			// update the chip select signal when true
												    	// when false, CS must be already activ
){
  uint8_t cmd[2] = {M95640_EEPROM_CMD_WRSR, M95640_EEPROM_STATUS_SRWD};
  uint8_t status[2] = {0xFF,0xFF};

  if (withChipSelect) eeprom_m95640_chipSelected();

  // Send command - Status Register Write Disable
  spi_rwRegister(
  		  spi,
          cmd,
		  status,
  		  2
    );

  if (withChipSelect) eeprom_m95640_chipUnSelected();

  return status[1];
}


#endif

