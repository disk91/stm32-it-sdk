/* ==========================================================
 * m95640p.h - Driver for SPI EEPROM M95640
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 7 nov. 2018
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

#ifndef IT_SDK_DRIVERS_EEPROM_M95640_H_
#define IT_SDK_DRIVERS_EEPROM_M95640_H_

#include <stdbool.h>
#include <it_sdk/config.h>
#if ITSDK_DRIVERS_M95640 == __ENABLE
#include <it_sdk/wrappers.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
	#include <stm32l_sdk/spi/spi.h>
#else
	#error "M95640 driver is not supported for this platform"
#endif

// Largely inspired by STMicroelectronics source code << COPYRIGHT(c) 2018 STMicroelectronics >>

// EEPROM SPI commands
#define M95640_EEPROM_CMD_WREN    		0x06    // Write Enable
#define M95640_EEPROM_CMD_WRDI    		0x04    // Write Disable
#define M95640_EEPROM_CMD_RDSR    		0x05    // Read Status Register
#define M95640_EEPROM_CMD_WRSR    		0x01    // Write Status Register
#define M95640_EEPROM_CMD_READ    		0x03    // Read from Memory Array
#define M95640_EEPROM_CMD_WRITE   		0x02    // Write to Memory Array

// EEPROM SPI status
#define M95640_EEPROM_STATUS_SRWD    	0x80    // Status Register Write Disable
#define M95640_EEPROM_STATUS_BP      	0x0C    // Block Protect
#define M95640_EEPROM_STATUS_WEL     	0x02    // Write Enable
#define M95640_EEPROM_STATUS_WIP     	0x01    // Write in Progress
#define M95640_EEPROM_STATUS_MSQ		0xF0	// Most significant quartet for masking

// End of >> COPYRIGHT(c) 2018 STMicroelectronics <<

// *** Public prototypes
void eeprom_m95640_hwInit();
bool eeprom_m95640_init(ITSDK_SPI_HANDLER_TYPE * spi);

void eeprom_m95640_write(
		ITSDK_SPI_HANDLER_TYPE * spi,
		uint16_t nAddress,
		uint8_t cNbBytes,
		uint8_t* pcBuffer
);

void eeprom_m95640_read(
		ITSDK_SPI_HANDLER_TYPE * spi,
		uint16_t nAddress,
		uint8_t cNbBytes,
		uint8_t* pcBuffer
);

uint8_t eeprom_m95640_getStatus(
		ITSDK_SPI_HANDLER_TYPE * spi,					// spi port to be used
		bool				 	withChipSelect			// update the chip select signal when true
														// when false, CS must be already activ
);


// *** Internal functions
typedef enum {
	M95640_READ_DATA	= 0x00,
	M95640_WRITE_DATA 	= 0x01
} M95640_ACCESS;


void eeprom_m95640_chipSelected();
void eeprom_m95640_chipUnSelected();
void eeprom_m95640_enableWrite(
		ITSDK_SPI_HANDLER_TYPE * spi,				// spi port to be used
		bool				 withChipSelect			// update the chip select signal when true
												    // when false, CS must be already activ
);
void eeprom_m95640_ensureEndOfWrite(
		ITSDK_SPI_HANDLER_TYPE * spi,				// spi port to be used
		bool				 withChipSelect			// update the chip select signal when true
												    // when false, CS must be already activ
);
void eeprom_m95640_access(
		ITSDK_SPI_HANDLER_TYPE * spi,				// spi port to be used
		uint16_t nAddress,							// Address in the eeprom
		uint8_t cNbBytes,							// Size of the data to be accessed
		uint8_t* pcBuffer,							// Buffer where data are / where to store
		M95640_ACCESS access						// type of access Read or Write
);

uint8_t eeprom_m95640_setSRWD(
		ITSDK_SPI_HANDLER_TYPE * spi,				// spi port to be used
		bool				 withChipSelect			// update the chip select signal when true
												    // when false, CS must be already activ
);

#endif // ITSDK_DRIVERS_M95640
#endif /* IT_SDK_DRIVERS_EEPROM_M95640_H_ */
