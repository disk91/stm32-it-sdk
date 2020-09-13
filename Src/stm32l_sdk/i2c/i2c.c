/* ==========================================================
 * i2c.c - I2C Peripheral
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 fev. 2019
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2019 Disk91
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

#if ITSDK_WITH_I2C == __I2C_ENABLED
#include <stm32l_sdk/i2c/i2c.h>
#include <it_sdk/wrappers.h>

/**
 * I2C Memory write
 */
_I2C_Status i2c_memWrite(
		ITSDK_I2C_HANDLER_TYPE * i2c,				// i2c handler
		uint16_t  devAdr,							// Device Address => 7 bits non shifted
		uint16_t  memAdr,							// Memory address to access
		uint16_t  memAdrSize,						// 8 for 8b, 16 for 16 bits ...
		uint8_t * values,							// Data to be written
		uint16_t  size								// Size of the data to be written
) {
	devAdr <<= 1;
	switch (memAdrSize) {
	case 8:
		memAdrSize = I2C_MEMADD_SIZE_8BIT;
		break;
	case 16:
		memAdrSize = I2C_MEMADD_SIZE_16BIT;
		break;
	default:
		return __I2C_ERROR;
		break;
	}
	return (_I2C_Status)HAL_I2C_Mem_Write( i2c, devAdr, memAdr, memAdrSize, values, size, ITSDK_I2C_TIMEOUT );
}

/**
 * Write the given I2C
 * The address is the 7 bit device address => No shift in the parameter
 */
_I2C_Status i2c_write(
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,
		uint8_t * values,
		uint16_t  size
) {
	devAdr <<= 1;
	return (_I2C_Status)HAL_I2C_Master_Transmit(i2c, devAdr, values, size, ITSDK_I2C_TIMEOUT);
}

/**
 * Write the given I2C 8b Register
 * The address is the 7 bit device address => No shift in the parameter
 */
_I2C_Status i2c_write8BRegister(
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,			// Non shifted device address
		uint16_t  regAdr,			// Register address (8b or 16b�
		uint8_t   value,			// 8B value to be written
		uint16_t  regSize			// Register address size 1B or 2B
) {
	uint8_t _buffer[3];
	uint8_t sz;
	if (regSize == 1) {
		_buffer[0] = (uint8_t)regAdr;
		_buffer[1] = value;
		sz=2;
	} else if (regSize == 2) {
		_buffer[0] = (uint8_t)(regAdr >> 8);
		_buffer[1] = (uint8_t)(regAdr & 0xFF);
		_buffer[2] = value;
		sz=3;
	} else return __I2C_ERROR;
	return i2c_write(i2c, devAdr, _buffer, sz);
}

/**
 * Write the given I2C 16b Register / LSB First
 * The address is the 7 bit device address => No shift in the parameter
 */
_I2C_Status i2c_write16BRegister(
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,			// Non shifted device address
		uint16_t  regAdr,			// Register address (8b or 16b�
		uint16_t  value,			// 16B value to be written
		uint16_t  regSize			// Register address size 1B or 2B
) {
	uint8_t _buffer[4];
	uint8_t sz;
	if (regSize == 1) {
		_buffer[0] = (uint8_t)regAdr;
		_buffer[1] = (uint8_t)(value & 0xFF);
		_buffer[2] = (uint8_t)(value >> 8);
		sz=3;
	} else if (regSize == 2) {
		_buffer[0] = (uint8_t)(regAdr >> 8);
		_buffer[1] = (uint8_t)(regAdr & 0xFF);
		_buffer[2] = (uint8_t)(value & 0xFF);
		_buffer[3] = (uint8_t)(value >> 8);
		sz=4;
	} else return __I2C_ERROR;
	return i2c_write(i2c, devAdr, _buffer, sz);
}


/**
 * I2C Memory read
 */
_I2C_Status i2c_memRead(
		ITSDK_I2C_HANDLER_TYPE * i2c,				// i2c handler
		uint16_t  devAdr,							// Device Address => 7 bits non shifted
		uint16_t  memAdr,							// Memory address to access
		uint16_t  memAdrSize,						// 8 for 8b, 16 for 16 bits ...
		uint8_t * values,							// Where to store read data
		uint16_t  size								// Size of the data to be read
) {
	devAdr <<= 1;
	switch (memAdrSize) {
	case 8:
		memAdrSize = I2C_MEMADD_SIZE_8BIT;
		break;
	case 16:
		memAdrSize = I2C_MEMADD_SIZE_16BIT;
		break;
	default:
		return __I2C_ERROR;
		break;
	}
	return (_I2C_Status)HAL_I2C_Mem_Read( i2c, devAdr, memAdr, memAdrSize, values, size, ITSDK_I2C_TIMEOUT );
}



/**
 * Read the given I2C
 * The address is the 7 bit device address => No shift in the parameter
 */
_I2C_Status i2c_read(
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,
		uint8_t * values,
		uint16_t  size
) {
	devAdr <<= 1;
	return (_I2C_Status)HAL_I2C_Master_Receive(i2c, devAdr, values, size,ITSDK_I2C_TIMEOUT);
}

/**
 * Write the given I2C 8b Register
 * The address is the 7 bit device address => No shift in the parameter
 */
_I2C_Status i2c_read8BRegister(
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,			// Non shifted device address
		uint16_t  regAdr,			// Register address (8b or 16b�
		uint8_t * value,			// 8B value to be read
		uint16_t  regSize			// Register address size 1B or 2B
) {
	uint8_t _buffer[3];
	uint8_t sz;
	// write address
	if (regSize == 1) {
		_buffer[0] = (uint8_t)regAdr;
		sz=1;
	} else if (regSize == 2) {
		_buffer[0] = (uint8_t)(regAdr >> 8);
		_buffer[1] = (uint8_t)(regAdr & 0xFF);
		sz=2;
	} else{
		return __I2C_ERROR;
	}
	if ( i2c_write(i2c, devAdr, _buffer, sz) == __I2C_OK ) {
		return i2c_read(i2c,devAdr,value,1);
	} else {
		return __I2C_ERROR;
	}
}

/**
 * Read the given I2C 16b Register / LSB First
 * The address is the 7 bit device address => No shift in the parameter
 */
_I2C_Status i2c_read16BRegister(
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,			// Non shifted device address
		uint16_t  regAdr,			// Register address (8b or 16b�
		uint16_t * value,			// 16b value to be read
		uint16_t  regSize			// Address's register size 1B or 2B
) {
	uint8_t _buffer[3];
	uint8_t sz;
	// write address
	if (regSize == 1) {
		_buffer[0] = (uint8_t)regAdr;
		sz=1;
	} else if (regSize == 2) {
		_buffer[0] = (uint8_t)(regAdr >> 8);
		_buffer[1] = (uint8_t)(regAdr & 0xFF);
		sz=2;
	} else return __I2C_ERROR;
	if ( i2c_write(i2c, devAdr, _buffer, sz) == __I2C_OK ) {
		_I2C_Status r = i2c_read(i2c,devAdr,_buffer,2);
		*value= (_buffer[0])+(_buffer[1] << 8);
		return r;
	} else return __I2C_ERROR;
}


/**
 * Reset the I2C port
 */
void i2c_reset(
		ITSDK_I2C_HANDLER_TYPE * i2c
){
	  HAL_I2C_DeInit(i2c);
	  HAL_I2C_Init(i2c);
}


#endif // __I2C_ENABLED
