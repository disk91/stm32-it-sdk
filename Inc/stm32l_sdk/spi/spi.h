/* ==========================================================
 * spi.h - stm32L0x1 spi header
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 4 nov. 2018
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
#ifndef STM32L_SDK_SPI_SPI_H_
#define STM32L_SDK_SPI_SPI_H_

#define __SPI_HANDLER_TYPE		SPI_HandleTypeDef
#define STM32_SPI_TIMEOUT		100					// sounds like milliseconds

typedef enum
{
  SPI_OK       = 0x00U,
  SPI_ERROR    = 0x01U,
  SPI_BUSY     = 0x02U,
  SPI_TIMEOUT  = 0x03U
} _SPI_Status;

_SPI_Status spi_rwRegister(
		SPI_HandleTypeDef * spi,
		uint8_t	* toTransmit,
		uint8_t * toReceive,
		uint8_t   sizeToTransmit
);

_SPI_Status spi_readRegister(
		SPI_HandleTypeDef * spi,
		uint8_t	* toTransmit,
		uint8_t * toReceive,
		uint8_t   sizeToTransmit
);

_SPI_Status spi_write_byte(
		SPI_HandleTypeDef * spi,
		uint8_t Value
);

void spi_wait4TransactionEnd(
		SPI_HandleTypeDef * spi
);

void spi_reset(
		SPI_HandleTypeDef * spi
);

#endif /* STM32L_SDK_SPI_SPI_H_ */
