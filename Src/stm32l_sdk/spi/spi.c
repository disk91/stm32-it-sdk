/* ==========================================================
 * spi.c - SPI Peripheral
 * Project : Disk91 SDK
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
 * ==========================================================
 */
#include <it_sdk/config.h>

#if ITSDK_WITH_SPI == __SPI_ENABLED
#include <stm32l_sdk/spi/spi.h>
#include "spi.h"


/**
 * Read the given SPI
 */
_SPI_Status spi_rwRegister(
		SPI_HandleTypeDef * spi,
		uint8_t	* toTransmit,
		uint8_t * toReceive,
		uint8_t   sizeToTransmit
) {
	return (_SPI_Status)HAL_SPI_TransmitReceive(
				spi,
				toTransmit,
				toReceive,
				sizeToTransmit,
				STM32_SPI_TIMEOUT
		);
}


_SPI_Status spi_readRegister(
		SPI_HandleTypeDef * spi,
		uint8_t	* toTransmit,
		uint8_t * toReceive,
		uint8_t   sizeToTransmit
) {

	return (_SPI_Status)HAL_SPI_TransmitReceive(
			spi,
			toTransmit,
			toReceive,
			sizeToTransmit,
			STM32_SPI_TIMEOUT
	);

}

_SPI_Status spi_write_byte(
		SPI_HandleTypeDef * spi,
		uint8_t Value
) {
  spi_wait4TransactionEnd(spi);
  return (_SPI_Status)HAL_SPI_Transmit(spi, (uint8_t*) &Value, 1, STM32_SPI_TIMEOUT);
}

void spi_wait4TransactionEnd(
		SPI_HandleTypeDef * spi
) {
	while (__HAL_SPI_GET_FLAG(spi, SPI_FLAG_TXE) == RESET);
}

void spi_reset(
		SPI_HandleTypeDef * spi
){
	  HAL_SPI_DeInit(spi);
	  HAL_SPI_Init(spi);
}


#endif // __SPI_ENABLED
