/* ==========================================================
 * uart_wrapper.c - wrapper function for uarts
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 12 sept. 2018
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
 * Serial1 is LPUART
 * Serial2 is USART2
 * Debuh is non affected
 *
 * ==========================================================
 */
#include <string.h>
#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0

#include <it_sdk/wrappers.h>
#include "stm32l0xx_hal.h"
#include "usart.h"

// ---------------------------------------------------------------------------
// serial 1 - is mapped to LPUART1
// ---------------------------------------------------------------------------

/**
 * flushing pending transmission
 */
void serial1_flush() {
  #if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
     while(__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_BUSY) == SET);
  #elif ( ITSDK_WITH_UART & __UART_USART1 ) > 0
   while(__HAL_UART_GET_FLAG(&huart1, USART_ISR_BUSY) == SET);
  #endif
}

void serial1_print(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg),0xFFFF);
  #elif ( ITSDK_WITH_UART & __UART_USART1 ) > 0
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg),0xFFFF);
  #endif
}

void serial1_println(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
	serial1_print(msg);
	char * eol = "\r\n";
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)eol, strlen(eol),0xFFFF);
  #elif ( ITSDK_WITH_UART & __UART_USART1 ) > 0
	serial1_print(msg);
	char * eol = "\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)eol, strlen(eol),0xFFFF);
  #endif
}

serial_read_response_e serial1_read(char * ch) {
#if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0

	// buffer overflow
	if (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_ORE);
	}

	// get one of the pending char if some.
	if (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE)){
		*ch = hlpuart1.Instance->RDR & 0x1FF;
		if (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE)) {
			return SERIAL_READ_PENDING_CHAR;
		} else {
			return SERIAL_READ_SUCCESS;
		}
	}
	return SERIAL_READ_NOCHAR;
#elif ( ITSDK_WITH_UART & __UART_USART1 ) > 0

	// buffer overflow
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
	}

	// get one of the pending char if some.
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)){
		*ch = huart1.Instance->RDR & 0x1FF;
		if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
			return SERIAL_READ_PENDING_CHAR;
		} else {
			return SERIAL_READ_SUCCESS;
		}
	}
	return SERIAL_READ_NOCHAR;
#else
	return SERIAL_READ_FAILED;
#endif
}


// ---------------------------------------------------------------------------
// serial 2 - is mapped to USART2
// ---------------------------------------------------------------------------

void serial2_flush() {
  #if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
     while(__HAL_UART_GET_FLAG(&huart2, USART_ISR_BUSY) == SET);
  #endif
}

void serial2_print(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg),0xFFFF);
  #endif
}

void serial2_println(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
	serial2_print(msg);
	char * eol = "\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)eol, strlen(eol),0xFFFF);
  #endif
}

serial_read_response_e serial2_read(char * ch) {
#if ( ITSDK_WITH_UART & __UART_USART2 ) > 0

	// buffer overflow
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_ORE);
	}

	// get one of the pending char if some.
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)){
		*ch = huart2.Instance->RDR & 0x1FF;
		if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
			return SERIAL_READ_PENDING_CHAR;
		} else {
			return SERIAL_READ_SUCCESS;
		}
	}
	return SERIAL_READ_NOCHAR;
#else
	return SERIAL_READ_FAILED;
#endif
}

// ---------------------------------------------------------------------------
// debug - is not mapped
// ---------------------------------------------------------------------------

void debug_flush() {
}

void debug_print(char * msg) {
}


#endif
