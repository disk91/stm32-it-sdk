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
#include <it_sdk/logger/logger.h>
#include <it_sdk/wrappers.h>
#include "stm32l0xx_hal.h"
#include "usart.h"

// ---------------------------------------------------------------------------
// serial 1 - is mapped to LPUART1
// ---------------------------------------------------------------------------

#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART1 ) > 0 || ( ITSDK_WITH_UART_RXIRQ & __UART_LPUART1 ) > 0
uint8_t __serial1_buffer[ITSDK_WITH_UART_RXIRQ_BUFSZ];
volatile uint8_t __serial1_bufferRd;
volatile uint8_t __serial1_bufferWr;
#endif
#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART2 ) > 0
uint8_t __serial2_buffer[ITSDK_WITH_UART_RXIRQ_BUFSZ];
volatile uint8_t __serial2_bufferRd;
volatile uint8_t __serial2_bufferWr;
#endif

/**
 * Init the Serial 1 extra configurations
 */
void serial1_init() {
#if ( ITSDK_WITH_UART_RXIRQ & __UART_LPUART1 ) > 0
    HAL_UART_Receive_IT(&hlpuart1, __serial1_buffer, 1);
    __serial1_bufferRd = 0;
    __serial1_bufferWr = 0;
#elif  ( ITSDK_WITH_UART_RXIRQ & __UART_USART1 ) > 0
    HAL_UART_Receive_IT(&huart1, __serial1_buffer, 1);
    __serial1_bufferRd = 0;
    __serial1_bufferWr = 0;
#endif
}

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

#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART1 ) > 0 || ( ITSDK_WITH_UART_RXIRQ & __UART_LPUART1 ) > 0

	if ( __serial1_bufferRd != __serial1_bufferWr ) {
		// char available
		*ch = __serial1_buffer[__serial1_bufferRd];
		itsdk_enterCriticalSection();
		__serial1_bufferRd = (__serial1_bufferRd + 1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1);
		itsdk_leaveCriticalSection();
		if ( __serial1_bufferRd != __serial1_bufferWr ) {
			return SERIAL_READ_PENDING_CHAR;
		} else {
			return SERIAL_READ_SUCCESS;
		}
	} else {
		return SERIAL_READ_NOCHAR;
	}


#else

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
#endif
}



// ---------------------------------------------------------------------------
// serial 2 - is mapped to USART2
// ---------------------------------------------------------------------------

/**
 * Init the Serial 2 extra configurations
 */
void serial2_init() {
#if  ( ITSDK_WITH_UART_RXIRQ & __UART_USART2 ) > 0
    HAL_UART_Receive_IT(&huart2, __serial2_buffer, 1);
    __serial2_bufferRd = 0;
    __serial2_bufferWr = 0;
#endif
}

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

#if  ( ITSDK_WITH_UART_RXIRQ & __UART_USART2 ) > 0

	if ( __serial2_bufferRd != __serial2_bufferWr ) {
		// char available
		*ch = __serial2_buffer[__serial2_bufferRd];
		itsdk_enterCriticalSection();
		__serial2_bufferRd = (__serial2_bufferRd + 1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1);
		itsdk_leaveCriticalSection();
		if ( __serial2_bufferRd != __serial2_bufferWr ) {
			return SERIAL_READ_PENDING_CHAR;
		} else {
			return SERIAL_READ_SUCCESS;
		}
	} else {
		return SERIAL_READ_NOCHAR;
	}

#else
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
#endif
}


// ---------------------------------------------------------------------------
// Global interrupt management
// ---------------------------------------------------------------------------


#if defined ITSDK_WITH_UART_RXIRQ && ITSDK_WITH_UART_RXIRQ != __UART_NONE

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
		log_info("overflow!");
	}

	if ( false
		#if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
			|| huart->Instance == LPUART1
		#endif
		#if ( ITSDK_WITH_UART & __UART_USART1 ) > 0
			|| huart->Instance == USART1
		#endif
	) {
		#if ( ITSDK_WITH_UART_RXIRQ & __UART_LPUART1 ) > 0 || ( ITSDK_WITH_UART_RXIRQ & __UART_USART1 ) > 0
		// at this point the data is in __serial1_buffer[__serial1_bufferWr]
		// only increment the pointer when we have an availbale space in the circular buffer
		if ( ((__serial1_bufferWr+1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1)) != __serial1_bufferRd  ) {
			__serial1_bufferWr = ((__serial1_bufferWr+1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1));
		}
		HAL_UART_Receive_IT(huart, &__serial1_buffer[__serial1_bufferWr], 1);
		#endif
	} else {
		#if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
		if ( huart->Instance == USART2 ) {
			#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART2 ) > 0
			// at this point the data is in __serial2_buffer[__serial2_bufferWr]
			if ( ((__serial2_bufferWr+1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1)) != __serial2_bufferRd  ) {
				__serial2_bufferWr = ((__serial2_bufferWr+1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1));
			}
			HAL_UART_Receive_IT(huart, &__serial2_buffer[__serial2_bufferWr], 1);
			#endif
		}
		#endif
	}

}

#endif

// ---------------------------------------------------------------------------
// debug - is not mapped
// ---------------------------------------------------------------------------

void debug_flush() {
}

void debug_print(char * msg) {
}


#endif
