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

#if ITSDK_LOGGER_WITH_SEG_RTT == __ENABLE
#include <drivers/SeggerRTT/SEGGER_RTT.h>
#endif

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
#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART1 ) > 0 || ( ITSDK_WITH_UART_RXIRQ & __UART_LPUART1 ) > 0
	#if ( ITSDK_WITH_UART_RXIRQ & __UART_LPUART1 ) > 0
		UART_HandleTypeDef * _uart = &hlpuart1;
	#elif  ( ITSDK_WITH_UART_RXIRQ & __UART_USART1 ) > 0
		UART_HandleTypeDef * _uart = &huart1;
	#endif
    __HAL_UART_ENABLE_IT(_uart,UART_IT_ERR);
    __HAL_UART_ENABLE_IT(_uart,UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(_uart,UART_IT_TC);
    __HAL_UART_DISABLE_IT(_uart,UART_IT_TXE);
    // Clear pending interrupt & co
    HAL_UART_Receive_IT(_uart, __serial1_buffer, 1);
    _uart->Instance->RDR;
    _uart->Instance->ISR;
    _uart->Instance->ICR;
    // Reset circular buffer
    __serial1_bufferRd = 0;
    __serial1_bufferWr = 0;
#endif
}

/**
 * Connect & configure the serial1
 */
void serial1_connect() {
#if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0 || ( ITSDK_WITH_UART & __UART_USART1) > 0
	#if (ITSDK_WITH_UART & __UART_LPUART1 ) > 0
		UART_HandleTypeDef * _uart = &hlpuart1;
	#elif  ( ITSDK_WITH_UART & __UART_USART1) > 0
		UART_HandleTypeDef * _uart = &huart1;
	#endif
		HAL_UART_MspInit(_uart);
#endif
}

/**
 * Disconnect the serail1 from the pads
 */
void serial1_disconnect() {
#if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0 || ( ITSDK_WITH_UART & __UART_USART1) > 0
	#if (ITSDK_WITH_UART & __UART_LPUART1 ) > 0
		UART_HandleTypeDef * _uart = &hlpuart1;
	#elif  ( ITSDK_WITH_UART & __UART_USART1) > 0
		UART_HandleTypeDef * _uart = &huart1;
	#endif
		HAL_UART_MspDeInit(_uart);
#endif
}

/**
 * flushing pending transmission
 */
void serial1_flush() {
  #if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
     while(__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_BUSY) == SET);
     while(__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_TC) == RESET);
  #elif ( ITSDK_WITH_UART & __UART_USART1 ) > 0
	   while(__HAL_UART_GET_FLAG(&huart1, USART_ISR_BUSY) == SET);
	   while(__HAL_UART_GET_FLAG(&huart1, USART_ISR_TC) == RESET);
  #endif
}

void serial1_print(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg),0xFFFF);
  #elif ( ITSDK_WITH_UART & __UART_USART1 ) > 0
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg),0xFFFF);
  #endif
}

void serial1_write(uint8_t * bytes,uint16_t len) {
  #if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
	HAL_UART_Transmit(&hlpuart1, bytes, len,0xFFFF);
  #elif ( ITSDK_WITH_UART & __UART_USART1 ) > 0
	HAL_UART_Transmit(&huart1, bytes, len,0xFFFF);
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

/**
 * Change the Uart setting baudrate
 * Return BOOL_TRUE on success
 */
itsdk_bool_e serial1_changeBaudRate(serial_baudrate_e bd) {
	UART_HandleTypeDef * lhuart;
	#if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
		lhuart = &hlpuart1;
	#elif ( ITSDK_WITH_UART & __UART_USART1 ) > 0
		lhuart = &huart1;
	#else
		return BOOL_FALSE;
	#endif
	switch( bd ) {
		case SERIAL_SPEED_1200 : lhuart->Init.BaudRate = 1200; break;
		case SERIAL_SPEED_2400 : lhuart->Init.BaudRate = 2400; break;
		case SERIAL_SPEED_4800 : lhuart->Init.BaudRate = 4800; break;
		default:
		case SERIAL_SPEED_9600 : lhuart->Init.BaudRate = 9600; break;
		case SERIAL_SPEED_19200 : lhuart->Init.BaudRate = 19200; break;
		case SERIAL_SPEED_38400 : lhuart->Init.BaudRate = 38400; break;
		case SERIAL_SPEED_57600 : lhuart->Init.BaudRate = 57600; break;
		case SERIAL_SPEED_115200 : lhuart->Init.BaudRate = 115200; break;
	}
	serial1_flush();
	if (HAL_UART_Init(lhuart) != HAL_OK) {
	  return BOOL_FALSE;
	}
	serial1_init();
	return BOOL_TRUE;
}

// ---------------------------------------------------------------------------
// serial 2 - is mapped to USART2
// ---------------------------------------------------------------------------

/**
 * Init the Serial 2 extra configurations
 */
void serial2_init() {
#if  ( ITSDK_WITH_UART_RXIRQ & __UART_USART2 ) > 0
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_ERR);
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart2,UART_IT_TC);
    __HAL_UART_DISABLE_IT(&huart2,UART_IT_TXE);
    HAL_UART_Receive_IT(&huart2, __serial2_buffer, 1);
    huart2.Instance->RDR;
    huart2.Instance->ISR;
    huart2.Instance->ICR;
    __serial2_bufferRd = 0;
    __serial2_bufferWr = 0;
#endif
}

/**
 * Connect & configure the serial1
 */
void serial2_connect() {
#if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
	HAL_UART_MspInit(&huart2);
#endif
}

/**
 * Disconnect the serail1 from the pads
 */
void serial2_disconnect() {
#if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
	HAL_UART_MspDeInit(&huart2);
#endif
}

void serial2_flush() {
  #if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
  while((__HAL_UART_GET_FLAG(&huart2, USART_ISR_BUSY)) == SET);
  while((__HAL_UART_GET_FLAG(&huart2, USART_ISR_TC)) == RESET);
  #endif
}

void serial2_print(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg),0xFFFF);
  #endif
}

void serial2_write(uint8_t * bytes,uint16_t len) {
#if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
	HAL_UART_Transmit(&huart2, bytes, len,0xFFFF);
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

/**
 * Change the Uart setting baudrate
 * Return BOOL_TRUE on success
 */
itsdk_bool_e serial2_changeBaudRate(serial_baudrate_e bd) {
	UART_HandleTypeDef * lhuart;
	#if  ( ITSDK_WITH_UART_RXIRQ & __UART_USART2 )
	   lhuart = &huart2;
	#else
		return BOOL_FALSE;
	#endif
	switch( bd ) {
		case SERIAL_SPEED_1200 : lhuart->Init.BaudRate = 1200; break;
		case SERIAL_SPEED_2400 : lhuart->Init.BaudRate = 2400; break;
		case SERIAL_SPEED_4800 : lhuart->Init.BaudRate = 4800; break;
		default:
		case SERIAL_SPEED_9600 : lhuart->Init.BaudRate = 9600; break;
		case SERIAL_SPEED_19200 : lhuart->Init.BaudRate = 19200; break;
		case SERIAL_SPEED_38400 : lhuart->Init.BaudRate = 38400; break;
		case SERIAL_SPEED_57600 : lhuart->Init.BaudRate = 57600; break;
		case SERIAL_SPEED_115200 : lhuart->Init.BaudRate = 115200; break;
	}
	serial2_flush();
	if (HAL_UART_Init(lhuart) != HAL_OK) {
	  return BOOL_FALSE;
	}
	serial2_init();
	return BOOL_TRUE;
}


// ---------------------------------------------------------------------------
// Global interrupt management
// ---------------------------------------------------------------------------


#if defined ITSDK_WITH_UART_RXIRQ && ITSDK_WITH_UART_RXIRQ != __UART_NONE
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	// Clear the error flags
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (   __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)
		|| __HAL_UART_GET_FLAG(huart, UART_FLAG_NE)
		|| __HAL_UART_GET_FLAG(huart, UART_FLAG_FE)
	) {
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
	}

    do {
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
			// only increment the pointer when we have an available space in the circular buffer
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
	} while ( __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) );

}

#endif

// ---------------------------------------------------------------------------
// debug - is not mapped
// ---------------------------------------------------------------------------

void debug_flush() {
}

void debug_print(debug_print_type_e lvl, char * msg) {
#if ITSDK_LOGGER_WITH_SEG_RTT == __ENABLE
	static uint8_t wasEndLine = 1;
	if ( wasEndLine == 1 ) {
		switch (lvl) {
		case DEBUG_PRINT_DEBUG:
			SEGGER_RTT_WriteString(0,RTT_CTRL_BG_BRIGHT_CYAN);
			SEGGER_RTT_WriteString(0,"DEBUG    ");
			break;
		case DEBUG_PRINT_WARNING:
			SEGGER_RTT_WriteString(0,RTT_CTRL_BG_BRIGHT_MAGENTA);
			SEGGER_RTT_WriteString(0,"WARNING  ");
			break;
		case DEBUG_PRINT_ERROR:
			SEGGER_RTT_WriteString(0,RTT_CTRL_BG_BRIGHT_RED);
			SEGGER_RTT_WriteString(0,RTT_CTRL_TEXT_BRIGHT_BLACK);
			SEGGER_RTT_WriteString(0,"ERROR    ");
			break;
		default:
		case DEBUG_PRINT_INFO:
		case DEBUG_PRINT_ANY:
			SEGGER_RTT_WriteString(0,RTT_CTRL_BG_BRIGHT_WHITE);
			SEGGER_RTT_WriteString(0,RTT_CTRL_TEXT_BRIGHT_BLACK);
			SEGGER_RTT_WriteString(0,"INFO     ");
			break;
		}
	}
	SEGGER_RTT_WriteString(0,RTT_CTRL_RESET);
	switch (lvl) {
	case DEBUG_PRINT_DEBUG:
		SEGGER_RTT_WriteString(0,RTT_CTRL_TEXT_BRIGHT_CYAN);
		break;
	case DEBUG_PRINT_WARNING:
		SEGGER_RTT_WriteString(0,RTT_CTRL_TEXT_BRIGHT_MAGENTA);
		break;
	case DEBUG_PRINT_ERROR:
		SEGGER_RTT_WriteString(0,RTT_CTRL_TEXT_BRIGHT_RED);
		break;
	default:
		break;
	}
	SEGGER_RTT_WriteString(0, msg);
	SEGGER_RTT_WriteString(0,RTT_CTRL_RESET);
	int v = strlen(msg);
	wasEndLine = ( msg[v-1] == '\r' || msg[v-1] == '\n' )?1:0;
#endif
}


#endif
