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
#if ITSDK_PLATFORM == __PLATFORM_STM32L0 || ITSDK_PLATFORM == __PLATFORM_STM32L4 || ITSDK_PLATFORM == __PLATFORM_STM32WLE
#include <it_sdk/logger/logger.h>
#include <it_sdk/wrappers.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
	#include "stm32l0xx_hal.h"
#elif ITSDK_PLATFORM == __PLATFORM_STM32L4
	#include "stm32l4xx.h"
	#include "stm32l4xx_hal_def.h"
#elif ITSDK_PLATFORM == __PLATFORM_STM32WLE
	#include "stm32wlxx_hal.h"
#endif
#if ITSDK_WITH_UART > 0
	#include "usart.h"
#endif

#if ITSDK_LOGGER_WITH_SEG_RTT == __ENABLE
#include <drivers/SeggerRTT/SEGGER_RTT.h>
#endif

// ---------------------------------------------------------------------------
// serial 1 - is mapped to LPUART1 or USART1
// ---------------------------------------------------------------------------

#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART1 ) > 0 || ( ITSDK_WITH_UART_RXIRQ & __UART_LPUART1 ) > 0
uint8_t __serial1_buffer[ITSDK_WITH_UART_RXIRQ_BUFSZ];
volatile uint8_t __serial1_bufferRd = 0;
volatile uint8_t __serial1_bufferWr = 0;
#endif
#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART2 ) > 0
uint8_t __serial2_buffer[ITSDK_WITH_UART_RXIRQ_BUFSZ];
volatile uint8_t __serial2_bufferRd = 0;
volatile uint8_t __serial2_bufferWr = 0;
#endif
#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART3 ) > 0
uint8_t __serial3_buffer[ITSDK_WITH_UART_RXIRQ_BUFSZ];
volatile uint8_t __serial3_bufferRd = 0;
volatile uint8_t __serial3_bufferWr = 0;
#endif
#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART4 ) > 0
uint8_t __serial4_buffer[ITSDK_WITH_UART_RXIRQ_BUFSZ];
volatile uint8_t __serial4_bufferRd = 0;
volatile uint8_t __serial4_bufferWr = 0;
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

	__HAL_UART_DISABLE_IT(_uart,UART_IT_ERR);
    __HAL_UART_DISABLE_IT(_uart,UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(_uart,UART_IT_TC);
    __HAL_UART_DISABLE_IT(_uart,UART_IT_TXE);

    // Reset circular buffer
	itsdk_enterCriticalSection();
    __serial1_bufferRd = 0;
    __serial1_bufferWr = 0;
	itsdk_leaveCriticalSection();

	__HAL_UART_ENABLE_IT(_uart,UART_IT_ERR);
    __HAL_UART_ENABLE_IT(_uart,UART_IT_RXNE);
    // Clear pending interrupt & co
    // Unclear why we have this, was blocking for STM32L4 device at least
    //HAL_UART_Receive_IT(_uart, __serial1_buffer, 1);
    _uart->Instance->RDR;
    _uart->Instance->ISR;
    _uart->Instance->ICR;
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
		UART_MASK_COMPUTATION(&hlpuart1);
		*ch = hlpuart1.Instance->RDR & hlpuart1.Mask;

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
		UART_MASK_COMPUTATION(&huart1);
		*ch = huart1.Instance->RDR & huart1.Mask;
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
  #if ( ITSDK_WITH_UART & ( __UART_USART1 | __UART_LPUART1 ) ) > 0
	UART_HandleTypeDef * lhuart;
	#if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
		lhuart = &hlpuart1;
	#elif ( ITSDK_WITH_UART & __UART_USART1 ) > 0
		lhuart = &huart1;
	#else
		return BOOL_FALSE;
	#endif
	switch( bd ) {
		case SERIAL_SPEED_300 : lhuart->Init.BaudRate = 300; break;
		case SERIAL_SPEED_600 : lhuart->Init.BaudRate = 600; break;
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
  #else
	return BOOL_FALSE;
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
    __HAL_UART_DISABLE_IT(&huart2,UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&huart2,UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart2,UART_IT_TC);
    __HAL_UART_DISABLE_IT(&huart2,UART_IT_TXE);
	itsdk_enterCriticalSection();
    __serial2_bufferRd = 0;
    __serial2_bufferWr = 0;
	itsdk_leaveCriticalSection();
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_ERR);
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
    // Unclear why we have this, was blocking for STM32L4 device at least
    //HAL_UART_Receive_IT(&huart2, __serial2_buffer, 1);
    huart2.Instance->RDR;
    huart2.Instance->ISR;
    huart2.Instance->ICR;
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
		UART_MASK_COMPUTATION(&huart2);
		*ch = huart2.Instance->RDR & huart2.Mask;
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
  #if ( ITSDK_WITH_UART_RXIRQ & __UART_USART2 ) > 0
	UART_HandleTypeDef * lhuart;
	#if  ( ITSDK_WITH_UART & __UART_USART2 ) > 0
	   lhuart = &huart2;
	#else
		return BOOL_FALSE;
	#endif
	switch( bd ) {
		case SERIAL_SPEED_300 : lhuart->Init.BaudRate = 300; break;
		case SERIAL_SPEED_600 : lhuart->Init.BaudRate = 600; break;
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
  #else
	return BOOL_FALSE;
  #endif
}


// ---------------------------------------------------------------------------
// serial 3 - is mapped to USART3
// ---------------------------------------------------------------------------

/**
 * Init the Serial 3 extra configurations
 */
void serial3_init() {
#if  ( ITSDK_WITH_UART_RXIRQ & __UART_USART3 ) > 0
    __HAL_UART_DISABLE_IT(&huart3,UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&huart3,UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart3,UART_IT_TC);
    __HAL_UART_DISABLE_IT(&huart3,UART_IT_TXE);
	itsdk_enterCriticalSection();
    __serial3_bufferRd = 0;
    __serial3_bufferWr = 0;
	itsdk_leaveCriticalSection();
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_ERR);
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
    // Unclear why we have this, was blocking for STM32L4 device at least
    //HAL_UART_Receive_IT(&huart3, __serial3_buffer, 1);
    huart3.Instance->RDR;
    huart3.Instance->ISR;
    huart3.Instance->ICR;
#endif
}

/**
 * Connect & configure the serial1
 */
void serial3_connect() {
#if ( ITSDK_WITH_UART & __UART_USART3 ) > 0
	HAL_UART_MspInit(&huart3);
#endif
}

/**
 * Disconnect the serail1 from the pads
 */
void serial3_disconnect() {
#if ( ITSDK_WITH_UART & __UART_USART3 ) > 0
	HAL_UART_MspDeInit(&huart3);
#endif
}

void serial3_flush() {
  #if ( ITSDK_WITH_UART & __UART_USART3 ) > 0
  while((__HAL_UART_GET_FLAG(&huart3, USART_ISR_BUSY)) == SET);
  while((__HAL_UART_GET_FLAG(&huart3, USART_ISR_TC)) == RESET);
  #endif
}

void serial3_print(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_USART3 ) > 0
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg),0xFFFF);
  #endif
}

void serial3_write(uint8_t * bytes,uint16_t len) {
#if ( ITSDK_WITH_UART & __UART_USART3 ) > 0
	HAL_UART_Transmit(&huart3, bytes, len,0xFFFF);
#endif
}

void serial3_println(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_USART3 ) > 0
	serial3_print(msg);
	char * eol = "\r\n";
	HAL_UART_Transmit(&huart3, (uint8_t*)eol, strlen(eol),0xFFFF);
  #endif
}

serial_read_response_e serial3_read(char * ch) {

#if  ( ITSDK_WITH_UART_RXIRQ & __UART_USART3 ) > 0

	if ( __serial3_bufferRd != __serial3_bufferWr ) {
		// char available
		*ch = __serial3_buffer[__serial3_bufferRd];
		itsdk_enterCriticalSection();
		__serial3_bufferRd = (__serial3_bufferRd + 1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1);
		itsdk_leaveCriticalSection();
		if ( __serial3_bufferRd != __serial3_bufferWr ) {
			return SERIAL_READ_PENDING_CHAR;
		} else {
			return SERIAL_READ_SUCCESS;
		}
	} else {
		return SERIAL_READ_NOCHAR;
	}

#else
  #if ( ITSDK_WITH_UART & __UART_USART3 ) > 0

	// buffer overflow
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE);
	}

	// get one of the pending char if some.
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)){
		UART_MASK_COMPUTATION(&huart3);
		*ch = huart3.Instance->RDR & huart3.Mask;
		if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)) {
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
itsdk_bool_e serial3_changeBaudRate(serial_baudrate_e bd) {
  #if ( ITSDK_WITH_UART_RXIRQ & __UART_USART3 ) > 0
	UART_HandleTypeDef * lhuart;
	#if  ( ITSDK_WITH_UART & __UART_USART3 ) > 0
	   lhuart = &huart3;
	#else
		return BOOL_FALSE;
	#endif
	switch( bd ) {
		case SERIAL_SPEED_300 : lhuart->Init.BaudRate = 300; break;
		case SERIAL_SPEED_600 : lhuart->Init.BaudRate = 600; break;
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
	serial3_flush();
	if (HAL_UART_Init(lhuart) != HAL_OK) {
	  return BOOL_FALSE;
	}
	serial3_init();
	return BOOL_TRUE;
  #else
	return BOOL_FALSE;
  #endif
}


// ---------------------------------------------------------------------------
// serial 4 - is mapped to USART4
// ---------------------------------------------------------------------------

/**
 * Init the Serial 4 extra configurations
 */
void serial4_init() {
#if  ( ITSDK_WITH_UART_RXIRQ & __UART_USART4 ) > 0
    __HAL_UART_DISABLE_IT(&huart4,UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&huart4,UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart4,UART_IT_TC);
    __HAL_UART_DISABLE_IT(&huart4,UART_IT_TXE);
	itsdk_enterCriticalSection();
    __serial4_bufferRd = 0;
    __serial4_bufferWr = 0;
	itsdk_leaveCriticalSection();
    __HAL_UART_ENABLE_IT(&huart4,UART_IT_ERR);
    __HAL_UART_ENABLE_IT(&huart4,UART_IT_RXNE);
    // Unclear why we have this, was blocking for STM32L4 device at least
    // to be investigated, without this is blocking when a char is pending and blocking when no char pending...
    //HAL_UART_Receive_IT(&huart4, __serial4_buffer, 1);
    huart4.Instance->RDR;
    huart4.Instance->ISR;
    huart4.Instance->ICR;
#endif
}

/**
 * Connect & configure the serial1
 */
void serial4_connect() {
#if ( ITSDK_WITH_UART & __UART_USART4 ) > 0
	HAL_UART_MspInit(&huart4);
#endif
}

/**
 * Disconnect the serail1 from the pads
 */
void serial4_disconnect() {
#if ( ITSDK_WITH_UART & __UART_USART4 ) > 0
	HAL_UART_MspDeInit(&huart4);
#endif
}

void serial4_flush() {
  #if ( ITSDK_WITH_UART & __UART_USART4 ) > 0
  while((__HAL_UART_GET_FLAG(&huart4, USART_ISR_BUSY)) == SET);
  while((__HAL_UART_GET_FLAG(&huart4, USART_ISR_TC)) == RESET);
  #endif
}

void serial4_print(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_USART4 ) > 0
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg),0xFFFF);
  #endif
}

void serial4_write(uint8_t * bytes,uint16_t len) {
#if ( ITSDK_WITH_UART & __UART_USART4 ) > 0
	HAL_UART_Transmit(&huart4, bytes, len,0xFFFF);
#endif
}

void serial4_println(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_USART4 ) > 0
	serial4_print(msg);
	char * eol = "\r\n";
	HAL_UART_Transmit(&huart4, (uint8_t*)eol, strlen(eol),0xFFFF);
  #endif
}

serial_read_response_e serial4_read(char * ch) {

#if  ( ITSDK_WITH_UART_RXIRQ & __UART_USART4 ) > 0

	if ( __serial4_bufferRd != __serial4_bufferWr ) {
		// char available
		*ch = __serial4_buffer[__serial4_bufferRd];
		itsdk_enterCriticalSection();
		__serial4_bufferRd = (__serial4_bufferRd + 1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1);
		itsdk_leaveCriticalSection();
		if ( __serial4_bufferRd != __serial4_bufferWr ) {
			return SERIAL_READ_PENDING_CHAR;
		} else {
			return SERIAL_READ_SUCCESS;
		}
	} else {
		return SERIAL_READ_NOCHAR;
	}

#else
  #if ( ITSDK_WITH_UART & __UART_USART4 ) > 0

	// buffer overflow
	if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_ORE);
	}

	// get one of the pending char if some.
	if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE)){
		UART_MASK_COMPUTATION(&huart4);
		*ch = huart4.Instance->RDR & huart4.Mask;
		if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE)) {
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
itsdk_bool_e serial4_changeBaudRate(serial_baudrate_e bd) {
  #if ( ITSDK_WITH_UART_RXIRQ & __UART_USART4 ) > 0
	UART_HandleTypeDef * lhuart;
	#if  ( ITSDK_WITH_UART & __UART_USART4 ) > 0
	   lhuart = &huart4;
	#else
		return BOOL_FALSE;
	#endif
	switch( bd ) {
		case SERIAL_SPEED_300 : lhuart->Init.BaudRate = 300; break;
		case SERIAL_SPEED_600 : lhuart->Init.BaudRate = 600; break;
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
	serial4_flush();
	if (HAL_UART_Init(lhuart) != HAL_OK) {
	  return BOOL_FALSE;
	}
	serial4_init();
	return BOOL_TRUE;
  #else
	return BOOL_FALSE;
  #endif
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
		#if ( ITSDK_WITH_UART & __UART_USART2 ) > 0
		} else if ( huart->Instance == USART2 ) {
			#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART2 ) > 0
				// at this point the data is in __serial2_buffer[__serial2_bufferWr]
				if ( ((__serial2_bufferWr+1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1)) != __serial2_bufferRd  ) {
					__serial2_bufferWr = ((__serial2_bufferWr+1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1));
				}
				HAL_UART_Receive_IT(huart, &__serial2_buffer[__serial2_bufferWr], 1);
			#endif
		#endif
		#if ( ITSDK_WITH_UART & __UART_USART3 ) > 0
		} else if ( huart->Instance == USART3 ) {
			#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART3 ) > 0
				// at this point the data is in __serial3_buffer[__serial3_bufferWr]
				if ( ((__serial3_bufferWr+1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1)) != __serial3_bufferRd  ) {
					__serial3_bufferWr = ((__serial3_bufferWr+1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1));
				}
				HAL_UART_Receive_IT(huart, &__serial3_buffer[__serial3_bufferWr], 1);
			#endif
		#endif
		#if ( ITSDK_WITH_UART & __UART_USART4 ) > 0
		} else if ( huart->Instance == UART4 ) {
			#if ( ITSDK_WITH_UART_RXIRQ & __UART_USART4 ) > 0
				// at this point the data is in __serial4_buffer[__serial4_bufferWr]
				if ( ((__serial4_bufferWr+1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1)) != __serial4_bufferRd  ) {
					__serial4_bufferWr = ((__serial4_bufferWr+1) & (ITSDK_WITH_UART_RXIRQ_BUFSZ-1));
				}
				HAL_UART_Receive_IT(huart, &__serial4_buffer[__serial4_bufferWr], 1);
			#endif
		#endif
		} else {
			// default case, get it and drop it
			uint8_t c;
			HAL_UART_Receive_IT(huart, &c, 1);
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

#endif // ITSDK_PLATFORM == __PLATFORM_STM32L0 || ITSDK_PLATFORM == __PLATFORM_STM32L4 || ITSDK_PLATFORM == __PLATFORM_STM32WLE

