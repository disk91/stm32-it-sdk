/* ==========================================================
 * uart_wrapper.c - wrapper function for uarts
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 12 sept. 2018
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
 * Serial1 is LPUART
 * Serial2 is USART2
 * Debuh is non affected
 *
 * ==========================================================
 */
#include <string.h>
#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0x1

#include <it_sdk/wrappers.h>
#include "stm32l0xx_hal.h"

// ---------------------------------------------------------------------------
// serial 1 - is mapped to LPUART1
// ---------------------------------------------------------------------------

/**
 * flushing pending transmission
 */
void serial1_flush() {
  #if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
     while(__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_BUSY) == SET);
  #endif
}

void serial1_print(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg),0xFFFF);
  #endif
}

void serial1_println(char * msg) {
  #if ( ITSDK_WITH_UART & __UART_LPUART1 ) > 0
	serial1_print(msg);
	char * eol = "\r\n";
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)eol, strlen(eol),0xFFFF);
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


// ---------------------------------------------------------------------------
// debug - is not mapped
// ---------------------------------------------------------------------------

void debug_flush() {
}

void debug_print(char * msg) {
}


#endif
