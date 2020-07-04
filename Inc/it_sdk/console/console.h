/* ==========================================================
 * console.h - debug & config console
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 9 f�vr. 2019
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
 *
 * ==========================================================
 */

#ifndef IT_SDK_CONSOLE_CONSOLE_H_
#define IT_SDK_CONSOLE_CONSOLE_H_

#include <stdbool.h>

#include <it_sdk/config.h>
#if ITSDK_WITH_CONSOLE == __ENABLE

#if ITSDK_CONSOLE_LINEBUFFER > 127
#error "ITSDK_CONSOLE_LINEBUFFER is too large, pBuffer is limited to 7 bits."
#endif

typedef enum {
	ITSDK_CONSOLE_SUCCES = 0,								// console command proceed with success
	ITSDK_CONSOLE_NOTFOUND,									// console command corresponding to no operation
	ITSDK_CONSOLE_FAILED									// console command corresponding to an operation but failed
} itsdk_console_return_e;

typedef struct itsdk_console_chain_s {
	itsdk_console_return_e (*console_private)(char * buffer, uint8_t sz);	// function to proceed operations when console is unlocked
	itsdk_console_return_e (*console_public)(char * buffer, uint8_t sz);	// function to proceed operation whatever
	struct itsdk_console_chain_s * next;									// next in chain
} itsdk_console_chain_t;


typedef struct {
	uint8_t		loginState:1;								// 0 when the console is locked / 1 when the console is unlocked
	uint8_t		pBuffer:7;									// index in the Reception buffer current element
	uint8_t		serialBuffer[ITSDK_CONSOLE_LINEBUFFER];		// Reception buffer
	uint32_t	expire;										// expiration time in S



} itsdk_console_state_t;

// ==============================================================================
// API
// ==============================================================================
void itsdk_console_setup();
void itsdk_console_loop();
void itsdk_console_registerCommand(itsdk_console_chain_t * chain);
void itsdk_console_removeCommand(itsdk_console_chain_t * chain);
bool itsdk_console_existCommand(itsdk_console_chain_t * chain);
void _itsdk_console_printf(char *format, ...);

#if ( ITSDK_CONSOLE_SERIAL & __UART_CUSTOM ) > 0
void itsdk_console_customSerial_print(char * msg);
serial_read_response_e itsdk_console_customSerial_read(char * ch);	// for sync char reception
void itsdk_console_customProcess_char(char c);						// for async char reception
#endif

// ==============================================================================
// INTERNAL
// ==============================================================================


#endif // ITSDK_WITH_CONSOLE
#endif /* IT_SDK_CONSOLE_CONSOLE_H_ */
