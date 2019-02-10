/* ==========================================================
 * console.c -  debug & config console
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 9 févr. 2019
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
#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <string.h>

#include <it_sdk/config.h>
#if ITSDK_WITH_CONSOLE == __ENABLE
#include <it_sdk/console/console.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/time/time.h>
#if ITSDK_WITH_SECURESTORE == __ENABLE
#include <it_sdk/eeprom/securestore.h>
#endif

static itsdk_console_state_t __console = { 0 };

static void _itsdk_console_printf(char *format, ...) {
	va_list args;
	char 	fmtBuffer[LOGGER_MAX_BUF_SZ]; 				// buffer for log line formating before printing
    va_start(args,format);
	vsnprintf(fmtBuffer,LOGGER_MAX_BUF_SZ,format,args);
	va_end(args);
#if ( ITSDK_CONSOLE_SERIAL & __UART_LPUART1 ) > 0
	serial1_print(fmtBuffer);
#endif
#if ( ITSDK_CONSOLE_SERIAL & __UART_USART2 ) > 0
	serial2_print(fmtBuffer);
#endif
}


static void _itsdk_console_processLine() {

	if ( __console.loginState == 0 ) {
		// console locked

		// We are going to remove the possible \r and create a 16B array with leading 0 to match with
		// the console password field in Secure Store
		// Password max size is 15 byte.
		if ( __console.pBuffer < 16 ) {
			if ( __console.pBuffer > 0 && __console.serialBuffer[__console.pBuffer-1] == '\r' ) {
				__console.pBuffer--;
			}
			for ( int i = __console.pBuffer ; i < 16 ; i++) {
				__console.serialBuffer[i] = 0;
			}
			// @TODO - Test the version w/o securestore
			 __console.loginState=1;
			#if ITSDK_WITH_SECURESTORE == __DISABLE
				uint8_t passwd[16] = ITSDK_SECSTORE_CONSOLEKEY;
			#else
				uint8_t passwd[16];
				itsdk_secstore_readBlock(ITSDK_SS_CONSOLEKEY, passwd);
			#endif
				for ( int i = 0 ; i < 16 ; i++) {
					if (__console.serialBuffer[i] != passwd[i] && __console.loginState == 1) __console.loginState=0;
				}
				bzero(passwd,16);
		}
		if ( __console.loginState == 1 ) {
			uint64_t s = itsdk_time_get_ms()/1000;
			__console.expire = (uint32_t)s + ITSDK_CONSOLE_EXPIRE_S;
			_itsdk_console_printf("OK\r\n");
		} else {
			_itsdk_console_printf("password:\r\n");
		}
	} else {
		if (__console.pBuffer > 0) {
			// We are logged


		}
	}

}

static void _itsdk_console_processChar(char c) {

	if ( c == '\n' || c == '\r' ) {
		_itsdk_console_processLine();
		__console.pBuffer = 0;
	} else {
		if ( __console.pBuffer < ITSDK_CONSOLE_LINEBUFFER ) {
			__console.serialBuffer[__console.pBuffer] = c;
			__console.pBuffer++;
		}
	}

}

/**
 * This function is call on every wake-up to proceed the pending characters on the serial
 * port and call the associated services.
 */
void itsdk_console_loop() {

	char c;
	serial_read_response_e r;

	// Check the expiration
	if ( __console.loginState == 1 ) {
		uint64_t s = itsdk_time_get_ms()/1000;
		if ( __console.expire < s ) {
			 __console.loginState = 0;
			 _itsdk_console_printf("logout\r\n");
		}
	}

  #if ( ITSDK_CONSOLE_SERIAL & __UART_LPUART1 ) > 0
	do {
		 r = serial1_read(&c);
		 if ( r == SERIAL_READ_SUCCESS || r == SERIAL_READ_PENDING_CHAR) {
			 _itsdk_console_processChar(c);
		 }
	} while ( r == SERIAL_READ_PENDING_CHAR );
  #endif
  #if ( ITSDK_CONSOLE_SERIAL & __UART_USART2 ) > 0
	do {
		 r = serial2_read(&c);
		 if ( r == SERIAL_READ_SUCCESS || r == SERIAL_READ_PENDING_CHAR) {
			 _itsdk_console_processChar(c);
		 }
	} while ( r == SERIAL_READ_PENDING_CHAR );
  #endif

}


#endif // ITSDK_WITH_CONSOLE

