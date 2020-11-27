/* ==========================================================
 * console.c -  debug & config console
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
#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <string.h>
#include <stdbool.h>


#include <it_sdk/config.h>
#if ITSDK_WITH_CONSOLE == __ENABLE
#include <it_sdk/itsdk.h>
#include <it_sdk/console/console.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/time/time.h>
#include <it_sdk/lowpower/lowpower.h>
#include <it_sdk/eeprom/sdk_state.h>
#include <it_sdk/eeprom/eeprom.h>
#if ITSDK_WITH_SECURESTORE == __ENABLE
#include <it_sdk/eeprom/securestore.h>
#endif
#if ITSDK_RADIO_CERTIF == __ENABLE && (ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE )
#include <it_sdk/radio/certification.h>
#endif

static itsdk_console_state_t __console;
static itsdk_console_chain_t __console_head_chain;

static void _itsdk_console_processLine();
static void _itsdk_console_processChar(char c);

/**
 * Dafault Operation
 */

static itsdk_console_return_e _itsdk_console_private(char * buffer, uint8_t sz) {

	if ( sz == 1 ) {
		switch ( buffer[0] ) {
		case '?':
			// help
			_itsdk_console_printf("X          : exit console\r\n");
			_itsdk_console_printf("R          : reset device\r\n");
			_itsdk_console_printf("R!         : clear the whole eeprom\r\n");
			_itsdk_console_printf("l / L      : switch LowPower ON / OFF\r\n");
			_itsdk_console_printf("s          : print device state\r\n");
			_itsdk_console_printf("t          : print current time in S\r\n");
#if ITSDK_WITH_ADC != __ADC_NONE
			_itsdk_console_printf("T          : print current cpu temperature in oC\r\n");
			_itsdk_console_printf("b          : print battery level\r\n");
			_itsdk_console_printf("B          : print VCC level\r\n");
#endif
			_itsdk_console_printf("r          : print last Reset Cause\r\n");

#if ITSDK_RADIO_CERTIF == __ENABLE && (ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE )
			_itsdk_console_printf("c:0:nnn    : CW for CE tests with power\r\n");
			_itsdk_console_printf("c:1:nnn    : CW for EU Sigfox tests with power\r\n");
#endif

			return ITSDK_CONSOLE_SUCCES;
		case 'X':
			// exit console
			__console.loginState=0;
			goto success;
		case 't':
			// print time
			_itsdk_console_printf("Run time is %d s\r\n",(uint32_t)(itsdk_time_get_ms()/1000L));
			goto success;
#if ITSDK_WITH_ADC != __ADC_NONE
		case 'T':
			// print temperature
			{
			uint16_t t = adc_getTemperature();
			_itsdk_console_printf("Temperature is %d.%doC\r\n",t/100,t-((t/100)*100));
			goto success;
			}
		case 'b':
			// battery level
			_itsdk_console_printf("Battery level %dmV\r\n",(uint32_t)(adc_getVBat()));
			goto success;
		case 'B':
			// Vcc level
			_itsdk_console_printf("VCC level %dmV\r\n",(uint32_t)(adc_getVdd()));
			goto success;
#endif
		case 'r':
			// Last Reset cause
			_itsdk_console_printf("Reset: ");
			switch(itsdk_state.lastResetCause) {
			case RESET_CAUSE_BOR: _itsdk_console_printf("BOR\r\n"); break;
			case RESET_CAUSE_RESET_PIN: _itsdk_console_printf("RESET PIN\r\n"); break;
			case RESET_CAUSE_POWER_ON: _itsdk_console_printf("POWER ON\r\n"); break;
			case RESET_CAUSE_SOFTWARE: _itsdk_console_printf("SOFT\r\n"); break;
			case RESET_CAUSE_IWDG: _itsdk_console_printf("IWDG\r\n"); break;
			case RESET_CAUSE_WWDG: _itsdk_console_printf("WWDG\r\n"); break;
			case RESET_CAUSE_LOWPOWER: _itsdk_console_printf("LOW POWER"); break;
			default:
				_itsdk_console_printf("UNKNOWN\r\n"); break;
			}
			goto success;
		case 'R':
			// Reset device
			_itsdk_console_printf("OK\r\n");
			itsdk_reset();
			_itsdk_console_printf("KO\r\n");			// never reached...
			return ITSDK_CONSOLE_FAILED;
		case 'l':
			// switch lowPower On
			lowPower_enable();
			goto success;
		case 'L':
			// switch LowPower Off
			lowPower_disable();
			goto success;
		}
	} else if (sz==2) {
		if ( buffer[0] == 'R' && buffer[1] == '!' ) {
			// Clear all the eeprom content the reset - hard factory default
			_itsdk_console_printf("OK\r\n");
			eeprom_clearAllEprom();
			itsdk_delayMs(100);
			itsdk_reset();
			return ITSDK_CONSOLE_FAILED;
		}
	}
#if ITSDK_RADIO_CERTIF == __ENABLE && (ITSDK_WITH_SIGFOX_LIB == __ENABLE || ITSDK_WITH_LORAWAN_LIB == __ENABLE )
	  else if ( sz==7 ) {
		if ( buffer[0] == 'c' && buffer[1] == ':' && buffer[3] == ':' ) {
		 int power = itdt_convertDecChar3UInt(&buffer[4]);
		 if ( power == ITSDK_INVALID_VALUE_16B ) goto failed;
		 if ( buffer[2] == '0' ) {
			 // CE certification, frequency 868.100.000MHz
			 if ( startContinousWaveTransmission( 868100000,power,0 ) == BOOL_FALSE ) goto failed;
			 goto success;
		 } else if ( buffer[2] == '1' ) {
			 // Sigfox RC1 certification, frequency 868.130.000MHz
			 if ( startContinousWaveTransmission( 868130000,power,0 ) == BOOL_FALSE ) goto failed;
			 goto success;
		 } else goto failed;
		}
	}
#endif
	return ITSDK_CONSOLE_NOTFOUND;

success:
	_itsdk_console_printf("OK\r\n");
	return ITSDK_CONSOLE_SUCCES;
failed:
	_itsdk_console_printf("KO\r\n");
	return ITSDK_CONSOLE_FAILED;
}

static itsdk_console_return_e _itsdk_console_public(char * buffer, uint8_t sz) {

	if ( sz == 1 ) {
		switch ( buffer[0] ) {
		case '?':
			// help
			_itsdk_console_printf("--- Common\r\n");
			_itsdk_console_printf("?          : print help\r\n");
			_itsdk_console_printf("!          : print copyright\r\n");
			_itsdk_console_printf("v          : print version\r\n");
			_itsdk_console_printf("o          : print OK\r\n");
			return ITSDK_CONSOLE_SUCCES;
			break;
		case 'o':
			_itsdk_console_printf("OK\r\n");
			return ITSDK_CONSOLE_SUCCES;
			break;
		case '!':
			// Copyright
			_itsdk_console_printf("IT_SDK - (c) 2020 - Paul Pinault aka Disk91\r\n");
			_itsdk_console_printf(ITSKD_CONSOLE_COPYRIGHT);
			return ITSDK_CONSOLE_SUCCES;
			break;
		case 'v':
			// Version
			_itsdk_console_printf("FW Version %s\r\n",ITSDK_USER_VERSION);
			_itsdk_console_printf("Build %s %s\r\n",__DATE__, __TIME__);
			_itsdk_console_printf("IT_SDK Version %s\r\n",ITSDK_VERSION);
			return ITSDK_CONSOLE_SUCCES;
			break;
		case 's':
			// State
			itsdk_print_state();
			return ITSDK_CONSOLE_SUCCES;
			break;
		}
	}
	return ITSDK_CONSOLE_NOTFOUND;
}



/**
 * Setup the console & associated chain
 */
void itsdk_console_setup() {
	__console.expire = 0;
	__console.loginState = 0;
	__console.pBuffer = 0;
	__console_head_chain.console_private = _itsdk_console_private;
	__console_head_chain.console_public = _itsdk_console_public;
	__console_head_chain.next = NULL;
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

  #if ( ITSDK_CONSOLE_SERIAL & ( __UART_LPUART1 | __UART_USART1 ) ) > 0
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
  #if ( ITSDK_CONSOLE_SERIAL & __UART_CUSTOM ) > 0
	do {
		 r = itsdk_console_customSerial_read(&c);
		 if ( r == SERIAL_READ_SUCCESS || r == SERIAL_READ_PENDING_CHAR) {
			 _itsdk_console_processChar(c);
		 }
	} while ( r == SERIAL_READ_PENDING_CHAR );
  #endif

}

// =================================================================================================
// Processing output
// =================================================================================================

#if ( ITSDK_CONSOLE_SERIAL & __UART_CUSTOM ) > 0
__weak void itsdk_console_customSerial_print(char * msg) {
	return;
}
__weak serial_read_response_e itsdk_console_customSerial_read(char * ch) {
	return SERIAL_READ_NOCHAR;
}
void itsdk_console_customProcess_char(char c) {
	_itsdk_console_processChar(c);
}
#endif

void _itsdk_console_printf(char *format, ...) {
	va_list args;
	char 	fmtBuffer[LOGGER_MAX_BUF_SZ]; 				// buffer for log line formating before printing
    va_start(args,format);
	vsnprintf(fmtBuffer,LOGGER_MAX_BUF_SZ,format,args);
	va_end(args);
#if ( ITSDK_CONSOLE_SERIAL & ( __UART_LPUART1 | __UART_USART1 ) ) > 0
	serial1_print(fmtBuffer);
#endif
#if ( ITSDK_CONSOLE_SERIAL & __UART_USART2 ) > 0
	serial2_print(fmtBuffer);
#endif
#if ( ITSDK_CONSOLE_SERIAL & __UART_CUSTOM ) > 0
	itsdk_console_customSerial_print(fmtBuffer);
#endif
}

// =================================================================================================
// Processing input
// =================================================================================================

static void _itsdk_console_processLine() {

	// Empty line
	if ( __console.pBuffer == 0 ) return;

	// Clean the buffer
	if ( __console.pBuffer > 0 && __console.serialBuffer[__console.pBuffer-1] == '\r' ) {
		__console.pBuffer--;
	}
	for ( int i = __console.pBuffer ; i < ITSDK_CONSOLE_LINEBUFFER ; i++) {
		__console.serialBuffer[i] = 0;
	}

	if ( __console.loginState == 0 ) {
		// console locked

		// We are going to remove the possible \r and create a 16B array with leading 0 to match with
		// the console password field in Secure Store
		// Password max size is 15 byte.
		if ( __console.pBuffer < 16 ) {
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
			// Login sucess
			uint64_t s = itsdk_time_get_ms()/1000;
			__console.expire = (uint32_t)s + ITSDK_CONSOLE_EXPIRE_S;
			_itsdk_console_printf("OK\r\n");
		} else {
			// Login Failed This can be a public operation request
			itsdk_console_chain_t * c = &__console_head_chain;
			itsdk_console_return_e  ret = ITSDK_CONSOLE_NOTFOUND;
			itsdk_console_return_e  lret;
			while ( c != NULL ) {
				if ( c->console_public != NULL ) {
  				   lret= c->console_public((char*)__console.serialBuffer,(uint8_t)__console.pBuffer);
				   switch ( lret ) {
					  case ITSDK_CONSOLE_SUCCES:
					  case ITSDK_CONSOLE_FAILED:
						  ret = ITSDK_CONSOLE_SUCCES;
						  break;
					  default:
					      break;
				   }
				}
			   c = c->next;
			}
			// Print the password prompt only when it was not a command
			if ( ret == ITSDK_CONSOLE_NOTFOUND ) {
				_itsdk_console_printf("password:\r\n");
				_itsdk_console_printf("KO\r\n");
			}
		}
	} else {
		if (__console.pBuffer > 0) {
			// We are logged

			// Update session expiration
			uint64_t s = itsdk_time_get_ms()/1000;
			__console.expire = (uint32_t)s + ITSDK_CONSOLE_EXPIRE_S;

			// Process command
			itsdk_console_chain_t * c = &__console_head_chain;
			itsdk_console_return_e  ret = ITSDK_CONSOLE_NOTFOUND;
			itsdk_console_return_e  lret;
			while ( c != NULL ) {
			  if ( c->console_public != NULL ) {
				  lret = c->console_public((char*)__console.serialBuffer,(uint8_t)__console.pBuffer);
				  switch ( lret ) {
					  case ITSDK_CONSOLE_SUCCES:
					  case ITSDK_CONSOLE_FAILED:
						  ret = ITSDK_CONSOLE_SUCCES;
						  break;
					  default:break;
				   }
			  }
			  if ( c->console_private != NULL ) {
				  lret = c->console_private((char*)__console.serialBuffer,(uint8_t)__console.pBuffer);
				  switch ( lret ) {
					  case ITSDK_CONSOLE_SUCCES:
					  case ITSDK_CONSOLE_FAILED:
						  ret = ITSDK_CONSOLE_SUCCES;
						  break;
					  default:break;
				   }
			  }
			  c = c->next;
			}
			if ( ret == ITSDK_CONSOLE_NOTFOUND ) {
				_itsdk_console_printf("KO\r\n");
			}
		}
	}

}

/**
 * Process 1 char read
 */
static void _itsdk_console_processChar(char c) {

	if ( c == '\n' || c == '\r' || c == '\0' ) {
		if ( __console.pBuffer > 0 ) {
//			log_info("RET");
			_itsdk_console_processLine();
			__console.pBuffer = 0;
		}
//		log_info("ESC");
	} else {
		if ( __console.pBuffer < ITSDK_CONSOLE_LINEBUFFER ) {

//			if ( c > 32 ) {
//			  log_info("[%c]",c);
//			} else log_info("(%02X)",c);

			__console.serialBuffer[__console.pBuffer] = c;
			__console.pBuffer++;
		}
	}

}


// =================================================================================================
// Console operation chain management
// =================================================================================================


/**
 * Add an action to the chain, the action **must be** static
 * The action list is added at end of the chain
 */
void itsdk_console_registerCommand(itsdk_console_chain_t * chain) {
	itsdk_console_chain_t * c = &__console_head_chain;
	if ( c->console_private != _itsdk_console_private ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_CONSOLE_NOTSETUP,0);
	}

	while ( c->next != NULL && c->next != chain ) {
	  c = c->next;
	}
	if ( c->next != chain ) {
		// the Action is not already existing
		c->next=chain;
		chain->next = NULL;
	}
}

/**
 * Remove an action to the chain, the action **must be** static
 */
void itsdk_console_removeCommand(itsdk_console_chain_t * chain) {
	itsdk_console_chain_t * c = &__console_head_chain;
	while ( c != NULL && c->next != chain ) {
	  c = c->next;
	}
	if ( c != NULL ) {
		c->next = c->next->next;
	}
}

/**
 * Search for an existing action
 */
bool itsdk_console_existCommand(itsdk_console_chain_t * chain) {
	itsdk_console_chain_t * c = &__console_head_chain;
	while ( c != NULL && c->next != chain ) {
	  c = c->next;
	}
	if ( c != NULL ) {
		return true;
	}
	return false;
}



#endif // ITSDK_WITH_CONSOLE

