/* ==========================================================
 * em2050.h - implementation echostar EM2050 module
 * ----------------------------------------------------------
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
 * ---------------------------------------------------------
 *
 *  Created on: 29 November 2024
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2024
 * ==========================================================
 */

#include "it_sdk/config.h"
#if ITSDK_DRIVERS_EM2050 == __ENABLE
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "drivers/echostar/em2050.h"
#include "it_sdk/time/time.h"
#include "it_sdk/logger/logger.h"


typedef enum {
	R_CONTINUE = 0,			// next line to process expected
	R_EXIT_SUCCESS = 1,		// no new line expected, exit immediately
	R_EXIT_FAILURE = 2,		// unexpected response, exit immediately with error reporting
	R_CONTINUE_FAILURE = 3,	// unexpected response, continue until the end to clear the buffer (continue to callback for each lines)
	R_NONE = 5,				// this to indicate no decision cna be take, (used by subfunction)

	R_END
} e_lineFuncRet;

e_lineFuncRet processAtLineJustPrint(char * r);
e_lineFuncRet processEsModemBoot(char * r);
itsdk_bool_e sendAtCommand(char * cmd, uint64_t initTmout, uint64_t endTmout, e_lineFuncRet (*lineProcess)(char * line), itsdk_bool_e clear );
uint8_t __es_boot_line = 0;

// ===========================================
// UART Integration
// ===========================================

/**
 * You can create a custom serial adapter by overriding these function in the
 * application code
 */
#if ITSDK_DRIVERS_EM2050_SERIAL == __UART_CUSTOM
__weak void em2050_customSerial_write(uint8_t * bytes,uint16_t len) {
	return;
}
__weak void em2050_customSerial_println(char * msg) {
	return;
}
__weak serial_read_response_e em2050_customSerial_read(char * ch) {
	return SERIAL_READ_NOCHAR;
}
__weak void em2050_customSerialInit() {
	return;
}
__weak void em2050_customSerialConnect() {
	return;
}
#endif


// keep current modem state, 0 for sleep / 1 for active
static uint8_t __echostar_sleep_state;

static void __es_initGpio(itsdk_bool_e on) {

	#if (ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART1 || ITSDK_DRIVERS_EM2050_SERIAL == __UART_LPUART1 )
		#if ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART1
			MX_USART1_UART_Init();
		#else
			MX_LPUART1_UART_Init();
		#endif
		serial1_init();
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART2
		MX_USART2_UART_Init();
		serial2_init();
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART3
		MX_USART3_UART_Init();
		serial3_init();
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART4
		MX_UART4_Init();
		serial4_init();
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_CUSTOM
		em2050_customSerialInit();
	#else
		#error "ITSDK_DRIVERS_EM2050_SERIAL is not set"
	#endif

	if ( (ITSDK_DRIVERS_EM2050_GNSS_LNA_ON_PIN) != __LP_GPIO_NONE) {
		gpio_configure(ITSDK_DRIVERS_EM2050_GNSS_LNA_ON_PORT, ITSDK_DRIVERS_EM2050_GNSS_LNA_ON_PIN, GPIO_OUTPUT_PP );
	}

	if ( (ITSDK_DRIVERS_EM2050_RESET_PIN) != __LP_GPIO_NONE) {
		gpio_configure(ITSDK_DRIVERS_EM2050_RESET_PORT, ITSDK_DRIVERS_EM2050_RESET_PIN, GPIO_OUTPUT_PP );
		gpio_set(ITSDK_DRIVERS_EM2050_RESET_PORT,ITSDK_DRIVERS_EM2050_RESET_PIN);
	}

	if ( on ) {
		__echostar_sleep_state = 0; // make sure we activate it
		echoStarSwitchActive();
		echoStarWakeUpModem();
	} else {
		__echostar_sleep_state = 1; // make sure we deactivate it
		echoStarSwitchSleep();
	}

}

// keep current modem state, 0 for sleep / 1 for active
static uint8_t __echostar_sleep_state;

// Setup RTS to indicate the MCU is active mode
void echoStarSwitchActive() {
	// Doc says
	// RTS is a signal from MCU to Module. It must be configured as an open-drain I/O with internal
	// pullup.
	// When MCU is active & ready to receive data, RTS must be driven to LOW and the pullup may be switched off to
	// reduce power consumption.
	//
	if ( (ITSDK_DRIVERS_EM2050_RTS_PIN) != (__LP_GPIO_NONE) ) {
		if ( ! __echostar_sleep_state ) {
			gpio_configure(ITSDK_DRIVERS_EM2050_RTS_PORT, ITSDK_DRIVERS_EM2050_RTS_PIN, GPIO_OUTPUT_OD );
			gpio_reset(ITSDK_DRIVERS_EM2050_RTS_PORT,ITSDK_DRIVERS_EM2050_RTS_PORT);
			__echostar_sleep_state = 1;
		}
	}
}


// Setup RTS to indicate the MCU is sleeping mode
void echoStarSwitchSleep() {
	// Doc says
	// When the Application MCU is in sleep mode, signal must be configured as an interrupt-generating input with internal pull up
	// Modem will wake up Application MCU by driving RTS to LOW for 10uS (Falling interrupt) then signal floated
	if ( (ITSDK_DRIVERS_EM2050_RTS_PIN) != (__LP_GPIO_NONE) ) {
		if ( __echostar_sleep_state ) {
			gpio_configure(ITSDK_DRIVERS_EM2050_RTS_PORT, ITSDK_DRIVERS_EM2050_RTS_PIN, GPIO_INPUT_PULLUP );
			gpio_reset(ITSDK_DRIVERS_EM2050_RTS_PORT,ITSDK_DRIVERS_EM2050_RTS_PORT);
			itsdk_delayUs(20);
			__echostar_sleep_state = 0;
		}
	}
}

itsdk_bool_e echoStarWakeUpModem() {
	// Make sure the Modem knows MCU is active
	echoStarSwitchActive();

	// Doc Says
	// CTS is a signal from Modem to MCU it is configured as open-drain with internal pullup on the modem side
	// When modem is active & ready, CTS is driven LOW
	// MCU wake up Modem by driving CTS LOW for at least 10uS, then signal must be floating and will be pullup by Modem and when
	// ready it will be driven to low.
	if ( (ITSDK_DRIVERS_EM2050_CTS_PIN) != __LP_GPIO_NONE) {

		gpio_configure(ITSDK_DRIVERS_EM2050_CTS_PORT, ITSDK_DRIVERS_EM2050_CTS_PIN, GPIO_INPUT );
		if ( gpio_read(ITSDK_DRIVERS_EM2050_CTS_PORT,ITSDK_DRIVERS_EM2050_CTS_PIN) != 0 ) {
			//_LOG_ECHOSTAR_DEBUG((">"));
			// Modem is not active, waking up
			gpio_configure(ITSDK_DRIVERS_EM2050_CTS_PORT, ITSDK_DRIVERS_EM2050_CTS_PIN, GPIO_OUTPUT_PP );
			gpio_reset(ITSDK_DRIVERS_EM2050_CTS_PORT, ITSDK_DRIVERS_EM2050_CTS_PIN);
			itsdk_delayUs(15);
			gpio_configure(ITSDK_DRIVERS_EM2050_CTS_PORT, ITSDK_DRIVERS_EM2050_CTS_PIN, GPIO_INPUT );
			itsdk_delayUs(15);

			// send 0x00 on uart to wake me up
			uint8_t w = 0x00;
			#if (ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART1 || ITSDK_DRIVERS_EM2050_SERIAL == __UART_LPUART1 )
				serial1_write(&w,1);
			#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART2
				serial2_write(&w,1);
			#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART3
				serial3_write(&w,1);
			#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART4
				serial4_write(&w,1);
			#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_CUSTOM
				em2050_customSerial_write(&w,1);
			#else
				#error "ITSDK_DRIVERS_EM2050_SERIAL is not set"
			#endif


			// make sure we do not loop forever
			uint64_t t = itsdk_time_get_ms();
			while ( gpio_read(ITSDK_DRIVERS_EM2050_CTS_PORT,ITSDK_DRIVERS_EM2050_CTS_PIN) != 0 && (itsdk_time_get_ms() - t) < 100);
			if ( (itsdk_time_get_ms() - t) > 100 ) {
				// timeout
				return BOOL_FALSE;
			} else {
				//_LOG_ECHOSTAR_DEBUG(("<"));
				return BOOL_TRUE;
			}
		}

	}
	return BOOL_TRUE;
}


//
// read pending char on UART
//
static serial_read_response_e __es_read(char * ch)  {
	#if (ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART1 || ITSDK_DRIVERS_EM2050_SERIAL == __UART_LPUART1 )
		return serial1_read(ch);
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART2
		return serial2_read(ch);
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART3
		return serial3_read(ch);
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART4
		return serial4_read(ch);
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_CUSTOM
		return em2050_customSerial_read(ch);
	#endif
}

//
// Clear pending reception line buffer
//
static void __es_clear() {
	char c;
	while ( __es_read(&c ) != SERIAL_READ_NOCHAR );
}

static void __es_println(char * msg) {
	#if (ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART1 || ITSDK_DRIVERS_EM2050_SERIAL == __UART_LPUART1 )
		serial1_println(msg);
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART2
		serial2_println(msg);
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART3
		serial3_println(msg);
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART4
		serial4_println(msg);
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_CUSTOM
		em2050_customSerial_println(msg);
	#endif
}

// ===========================================
// Init
// ===========================================


void echoStarInit() {

	__es_initGpio(BOOL_TRUE);

	#if (ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART1 || ITSDK_DRIVERS_EM2050_SERIAL == __UART_LPUART1 )
		serial1_connect();
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART2
		serial2_connect();
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART3
		serial3_connect();
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_USART4
		serial4_connect();
	#elif ITSDK_DRIVERS_EM2050_SERIAL == __UART_CUSTOM
		em2050_customSerialConnect();
	#endif

	if ( (ITSDK_DRIVERS_EM2050_RESET_PIN) != __LP_GPIO_NONE) {
		// Reset the module and grab the starting messages
		gpio_reset(ITSDK_DRIVERS_EM2050_RESET_PORT,ITSDK_DRIVERS_EM2050_RESET_PIN);
		itsdk_delayMs(100);
		gpio_set(ITSDK_DRIVERS_EM2050_RESET_PORT,ITSDK_DRIVERS_EM2050_RESET_PIN);
	} else {
		#warning "TODO - Need to reset by software"
	}
	// grab the messages and print on console
	__es_boot_line = 0;
	sendAtCommand(NULL,250,5000,processEsModemBoot,BOOL_FALSE);

}

// Init GPIO for running Off
void echoStarInitOff() {
	__es_initGpio(BOOL_FALSE);
}


// ===========================================
// Send an AT command to the modem
// cmd : AT command to send ( do not add the \r\n ), NULL to just receive
// initTmout: in Ms, min wait for first char reception
// endRmout: in Ms, min wait for exiting between chars
// lineProcess: callback function for processing received char
// ===========================================
itsdk_bool_e sendAtCommand(char * cmd, uint64_t initTmout, uint64_t endTmout, e_lineFuncRet (*lineProcess)(char * line),itsdk_bool_e clear ) {
	char c;

	uint16_t __es_bufferWr = 0;
	char __es_buffer[ITSDK_DRIVERS_EM2050_LINEBUFFER];

	if ( clear ) __es_clear();
	if ( cmd != NULL ) __es_println(cmd);
	itsdk_bool_e end = BOOL_FALSE;
	itsdk_bool_e init = BOOL_TRUE;
	uint64_t start = itsdk_time_get_ms();
	while ( ! end ) {
		serial_read_response_e r = __es_read(&c);
		if (r == SERIAL_READ_NOCHAR ) {
			uint64_t t = itsdk_time_get_ms();
			// exit after like 1_000ms until something returned
			if ( init && (t-start) > initTmout ) end = true;
			// exit after like 100ms without an update
			else if ( !init && (t-start) > endTmout ) end = true;
		} else {
			#if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
			   wdg_refresh();
			#endif
			init = false;
			start = itsdk_time_get_ms();
			if ( c != '\r' && c != '\n' ) {
				__es_buffer[__es_bufferWr] = c;
				__es_bufferWr++;
				if ( __es_bufferWr >= (ITSDK_DRIVERS_EM2050_LINEBUFFER-1) ) {
					// out of memory
					_LOG_ECHOSTAR_ERROR(("Serial buffer overflow\r\n"));
					__es_buffer[ITSDK_DRIVERS_EM2050_LINEBUFFER-1] = '\0';
					switch ( lineProcess(__es_buffer) )  {
					  case R_EXIT_SUCCESS:
						  return BOOL_TRUE;
					  case R_EXIT_FAILURE:
						  return BOOL_FALSE;
					  default:
					  case R_CONTINUE:
					  case R_CONTINUE_FAILURE:
						  break;
					}
				}
			} else {
				if ( __es_bufferWr > 0 ) {
					// we have a line to process
					__es_buffer[__es_bufferWr] = '\0';
					switch ( lineProcess(__es_buffer) )  {
					  case R_EXIT_SUCCESS:
						  return BOOL_TRUE;
					  case R_EXIT_FAILURE:
						  return BOOL_FALSE;
					  default:
					  case R_CONTINUE:
					  case R_CONTINUE_FAILURE:
						  break;
					}
					__es_bufferWr = 0;
				}
			}
		}
	}
	// a voir si on process à la fin ou non
	// faire des status sur lineProcess pour une fin detectee
	// pour une erreur ou un success...
	return BOOL_TRUE;
}

// ===========================================
// Verify Serial line
// ===========================================

// print on console for debugging purpose
e_lineFuncRet processAtLineJustPrint(char * r) {
	_LOG_ECHOSTAR_INFO(("%d # ",strlen(r)));
	_LOG_ECHOSTAR_INFO((r));
	_LOG_ECHOSTAR_INFO(("\r\n"));
	return R_CONTINUE;
}

// When we just wait for an OK
e_lineFuncRet processAtLineExpectOk(char * r) {
	if ( strncmp(r,"OK",2) == 0) return R_EXIT_SUCCESS;
	if ( strncmp(r,"ERROR",5) == 0) return R_EXIT_FAILURE;
	return R_CONTINUE;
}

e_lineFuncRet processDefaultCases(char * r) {
	if ( strncmp(r,"OK",2) == 0) return R_EXIT_SUCCESS;
	if ( strncmp(r,"INFO:",5) == 0 ) return R_CONTINUE;
	if ( strncmp(r,"WARNING:",8) == 0 ) return R_CONTINUE;
	if ( strncmp(r,"ERROR",5) == 0) return R_EXIT_FAILURE;
	return R_NONE;
}

/* This is written on serial line on start, it also written on regular
 * basis ... so if a response starts by INFO: may be other things, better skipp them
 *
 * INFO: Echostar OEM Module Bootloader v1.0, checking FW image integrity
 * INFO: Bootloader FW check passed. Starting...
 * INFO: Echostar Mobile OEM LoRa Module FW1.13 started, cause: [External Reset] @ heure...
 * INFO: LoRaMac stack initialized @ 01/01/1970 00:03:28
 * INFO: LoRaMac stack started with region MSS-S, joining network. @....
 * ( this sounds to be for MSS mode, EU868 may not have it)
 * INFO: Aquiring beacon to join the network @ ...
 * INFO: Starting beacon search on channel 1, 2199500000Hz @ ...
 *
 */
e_lineFuncRet processEsModemBoot(char * r) {
	//_LOG_ECHOSTAR_INFO((r));_LOG_ECHOSTAR_INFO(("\r\n"));
	if ( strlen(r) > 2 ) {
		__es_boot_line++;
		if ( __es_boot_line < 6 ) return R_CONTINUE;
		else return R_EXIT_SUCCESS;
	} else return R_CONTINUE;
}


e_lineFuncRet processAtLine(char * r) {
	// clear the logs ...
	if ( strncmp(r,"INFO:",5) == 0 ) {
		return R_CONTINUE;
	}
	if ( strncmp(r,"OK",2) == 0) {
		// command success
		return R_EXIT_SUCCESS;
	}
	return R_CONTINUE;
}

itsdk_bool_e isEchoStarModemresponding() {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	return sendAtCommand("AT",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processAtLine,BOOL_TRUE);
}

void echoStarshowMessages() {
	char c;
	serial_read_response_e r;
	uint8_t empty=1;
	do {
		r = __es_read(&c);
		if ( r != SERIAL_READ_NOCHAR ) {
			// filter what to print
			if ( c >= ' ' && c <= '~'  ) {
				empty = 0;
				_LOG_ECHOSTAR_INFO(("%c",c));
			}
			// only print new line when we had text before
			if ( c == '\n' && !empty ) {
				empty = 1;
				_LOG_ECHOSTAR_INFO(("\r\n"));
			}
		}
	} while ( r == SERIAL_READ_PENDING_CHAR );
}


// ======================== Manage Modem Config ==============================

itsdk_bool_e echoStarSoftReset() {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	if ( sendAtCommand("ATZ",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processDefaultCases,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// -------

union _es_config_u {
	uint16_t 		fw_version;
	uint8_t * 		devid;
	uint8_t *   	network_key;
	uint8_t *   	user_key;
	uint8_t *   	joinEui;
	uint8_t *   	pinCode;
	es_joinMode_t 	joinMode;
	es_joinStatus_t	joinStatus;
	es_region_t		region;
	uint8_t			txPower;
	int8_t			antGain;
	struct s_pwsf {
		uint8_t		txPower;
		es_sf_t		sf;
	} 				pwsf;
	es_adr_t		adr;
	uint8_t *		encoded;
	es_class_t		class;
	uint32_t		katime;
};
union _es_config_u _es_config;


// -------

e_lineFuncRet processVersionLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strncmp(r,"Firm",4) == 0) {
		// should be the version line
		if ( strlen(r) == 21 ) {
			_es_config.fw_version = (r[18]-'0') * 256 + (r[20]-'0');
		} else if ( strlen(r) > 21 ) {
			_es_config.fw_version = (r[18]-'0') * 256 + (r[20]-'0')*10 + (r[21]-'0');
		} else {
			_LOG_ECHOSTAR_ERROR(("ES - invalid version response\r\n"));
			_LOG_ECHOSTAR_ERROR((r));_LOG_ECHOSTAR_ERROR(("\r\n"));
		}
	}
	return R_CONTINUE;
}


// Return the firmware version with 8MSB bits with major and 8LSB bits miner
uint16_t echoStarGetVersion() {
	_es_config.fw_version = 0;
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	if ( sendAtCommand("ATI",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processVersionLine,BOOL_TRUE) == BOOL_TRUE ) {
		return _es_config.fw_version;
	} else return 0;
}

// -------

e_lineFuncRet processATVDeviceIdLine(char * r) {
	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 51 && strncmp(r,"Device ID",9) == 0) {
		// Device ID (Device EUI): DI: 00:16:C0:01:F0:10:01:60
		char * c = &r[28];
		for ( int i = 0 ; i < 8 ; i++ ) {
			_es_config.devid[i] = itdt_convertHexChar2Int(c);
			c+=3;
		}
	}
	return R_CONTINUE;
}

e_lineFuncRet processDeviceIdLine(char * r) {
	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 27 && strncmp(r,"DI:",3) == 0) {
		// DI: 00:16:C0:01:F0:10:01:60
		char * c = &r[4];
		for ( int i = 0 ; i < 8 ; i++ ) {
			_es_config.devid[i] = itdt_convertHexChar2Int(c);
			c+=3;
		}
	}
	return R_CONTINUE;
}

// Return the deviceID as an array
itsdk_bool_e echoStarGetDeviceId(uint8_t * devEui) {
	_es_config.devid = devEui;
	bzero(_es_config.devid,8);
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	if ( sendAtCommand("AT+DI?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processDeviceIdLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}


// -------

e_lineFuncRet processNtwKeyLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 27 && strncmp(r,"NK:",3) == 0) {
		// DI: 00:16:C0:01:F0:10:01:60
		char * c = &r[4];
		for ( int i = 0 ; i < 16 ; i++ ) {
			_es_config.network_key[i] = itdt_convertHexChar2Int(c);
			c+=3;
		}
	}
	return R_CONTINUE;
}

// Return the NetworkKey as an array
itsdk_bool_e echoStarGetNetworkKey(uint8_t * key) {
	bzero(key,16);
	_es_config.network_key = key;
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	if ( sendAtCommand("AT+NK?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processNtwKeyLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// Set the NetworkKey as an array, if null, send 0xFF, this reset the key to original version
itsdk_bool_e echoStarSetNetworkKey(uint8_t * ntwKey) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	char cmd[64];
	if ( ntwKey != NULL ) {
		sprintf(cmd,"AT+NK=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
				ntwKey[0],ntwKey[1],ntwKey[2],ntwKey[3],
				ntwKey[4],ntwKey[5],ntwKey[6],ntwKey[7],
				ntwKey[8],ntwKey[9],ntwKey[10],ntwKey[11],
				ntwKey[12],ntwKey[13],ntwKey[14],ntwKey[15]
		);
	} else {
		sprintf(cmd,"AT+NK=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
				0xFF, 0xFF, 0xFF, 0xFF,
				0xFF, 0xFF, 0xFF, 0xFF,
				0xFF, 0xFF, 0xFF, 0xFF,
				0xFF, 0xFF, 0xFF, 0xFF
		);
	}
	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processNtwKeyLine, BOOL_TRUE) == BOOL_TRUE ) {
		if ( sendAtCommand("AT&W",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processAtLineExpectOk,BOOL_TRUE) == BOOL_TRUE ) {
			// apparently a reset is required when the appKey is changed
			echoStarInit();
			return BOOL_TRUE;
		} else return BOOL_FALSE;
	} else return BOOL_FALSE;
}

// -------

e_lineFuncRet processUserKeyLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 27 && strncmp(r,"CK:",3) == 0) {
		// CK: 00:16:C0:01:F0:10:01:60
		char * c = &r[4];
		for ( int i = 0 ; i < 16 ; i++ ) {
			_es_config.user_key[i] = itdt_convertHexChar2Int(c);
			c+=3;
		}
	}
	return R_CONTINUE;
}

// Return the NetworkKey as an array
itsdk_bool_e echoStarGetUserKey(uint8_t * key) {
	bzero(key,16);
	_es_config.user_key = key;
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	if ( sendAtCommand("AT+CK?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processUserKeyLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// Set the NetworkKey as an array, if null, send 0xFF, this reset the key to original version
itsdk_bool_e echoStarSetUserKey(uint8_t * ntwKey) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	char cmd[64];
	if ( ntwKey != NULL ) {
		sprintf(cmd,"AT+CK=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
				ntwKey[0],ntwKey[1],ntwKey[2],ntwKey[3],
				ntwKey[4],ntwKey[5],ntwKey[6],ntwKey[7],
				ntwKey[8],ntwKey[9],ntwKey[10],ntwKey[11],
				ntwKey[12],ntwKey[13],ntwKey[14],ntwKey[15]
		);
	} else return BOOL_FALSE;

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processUserKeyLine, BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}


// -------

e_lineFuncRet processJoinEuiLine(char * r) {
	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 27 && strncmp(r,"NI:",3) == 0) {
		// NI: 45:43:48:4F:2D:45:56:4B
		char * c = &r[4];
		for ( int i = 0 ; i < 8 ; i++ ) {
			_es_config.joinEui[i] = itdt_convertHexChar2Int(c);
			c+=3;
		}
	}
	return R_CONTINUE;
}

// Return the deviceID as an array
itsdk_bool_e echoStarGetJoinEui(uint8_t * joinEui) {
	bzero(joinEui,8);
	_es_config.joinEui = joinEui;
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	if ( sendAtCommand("AT+NI?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processJoinEuiLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// Set the JoinEui as an array, if null, send 0xFF, this reset the key to original version
itsdk_bool_e echoStarSetJoinEui(uint8_t * joinEui) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	char cmd[64];
	if ( joinEui != NULL ) {
		sprintf(cmd,"AT+NI=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
				joinEui[0],joinEui[1],joinEui[2],joinEui[3],
				joinEui[4],joinEui[5],joinEui[6],joinEui[7]
		);
	} else {
		sprintf(cmd,"AT+NI=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
				0xFF, 0xFF, 0xFF, 0xFF,
				0xFF, 0xFF, 0xFF, 0xFF
		);
	}
	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processJoinEuiLine,BOOL_TRUE) == BOOL_TRUE ) {
		// apparently it needs to be saved with AT&W
		if ( sendAtCommand("AT&W",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processAtLineExpectOk,BOOL_TRUE) == BOOL_TRUE ) {
			// apparently a reset is required when the joinEui is changed
			echoStarInit();
			return BOOL_TRUE;
		} else return BOOL_FALSE;
	} else return BOOL_FALSE;
}


// -------

e_lineFuncRet processPinCodeLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");
	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 16 && strncmp(r,"PIN:",4) == 0) {
		// PIN: 19:CF:DC:4C
		char * c = &r[5];
		for ( int i = 0 ; i < 8 ; i++ ) {
			_es_config.pinCode[i] = itdt_convertHexChar2Int(c);
			c+=3;
		}
	}
	return R_CONTINUE;
}

// Return the PinCode as an array[4]
// Pincode allows to claim the device in Semtech portal
itsdk_bool_e echoStarGetPinCode(uint8_t * pinCode) {
	bzero(pinCode,4);
	_es_config.pinCode = pinCode;
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	if ( sendAtCommand("AT+PIN?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processPinCodeLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}


// -------

e_lineFuncRet processJoinModeLine(char * r) {

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 5 && strncmp(r,"NJM:",4) == 0) {
		// NJM:1
		char c = r[4];
		if ( c == '0' ) _es_config.joinMode = JM_ABP;
		else if ( c == '1' ) _es_config.joinMode = JM_OTAA;
		else _es_config.joinMode = JM_UNKNOWN;
	}
	return R_CONTINUE;
}

// Return the Join Mode as a value
es_joinMode_t echoStarGetJoinMode() {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	if ( sendAtCommand("AT+NJM?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processJoinModeLine,BOOL_TRUE) == BOOL_TRUE ) {
		return _es_config.joinMode;
	} else return JM_UNKNOWN;
}

// Set the JoinMode
itsdk_bool_e echoStarSetJoinMode(es_joinMode_t joinMode) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	char cmd[64];
	if ( joinMode != JM_UNKNOWN ) {
		sprintf(cmd,"AT+NJM=%d",joinMode);
	} else {
		return BOOL_FALSE;
	}
	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processJoinModeLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}



// -------

e_lineFuncRet processJoinStatusLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 5 && strncmp(r,"NJS:",4) == 0) {
		// NJS:1
		char c = r[4];
		if ( c == '0' ) _es_config.joinStatus = JS_DISCONNECTED;
		else if ( c == '1' ) _es_config.joinStatus = JS_JOINED;
		else _es_config.joinStatus = JS_UNKNOWN;
	}
	return R_CONTINUE;
}

// Return the Join Mode as a value
es_joinStatus_t echoStarGetJoinStatus() {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	_es_config.joinStatus = JS_UNKNOWN;
	if ( sendAtCommand("AT+NJS?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processJoinStatusLine,BOOL_TRUE) == BOOL_TRUE ) {
		return _es_config.joinStatus;
	} else {
		return JS_UNKNOWN;
	}
}

// -------

e_lineFuncRet processRegionLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 29 && strncmp(r,"Current Def",11) == 0) {
		// Current Default Region: EU868
		char * c = &r[24];
		if ( strncmp(c,"MSS-S",5) == 0 ) _es_config.region = REGION_MSS_S;
		else if (strncmp(c,"EU868",5) == 0 ) _es_config.region = REGION_EU868;
		else if (strncmp(c,"US915",5) == 0 ) _es_config.region = REGION_US915;
		else _es_config.region = REGION_UNKNOWN;
	}
	return R_CONTINUE;
}

// Return the Region as a value
es_region_t echoStarGetRegion() {
	if ( ! echoStarWakeUpModem() ) return REGION_UNKNOWN;
	if ( sendAtCommand("AT+REGION?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processRegionLine,BOOL_TRUE) == BOOL_TRUE ) {
		return _es_config.region;
	} else return REGION_UNKNOWN;
}

// Set the Region
itsdk_bool_e echoStarSetRegion(es_region_t region) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	char cmd[64];
	if ( region == REGION_MSS_S ) sprintf(cmd,"AT+REGION=MSS-S");
	else if ( region == REGION_EU868 ) sprintf(cmd,"AT+REGION=EU868");
	else if ( region == REGION_US915 ) sprintf(cmd,"AT+REGION=US915");
	else return BOOL_FALSE;

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processRegionLine,BOOL_TRUE) == BOOL_TRUE ) {
		// wait for reinit ( 2 lines )
		// INFO: LoRaMac stack initialized @ 01/01/1970 01:06:49
		// WARNING: Restarting LoRaWAN stack with new region EU868, rejoining network. @ 0
		__es_boot_line = 1;
		sendAtCommand(NULL,1000,3000,processEsModemBoot,BOOL_FALSE);
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}


// -------

e_lineFuncRet processMssTxPowerLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 8 && strncmp(r,"TXPMSS:",7) == 0) {
		// TXPMSS:27
		char * c = &r[7];
		int v = atoi(c);
		_es_config.txPower = (uint8_t)v;
	}
	return R_CONTINUE;
}

// Return the Tx Mss power as a value
uint8_t echoStarGetMssTxPower() {
	if ( ! echoStarWakeUpModem() ) return TX_POWER_ERROR;
	_es_config.txPower = TX_POWER_ERROR;
	if ( sendAtCommand("AT+TXPMSS?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processMssTxPowerLine,BOOL_TRUE) == BOOL_TRUE ) {
		return _es_config.txPower;
	} else return TX_POWER_ERROR;
}

// Set the Sat Tx Power
itsdk_bool_e echoStarSetMssTxPower(uint8_t power) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;

	char cmd[64];
	if ( power > 27 ) return BOOL_FALSE;
	sprintf(cmd,"AT+TXPMSS=%02d",power);

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processMssTxPowerLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// -------

e_lineFuncRet processMssAntGain(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 8 && strncmp(r,"ANTMSS:",7) == 0) {
		// ANTMSS:0
		char * c = &r[7];
		int v = atoi(c);
		_es_config.antGain = (uint8_t)v;
	}
	return R_CONTINUE;
}

// Return the Mss antena Gain as a value
int8_t echoStarGetMssAntGain() {
	if ( ! echoStarWakeUpModem() ) return ANT_GAIN_ERROR;
	_es_config.antGain = ANT_GAIN_ERROR;
	if ( sendAtCommand("AT+ANTMSS?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processMssAntGain,BOOL_TRUE) == BOOL_TRUE ) {
		return _es_config.antGain;
	} else return ANT_GAIN_ERROR;
}

// Set the Sat Tx Power
itsdk_bool_e echoStarSetMssAntGain(int8_t gain) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;

	char cmd[64];
	sprintf(cmd,"AT+ANTMSS=%d",gain);

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processMssAntGain,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}


// -------

e_lineFuncRet processSGhTxPowerLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 5 && strncmp(r,"TXP:",4) == 0) {
		// TXP:22
		char * c = &r[4];
		int v = atoi(c);
		_es_config.txPower = (uint8_t)v;
	}
	return R_CONTINUE;
}

// Return the Region as a value
uint8_t echoStarGetSGhTxPower() {
	if ( ! echoStarWakeUpModem() ) return TX_POWER_ERROR;
	_es_config.txPower = TX_POWER_ERROR;
	if ( sendAtCommand("AT+TXP?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processSGhTxPowerLine,BOOL_TRUE) == BOOL_TRUE ) {
		return _es_config.txPower;
	} else return TX_POWER_ERROR;
}

// Set the Sub GHz Tx power
itsdk_bool_e echoStarSetSGhTxPower(uint8_t power) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;

	char cmd[64];
	if ( power > 22 ) return BOOL_FALSE;
	sprintf(cmd,"AT+TXP=%02d",power);

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processSGhTxPowerLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// -------

e_lineFuncRet processKeepAliveTimeLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 7 && strncmp(r,"KATIME:",7) == 0) {
		// KATIME:86400000
		char * c = &r[7];
		int v = atoi(c);
		_es_config.katime = v;
	}
	return R_CONTINUE;
}

// Return the Region as a value
uint32_t echoStarGetKeepAliveTimeMs() {
	if ( ! echoStarWakeUpModem() ) return 0;
	_es_config.katime = 0;
	if ( sendAtCommand("AT+KATIME?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processKeepAliveTimeLine,BOOL_TRUE) == BOOL_TRUE ) {
		return _es_config.katime;
	} else return 0;
}

// Set the Sub GHz Tx power
itsdk_bool_e echoStarSetKeepAliveTimeMs(uint32_t kaTimeMs) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;

	char cmd[64];
	if ( kaTimeMs < 30*1000 ) return BOOL_FALSE;
	sprintf(cmd,"AT+KATIME=%d",(unsigned int)kaTimeMs);

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processKeepAliveTimeLine,BOOL_TRUE) == BOOL_TRUE ) {
		// apparently it needs to be saved with AT&W
		if ( sendAtCommand("AT&W",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processAtLineExpectOk,BOOL_TRUE) == BOOL_TRUE ) {
			// apparently a reset is required when the joinEui is changed
			echoStarInit();
			return BOOL_TRUE;
		} else return BOOL_FALSE;
	} else return BOOL_FALSE;
}

// -------

e_lineFuncRet processAdrStateLine(char * r) {
	//log_debug("[%d] ",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 5 && strncmp(r,"ADR:",4) == 0) {
		// ADR:1
		if ( r[4] == '0') _es_config.adr = ADR_OFF;
		else if ( r[4] == '1' ) _es_config.adr = ADR_ON;
		else _es_config.adr = ADR_UNKNOWN;
	}
	return R_CONTINUE;
}

// Return the Region as a value
es_adr_t echoStarGetAdrState() {
	if ( ! echoStarWakeUpModem() ) return ADR_UNKNOWN;
	_es_config.adr = ADR_UNKNOWN;
	if ( sendAtCommand("AT+ADR?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processAdrStateLine,BOOL_TRUE) == BOOL_TRUE ) {
		return _es_config.adr;
	} else return _es_config.adr; // apparently we get an error with fw 1.15 but the ADR value is returned
}

// Set the Sub GHz Tx power
itsdk_bool_e echoStarSetAdrState(es_adr_t adrState) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;

	char cmd[64];
	if ( adrState == ADR_ON ) {
		sprintf(cmd,"AT+ADR=1");
	} else if ( adrState == ADR_OFF ) {
		sprintf(cmd,"AT+ADR=0");
	} else return BOOL_FALSE;

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processAdrStateLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// -------

e_lineFuncRet processEncryptLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 5 && strncmp(r,"ENC: ",5) == 0) {
		// ENC: 3E:2D:DD:3E:5F:5C:59:A3:D1:F5:91:E9:A2:79:4B:F8
		char * c = &r[5];
		int sz = (strlen(c)+1)/3;
		for ( int i = 0 ; i < sz ; i++ ) {
			_es_config.encoded[i] = itdt_convertHexChar2Int(c);
			c+=3;
		}
	}
	if ( strlen(r) >= 5 && strncmp(r,"DEC: ",5) == 0) {
		// DEC: 3E:2D:DD:3E:5F:5C:59:A3:D1:F5:91:E9:A2:79:4B:F8
		char * c = &r[5];
		int sz = (strlen(c)+1)/3;
		for ( int i = 0 ; i < sz ; i++ ) {
			_es_config.encoded[i] = itdt_convertHexChar2Int(c);
			c+=3;
		}
	}
	return R_CONTINUE;
}

// Return the NetworkKey as an array
itsdk_bool_e echoStarDecrypt(uint8_t * payload, uint8_t size) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	if ( size > 50 ) return BOOL_FALSE;

	char cmd[128];
	if ( payload != NULL ) {
		char * p = cmd;
		sprintf(cmd,"AT+DEC=");
		p += 7;
		for ( int i = 0 ; i < size ; i++ ) {
			sprintf(p,"%02X",payload[i]);
			p +=2;
		}
	} else return BOOL_FALSE;
	_es_config.encoded = payload;

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processEncryptLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// Set the NetworkKey as an array, if null, send 0xFF, this reset the key to original version
itsdk_bool_e echoStarEncrypt(uint8_t * payload, uint8_t size) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	if ( size > 50 ) return BOOL_FALSE;

	char cmd[128];
	if ( payload != NULL ) {
		char * p = cmd;
		sprintf(cmd,"AT+ENC=");
		p += 7;
		for ( int i = 0 ; i < size ; i++ ) {
			sprintf(p,"%02X",payload[i]);
			p +=2;
		}
	} else return BOOL_FALSE;
	_es_config.encoded = payload;

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processEncryptLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// -------

e_lineFuncRet processDeviceClassLine(char * r) {
	//log_debug("[%d] ",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 4 && strncmp(r,"DC:",3) == 0) {
		// DC:C
		if ( r[3] == 'A') _es_config.class = EM_CLASS_A;
		else if ( r[3] == 'B' ) _es_config.class = EM_CLASS_B;
		else if ( r[3] == 'C' ) _es_config.class = EM_CLASS_C;
		else _es_config.class = EM_CLASS_UNKNOWN;
	}
	return R_CONTINUE;
}

// Return the Region as a value
es_class_t echoStarGetDeviceClass() {
	if ( ! echoStarWakeUpModem() ) return EM_CLASS_UNKNOWN;
	_es_config.class = EM_CLASS_UNKNOWN;
	if ( sendAtCommand("AT+DC?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processDeviceClassLine,BOOL_TRUE) == BOOL_TRUE ) {
		return _es_config.class;
	} else return EM_CLASS_UNKNOWN; // apparently we get an error with fw 1.15 but the ADR value is returned
}

// Set the Sub GHz Tx power
itsdk_bool_e echoStarSetDeviceClass(es_class_t devClass) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;

	char cmd[64];
	if ( devClass == EM_CLASS_A ) {
		sprintf(cmd,"AT+DC=A");
	} else if ( devClass == EM_CLASS_B ) {
		// not supported
		return BOOL_FALSE;
	} else if ( devClass == EM_CLASS_C ) {
		sprintf(cmd,"AT+DC=C");
	} else return BOOL_FALSE;

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processDeviceClassLine,BOOL_TRUE) == BOOL_TRUE ) {
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}


// -------


e_lineFuncRet processSatPowerAndSFLine(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	if ( strlen(r) >= 7 && strncmp(r,"CTP:",4) == 0) {
		// CTP:14,0
		char * c = &r[4];
		int v = atoi(c);
		_es_config.pwsf.txPower = (uint8_t)v;
		if ( r[5] == ',' ) c = &r[6];
		if ( r[6] == ',' ) c = &r[7];
		switch (*c) {
			case '0' : _es_config.pwsf.sf = SF_LRFHSS_1_3; break;
			case '1' : _es_config.pwsf.sf = SF_LRFHSS_2_3; break;
			case '3' : _es_config.pwsf.sf = SF_12; break;
			case '4' : _es_config.pwsf.sf = SF_11; break;
			default: _es_config.pwsf.sf = SF_UNKNOWN; break;
		}
	}
	return R_CONTINUE;
}

// Return the power and SF configuration, apparently not applicable to terrestrial
itsdk_bool_e echoStarGetSatPowerAndSf(uint8_t * power, es_sf_t * sf) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;
	_es_config.pwsf.txPower = TX_POWER_ERROR;
	_es_config.pwsf.sf = SF_UNKNOWN;
	if ( sendAtCommand("AT+CTP?",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processSatPowerAndSFLine,BOOL_TRUE) == BOOL_TRUE ) {
		*power = _es_config.pwsf.txPower;
		*sf = _es_config.pwsf.sf;
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// --- Factory Reset

e_lineFuncRet processFactoryReset(char * r) {
	//log_debug("%d\r\n",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;
	return R_CONTINUE;
}

itsdk_bool_e echoStarFactoryReset() {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;

	if ( sendAtCommand("AT&F",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processFactoryReset,BOOL_TRUE) == BOOL_TRUE ) {
		if ( sendAtCommand("AT&W",_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processUserKeyLine,BOOL_TRUE) == BOOL_TRUE ) {
		  return BOOL_TRUE;
		}
	}
	return BOOL_FALSE;
}

// -----------------------------------------------------
// Send & Receive
// -----------------------------------------------------

e_lineFuncRet processJoinLine(char * r) {
	//log_debug("[%d] ",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;

	return R_CONTINUE;
}

// Send a message
itsdk_bool_e echoStarJoin() {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;

	char cmd[64];
	sprintf(cmd,"AT+JOIN");

	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processJoinLine,BOOL_TRUE) == BOOL_TRUE ) {
		__es_boot_line = 3; // wait for 6-3 trace in console
		sendAtCommand(NULL,2000,2000,processEsModemBoot,BOOL_FALSE);
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

// -----


e_lineFuncRet processSendBytesLine(char * r) {
	// [53] AT+SENDB=2,0,1,0, A4 41 80 01 CC 92 20 59 19 02 32 47
	// [1]
	// [1]
	// [8] QUEUED:2
	// [3] OK
	// [1]
	// [7] SENT:2

	//log_debug("[%d] ",strlen(r));
	//log_error(r);log_error("\r\n");

	e_lineFuncRet ret = processDefaultCases(r);
	if ( ret != R_NONE ) return ret;
	return R_CONTINUE;
}

e_lineFuncRet processWaitOnSentLine(char * r) {
	//log_debug("[%d] ",strlen(r));
	//log_error(r);log_error("\r\n");

	if ( strlen(r) >= 5 && strncmp(r,"SENT:",5) == 0) {
		// SENT: 0
		return R_EXIT_SUCCESS;
	}
	return R_CONTINUE;
}

// Send a message ; in blocking mode, the transmission confirmation is expected ( can be long but limited to 10 seconds)
// Unclean how to get a downlink, the value is not displayed
uint8_t __es_message_counter = 0;
itsdk_bool_e echoStarSendBytes(uint8_t port, uint8_t * msg, uint8_t sz, itsdk_bool_e ack, es_urgent_t urg, itsdk_bool_e blocking) {
	if ( ! echoStarWakeUpModem() ) return BOOL_FALSE;

	char cmd[128];
	sprintf(cmd,"AT+SENDB=%d,%d,%d,%d,",__es_message_counter,(uint8_t)urg,port,((ack==BOOL_TRUE)?1:0));
	char * c = &cmd[strlen(cmd)];
	for ( int i = 0 ; i < sz ; i++ ) {
		sprintf(c," %02X",msg[i]);
		c+=3;
	}
	__es_message_counter++;
	if ( sendAtCommand(cmd,_EM2050_INITTMOUT_DEFAULT,_EM2050_TMOUT_DEFAULT,processSendBytesLine,BOOL_TRUE) == BOOL_TRUE ) {
		if ( blocking ) {
			sendAtCommand(NULL,10000,10000,processWaitOnSentLine,BOOL_FALSE);
		}
		return BOOL_TRUE;
	} else return BOOL_FALSE;
}

#endif // ITSDK_DRIVERS_EM2050

