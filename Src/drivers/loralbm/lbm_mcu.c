/* ==========================================================
 * lbm_mcu.c - LoRa Basic Modem / itsdk integration
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
 *  Created on: 30 july 2024
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2024
 * ==========================================================
 */
#include <it_sdk/config.h>
#if ITSDK_WITH_LORAWAN_LIB == __ENABLE && ITSDK_LORAWAN_STACK == __LORAWAN_LBM
#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>

#include <it_sdk/time/time.h>
#include <it_sdk/time/timer.h>
#include <it_sdk/itsdk.h>				// random
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>

#include <drivers/loralbm/lbm_mcu.h>
#include <smtc_modem_hal/smtc_modem_hal.h>


// =========================================================
//
// System
//
// =========================================================

// ---------------------------------------------------------
// @TODO - unclear when it's called, I assume it's not resetting the mcu
// ---------------------------------------------------------
void smtc_modem_hal_reset_mcu( void ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_reset_mcu\r\n"));
}

// ---------------------------------------------------------
// Call by the stack to reset the watchdog
// ---------------------------------------------------------
void smtc_modem_hal_reload_wdog( void ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_reload_wdog\r\n"));
	#if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
	  wdg_refresh();
	#endif
}

// ---------------------------------------------------------
// Panic Management
// ---------------------------------------------------------
void smtc_modem_hal_on_panic( uint8_t* func, uint32_t line, const char* fmt, ... ) {

	_LOG_LBMMCU_ERROR(("[LBM] smtc_modem_hal_on_panic f(%08X) l(%d) \r\n",func,line));

  #if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWLORADBG) > 0
	va_list args;
	char 	fmtBuffer[LOGGER_MAX_BUF_SZ];
    va_start(args,fmt);
    vsnprintf(fmtBuffer,LOGGER_MAX_BUF_SZ,fmt,args);
    va_end(args);
    _LOG_LBMMCU_ERROR((fmtBuffer));
  #endif
}

// ---------------------------------------------------------
// Traces
// ---------------------------------------------------------
void smtc_modem_hal_print_trace( const char* fmt, ... ) {
  #if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWLORADBG) > 0
	va_list args;
	char 	fmtBuffer[LOGGER_MAX_BUF_SZ];
    va_start(args,fmt);
    vsnprintf(fmtBuffer,LOGGER_MAX_BUF_SZ,fmt,args);
    va_end(args);
    _LOG_LBMMCU_INFO((fmtBuffer));
  #endif
}


// ---------------------------------------------------------
// Get temperature in Celsius
// ---------------------------------------------------------
int8_t smtc_modem_hal_get_temperature( void ) {

	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_get_temperature\r\n"));
	return adc_getTemperature()/100;
}

// ---------------------------------------------------------
// Battery level
// Assuming it expects a battery percentage
// Depends on battry type so will return 100% by default and
// propose to upgrade it with a weak
// ---------------------------------------------------------
__weak uint8_t smtc_modem_hal_get_battery_level( void ) {

	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_get_battery_level\r\n"));
	return 100;
}

// ---------------------------------------------------------
// Get voltage
// ---------------------------------------------------------
uint16_t smtc_modem_hal_get_voltage_mv( void ) {

	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_get_voltage_mv\r\n"));
	return adc_getVdd();
}

// ----------------------------------------------------------
// Wakeup delay
// ----------------------------------------------------------
int8_t smtc_modem_hal_get_board_delay_ms( void ) {

	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_get_board_delay_ms\r\n"));
	return ITSDK_LORAW_SX126X_LATENCYMS;
}


// =========================================================
//
// Time Management
//
// =========================================================

// ---------------------------------------------------------
// Return current time in seconds
// Assuming it's a relative time from startup as we have in sdk
// ---------------------------------------------------------
uint32_t smtc_modem_hal_get_time_in_s( void ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_get_time_in_s\r\n"));
	uint64_t t = itsdk_time_get_ms();
	t /= 1000L;
	return (uint32_t)t;
}

// ---------------------------------------------------------
// Return current time in ms
// Assuming it's a relative time from startup as we have in sdk
// (wraps every 49 days)
// ---------------------------------------------------------
uint32_t smtc_modem_hal_get_time_in_ms( void ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_get_time_in_ms\r\n"));
	uint64_t t = itsdk_time_get_ms();
	return (uint32_t)t;
}


// ---------------------------------------------------------
// Set an offset into the RTC counter (debug use)
// let's do nothing
// ---------------------------------------------------------
void smtc_modem_hal_set_offset_to_test_wrapping( const uint32_t offset_to_test_wrapping ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_set_offset_to_test_wrapping\r\n"));
}


// =========================================================
//
// Timer Management
//
// =========================================================

// ---------------------------------------------------------
// Start a timer for a given duration with a callback
// ---------------------------------------------------------
void (*__lbm_timer_cbfunc)(uint32_t value) = NULL;
uint32_t __lbm_timer_cbValue;

void smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ), void* context ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_start_timer %d\r\n",milliseconds));

	itsdk_timer_return_t ret;

	ret = itsdk_stimer_register(
					milliseconds,
					(void (*)(uint32_t value))callback,
					(uint32_t)context,
					TIMER_ACCEPT_LOWPOWER
	);

	if ( ret != TIMER_INIT_SUCCESS ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_TIME_INITFLD,(uint16_t)ret);
		__lbm_timer_cbfunc = NULL;
		__lbm_timer_cbValue = 0;
	} else {
		__lbm_timer_cbfunc = (void (*)(uint32_t value))callback;
		__lbm_timer_cbValue = (uint32_t)context;
	}
}

// ---------------------------------------------------------
// Stop last running timer
// ---------------------------------------------------------
void smtc_modem_hal_stop_timer( void ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_stop_timer\r\n"));
	if ( __lbm_timer_cbfunc != NULL ) {
		itsdk_stimer_stop(__lbm_timer_cbfunc,__lbm_timer_cbValue);
	}
}

// =========================================================
//
// IRQ Management
//
// =========================================================


// ---------------------------------------------------------
// Enable IRQ
// ---------------------------------------------------------
void smtc_modem_hal_enable_modem_irq( void ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_enable_modem_irq\r\n"));

	#if ITSDK_LORAWAN_LIB == __LORAWAN_SX126X

		#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0

			ITSDK_LORAW_SX126X_SPI.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
			HAL_StatusTypeDef r = HAL_SUBGHZ_Init(&ITSDK_LORAW_SX126X_SPI);
			if (r != HAL_OK) {
			  _LOG_LBMMCU_ERROR(("[SX] Failed to init subghz %d\r\n",r));

			}

			__HAL_RCC_SUBGHZSPI_CLK_ENABLE();
			HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0, 0);
			HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);

		#else
			if ( ITSDK_LORAW_SX126X_IRQ_PIN != __LP_GPIO_NONE ) {
				gpio_interruptClear(ITSDK_LORAW_SX126X_IRQ_BANK, ITSDK_LORAW_SX126X_IRQ_PIN);
				gpio_configure(ITSDK_LORAW_SX126X_IRQ_BANK, ITSDK_LORAW_SX126X_IRQ_PIN, GPIO_INTERRUPT_RISING );
				gpio_interruptPriority(ITSDK_LORAW_SX126X_IRQ_BANK,ITSDK_LORAW_SX126X_IRQ_PIN,0,0);
				gpio_registerIrqAction(&__sx1262_irq);
				gpio_interruptEnable(ITSDK_LORAW_SX126X_IRQ_BANK, ITSDK_LORAW_SX126X_IRQ_PIN);
			}
		#endif

	#else // ITSDK_LORAWAN_LIB
		#error "Unsupported LORAWAN_LIB (HW target)"
	#endif

}

// ---------------------------------------------------------
// Disable IRQ
// ---------------------------------------------------------
void smtc_modem_hal_disable_modem_irq( void ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_disable_modem_irq\r\n"));

	#if ITSDK_LORAWAN_LIB == __LORAWAN_SX126X

		#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0

			__HAL_RCC_SUBGHZSPI_CLK_DISABLE();
			HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);

		#else
			if ( ITSDK_LORAW_SX126X_IRQ_PIN != __LP_GPIO_NONE ) {
				gpio_interruptClear(ITSDK_LORAW_SX126X_IRQ_BANK, ITSDK_LORAW_SX126X_IRQ_PIN);
				gpio_configure(ITSDK_LORAW_SX126X_IRQ_BANK, ITSDK_LORAW_SX126X_IRQ_PIN, GPIO_STOP );
				gpio_removeIrqAction(&__sx1262_irq);
			}
		#endif

	#else // ITSDK_LORAWAN_LIB
		#error "Unsupported LORAWAN_LIB (HW target)"
	#endif

}

// ---------------------------------------------------------
// Configure IRQ callback
// ---------------------------------------------------------

static void ( *__sx1262_irq_cb )( void* context ) = NULL;
static void* __sx1262_irq_context;
#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) == 0
	static gpio_irq_chain_t __sx1262_irq;
	void __sx1262_irq_callback(uint16_t gpio) {
		if (__sx1262_irq_cb != NULL) {
			__sx1262_irq_cb(__sx1262_irq_context);
		}
	}
#endif

void smtc_modem_hal_irq_config_radio_irq( void ( *callback )( void* context ), void* context ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_irq_config_radio_irq\r\n"));

	__sx1262_irq_cb = callback;
	__sx1262_irq_context = context;
	#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) == 0
		__sx1262_irq.irq_func = __sx1262_irq_callback;
		__sx1262_irq.pinMask = ITSDK_LORAW_SX126X_IRQ_PIN;
		gpio_registerIrqAction(&__sx1262_irq);
	#endif
}

#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0

// TODO ... IRQ requires to be mutualized for sigfox & LoRaWan ...
#warning TODO...

#endif


// ---------------------------------------------------------
// LBM Callback on every interrupt
// ---------------------------------------------------------
void smtc_modem_hal_user_lbm_irq( void ) {
	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_user_lbm_irq\r\n"));

}

// =========================================================
//
// EEPROM Context saving
//
// =========================================================


// ---------------------------------------------------------
// Save Context
// We have 6 type of context (modem, key_modem, lorawan_stack,
//  FUOTA, Secure element, store & forward)
// So the offset in flash may be different (unsure about offset,
//  should be an offset considering the context used)
// buffer is data to store & size, size of data
// ---------------------------------------------------------
void smtc_modem_hal_context_store(
		const modem_context_type_t ctx_type,
		uint32_t offset,
		const uint8_t* buffer,
        const uint32_t size ) {

	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_context_store type(%d) off(%d) sz(%d) \r\n",ctx_type,offset,size));

	// TODO
#warning "TODO To be implemented later once understand what is expected really"

}

// ---------------------------------------------------------
// Restore Context
// ---------------------------------------------------------
void smtc_modem_hal_context_restore(
		const modem_context_type_t ctx_type,
		uint32_t offset,
		uint8_t* buffer,
        const uint32_t size ) {

	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_context_restore type(%d) off(%d) sz(%d) \r\n",ctx_type,offset,size));

	// TODO
#warning "TODO To be implemented later once understand what is expected really"

}

// ---------------------------------------------------------
// Restore Context (unclear, notion of page)
// ---------------------------------------------------------
void smtc_modem_hal_context_flash_pages_erase(
		const modem_context_type_t ctx_type,
		uint32_t offset,
		uint8_t nb_page ) {

	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_context_flash_pages_erase type(%d) off(%d) pg(%d) \r\n",ctx_type,offset,nb_page));

	// TODO
#warning "To be implemented later once understand what is expected really"
}

// ---------------------------------------------------------
// The number of page in flash for the Store & forward service
// unclear ... notion of page too much specific
// ---------------------------------------------------------
uint16_t smtc_modem_hal_store_and_forward_get_number_of_pages( void ) {

	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_store_and_forward_get_number_of_pages\r\n"));

	// TODO
#warning "To be implemented later once understand what is expected really"

	return 6;
}

// ---------------------------------------------------------
// Gives the size of a flash page in bytes
// ---------------------------------------------------------
uint16_t smtc_modem_hal_flash_get_page_size( void ) {

	_LOG_LBMMCU_DEBUG(("[LBM] smtc_modem_hal_flash_get_page_size\r\n"));

	// TODO
#warning "To be implemented later once understand what is expected really"


	return 64;
}

// =========================================================
//
// Crashlog management
//
// =========================================================

// ---------------------------------------------------------
// Store a crash log in NVM
// ---------------------------------------------------------
void smtc_modem_hal_crashlog_store( const uint8_t* crash_string, uint8_t crash_string_length ) {
	// @TODO see later
	#warning "see later"
	_LOG_LBMMCU_ERROR(("[LBM] %s\r\n",crash_string));
}

// ---------------------------------------------------------
// Retreive a crash log in NVM
// ---------------------------------------------------------
void smtc_modem_hal_crashlog_restore( uint8_t* crash_string, uint8_t* crash_string_length ) {

	_LOG_LBMMCU_ERROR(("[LBM] smtc_modem_hal_crashlog_restore\r\n"));

	// @TODO see later
	#warning "see later"
	crash_string[0]='\0';
	*crash_string_length = 0;
}

// ---------------------------------------------------------
// Set crashlog status
// ---------------------------------------------------------

void smtc_modem_hal_crashlog_set_status( bool available ) {

	_LOG_LBMMCU_ERROR(("[LBM] smtc_modem_hal_crashlog_set_status\r\n"));

	// @TODO see later
	#warning "see later"

	return;
}

// ---------------------------------------------------------
// Get crashlog previously stored status
// ---------------------------------------------------------
bool smtc_modem_hal_crashlog_get_status( void ) {

	_LOG_LBMMCU_ERROR(("[LBM] smtc_modem_hal_crashlog_get_status\r\n"));

	// @TODO see later
	#warning "see later"

	return false;
}

// =========================================================
//
// Random
//
// =========================================================


// ---------------------------------------------------------
// Returns a random value between [val_1,val_2]
// TODO To be optimized as this relly returns [val_1,val_2[
// but if val_2 is 0xFFFFFFF... it can be a problem
// ---------------------------------------------------------
uint32_t smtc_modem_hal_get_random_nb_in_range( const uint32_t val_1, const uint32_t val_2 ) {

	_LOG_LBMMCU_ERROR(("[LBM] smtc_modem_hal_get_random_nb_in_range %d - %d\r\n",val_1,val_2));
	uint32_t delta = val_2 - val_1;
	uint32_t v = itsdk_randomByte();
	if ( delta >= (1<<8) ) v = (v << 8) + itsdk_randomByte();
	if ( delta >= (1<<16)) v = (v << 8) + itsdk_randomByte();
	if ( delta >= (1<<24)) v = (v << 8) + itsdk_randomByte();

	v += val_1;
	v %= val_2;

	return v;
}


// =========================================================
//
// TCXO
//
// =========================================================

void smtc_modem_hal_start_radio_tcxo( void ) {

	_LOG_LBMMCU_ERROR(("[LBM] smtc_modem_hal_start_radio_tcxo\r\n"));
	#if ITSDK_LORAW_SX126X_TCXO_SXCTL == __DISABLE
		#if ITSDK_LORAW_SX126X_TCXO_PIN != __LP_GPIO_NONE
			gpio_configure_ext(ITSDK_LORAW_SX126X_TCXO_BANK,ITSDK_LORAW_SX126X_TCXO_PIN,GPIO_OUTPUT_PP, ITSDK_GPIO_SPEED_HIGH, ITSDK_GPIO_ALT_NONE);
			gpio_set(ITSDK_LORAW_SX126X_TCXO_BANK,ITSDK_LORAW_SX126X_TCXO_PIN);
		#endif
	#else
		#error SX controled TCXO not yet supported
	#endif
}

void smtc_modem_hal_stop_radio_tcxo( void ) {

	_LOG_LBMMCU_ERROR(("[LBM] smtc_modem_hal_stop_radio_tcxo\r\n"));
	#if ITSDK_LORAW_SX126X_TCXO_SXCTL == __DISABLE
		#if ITSDK_LORAW_SX126X_TCXO_PIN != __LP_GPIO_NONE
			gpio_configure_ext(ITSDK_LORAW_SX126X_TCXO_BANK,ITSDK_LORAW_SX126X_TCXO_PIN,GPIO_OUTPUT_PP, ITSDK_GPIO_SPEED_HIGH, ITSDK_GPIO_ALT_NONE);
			gpio_reset(ITSDK_LORAW_SX126X_TCXO_BANK,ITSDK_LORAW_SX126X_TCXO_PIN);
		#endif
	#else
		#error SX controled TCXO not yet supported
	#endif
}

uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms( void ) {

	_LOG_LBMMCU_ERROR(("[LBM] smtc_modem_hal_get_radio_tcxo_startup_delay_ms\r\n"));
	return ITSDK_LORAW_SX126X_TCXO_SMS;
}



// =========================================================
//
// RFSWITCH
//
// =========================================================

void smtc_modem_hal_set_ant_switch( bool is_tx_on ) {

	_LOG_LBMMCU_ERROR(("[LBM] smtc_modem_hal_set_ant_switch\r\n"));

	// @TODO depends on the power ... unclear when to switch RX vs unconnected
#warning TODO
}

// =========================================================
//
// FUOTA (not supported)
//
// =========================================================

uint32_t smtc_modem_hal_get_hw_version_for_fuota( void ) {
	return 0;
}

uint32_t smtc_modem_hal_get_fw_version_for_fuota( void ) {
	return 0;
}

uint8_t smtc_modem_hal_get_fw_status_available_for_fuota( void ) {
	return 0;
}

uint8_t smtc_modem_hal_get_fw_delete_status_for_fuota( uint32_t fw_to_delete_version ) {
	return 0;
}

uint32_t smtc_modem_hal_get_next_fw_version_for_fuota( void ) {
	return 0;
}

#endif // ITSDK_WITH_LORAWAN_LIB == __ENABLE && ITSDK_LORAWAN_STACK == __LORAWAN_LBM
