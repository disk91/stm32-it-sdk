/* ==========================================================
 * sl353.h -  Honeywell SL353 - Hall Effect magnetic sensor
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 28 may. 2019
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
#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_SL353 == __ENABLE
#include <it_sdk/config_defines.h>
#include <drivers/hall/sl353/sl353.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/logger/logger.h>


/**
 * Interrupt handler
 */
static drivers_sl353_state_t __sl353_state;

static void __sl353_interrupt(uint16_t GPIO_Pin) {
	drivers_sl353_state_e new;
	drivers_sl353_getImmediateState(&new);
	if ( __sl353_state.status != new ) {
		__sl353_state.status = new;
		__sl353_state.hasChanged = 1;
		if ( __sl353_state.onFieldChange != NULL ) __sl353_state.onFieldChange(__sl353_state.status);
		if ( __sl353_state.status == SL353_FIELD_DETECTED ) {
			if ( __sl353_state.onFieldSet != NULL ) __sl353_state.onFieldSet();
		} else {
			if ( __sl353_state.onFieldReset != NULL ) __sl353_state.onFieldReset();
		}
	}
}

static gpio_irq_chain_t __sl353_gpio_irq = {
		__sl353_interrupt,
		ITSDK_DRIVERS_SL353_INT_PIN,
		NULL,
};


/**
 * Setup the sensor according to the selected mode.
 */
drivers_sl353_ret_e drivers_sl353_setup(
		void (*onFieldSet)(void),
		void (*onFieldReset)(void),
		void (*onFieldChange)(drivers_sl353_state_e state)
) {

	log_debug("drivers_sl353_setup\r\n");

	if (ITSDK_DRIVERS_MAX17205_ALRT1_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_DRIVERS_SL353_INT_BANK,ITSDK_DRIVERS_SL353_INT_PIN,GPIO_INTERRUPT_ANY);
		gpio_registerIrqAction(&__sl353_gpio_irq);
	}
	drivers_sl353_getImmediateState(&__sl353_state.status);
	__sl353_state.hasChanged = 0;
	__sl353_state.onFieldSet = onFieldSet;
	__sl353_state.onFieldReset = onFieldReset;
	__sl353_state.onFieldChange = onFieldChange;
	return SL353_SUCCESS;
}

drivers_sl353_ret_e drivers_sl353_getImmediateState( drivers_sl353_state_e * state ) {
	uint8_t v = gpio_read(ITSDK_DRIVERS_SL353_INT_BANK,ITSDK_DRIVERS_SL353_INT_PIN);
	*state = ( v == 0 )? SL353_FIELD_DETECTED:SL353_FIELD_NOT_DETECTED;
	return SL353_SUCCESS;
}



#endif // ITSDK_DRIVERS_SL353

#endif // ITSDK_WITH_DRIVERS
