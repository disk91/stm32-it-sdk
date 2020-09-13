/* ==========================================================
 * main.c - Main Library Starting point
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 2 sept. 2018
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
 * 
 * This file is the main entry point for the Disk91 SDK
 * The project using the SDK must in its main function have the
 * Following content:
 * main() {
 *    // Needed project specific configuration
 *    ...
 *    // Call the IT_SDK Setup
 *    itsdk_setup();
 *
 *    // Loop forever
 *    while ( 1 ) {
 *      itsdk_loop();
 *    }
 * }
 *
 * The itsdk_setup() will make the sdk init then call the user function
 * project_setup(). This function have to be defined in your project for
 * executing the project specific setup.
 *
 * The itsdk_loop() will manage the low power execution and regularly call the
 * project_loop() function. It also managed the sdk scheduler. This function is
 * basically executed on each MCU wake-up. it depends on the hardware implementation.
 *
 * In case of MCU waking up from sleep with a MCU Reset (ex : ESP8266) a itsdk_restart()
 * function will be called instead of itsdk_setup()
 *
 * See each of the function for details.
 *
 * ----------------------------------------------------------
 *
 * ==========================================================
 */
#include <it_sdk/itsdk.h>
#include <it_sdk/lowpower/lowpower.h>
#include <it_sdk/sched/scheduler.h>
#include <it_sdk/time/time.h>
#include <it_sdk/time/timer.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/eeprom/sdk_state.h>
#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
  #include <it_sdk/lorawan/lorawan.h>
#endif
#if ITSDK_WITH_SIGFOX_LIB == __ENABLE
  #include <it_sdk/sigfox/sigfox.h>
#endif

#if ITSDK_WITH_SECURESTORE == __ENABLE
#include <it_sdk/eeprom/securestore.h>
#include <it_sdk/encrypt/encrypt.h>
#endif

#if ITSDK_WITH_CONSOLE == __ENABLE
#include <it_sdk/console/console.h>
#endif

#if ITSDK_WITH_ERROR_RPT == __ENABLE
#include <it_sdk/logger/error.h>
#endif

#if ITSDK_WITH_DRIVERS == __ENABLE
#include <it_sdk/configDrivers.h>
  #if ITSDK_DRIVERS_WITH_ACCEL_DRIVER == __ENABLE
	#include <it_sdk/accel/accel.h>
  #endif
  #if ITSDK_DRIVERS_WITH_GNSS_DRIVER == __ENABLE
	#include <it_sdk/gnss/gnss.h>
  #endif
#endif

/**
 * The setup function is called on every MCU Reset but not on wakeup from sleep
 * This function init the SDK library and underlaying hardware.
 * Then it calls the project specific setup function
 */
void itsdk_setup() {

	itsdk_time_init();
	#if ITSDK_LOGGER_CONF > 0
	log_init(ITSDK_LOGGER_CONF);
	#endif
	#if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
	  wdg_setupWithMaxMs(ITSDK_WDG_MS);
	#endif
	serial1_init();
	serial2_init();
	#if ITSDK_WITH_CONSOLE == __ENABLE
		itsdk_console_setup();
	#endif
	#if ITSDK_WITH_ERROR_RPT == __ENABLE
	  itsdk_error_setup();
	  ITSDK_ERROR_REPORT(ITSDK_ERROR_RESET,(uint16_t)itsdk_getResetCause());
	#endif
	#if ITSDK_WITH_SECURESTORE == __ENABLE
	  // Init the secure store if not yet initialized
	  if ( itsdk_secstore_isInit() != SS_SUCCESS ) {
		  itsdk_secstore_init();
		  itsdk_encrypt_resetFactoryDefaults(BOOL_TRUE);
		  #if ITSDK_WITH_LORAWAN_LIB == __ENABLE
		    itsdk_lorawan_resetFactoryDefaults(true);
		  #endif
		  #if ITSDK_WITH_SIGFOX_LIB == __ENABLE
 		    itsdk_sigfox_resetFactoryDefaults(true);
		  #endif
	  } else {
	     itsdk_encrypt_resetFactoryDefaults(BOOL_FALSE);	// on first boot init the ss communication credentials
	  }
	  itsdk_secStore_RegisterConsole();
	#endif
	// load the configuration according to setting
	itsdk_config_loadConfiguration(CONFIG_NORMAL_LOAD);
	itsdk_state_init();
	// Application setup
	project_setup();
    #if ITSDK_WITH_ERROR_RPT == __ENABLE
      itsdk_cleanResetCause();
    #endif

}

/**
 * The restart function is called when the MCU wakeup from lowpower mode with a full
 * reset like for ESP8266 to setup the itsdk without passing through itsdk_setup function.
 */
void itsdk_restart() {

	itsdk_loop();
}


/**
 * The loop function is called everytime the MCU is waking up or in a cycling loop
 * if the MCU have no lowpower mode. The function update the time component, call
 * all the recurrent SDK operations to be maintained.
 * When a scheduler has been activated it calls the scheduler task when needed.
 * Then is calls the project specific loop function.
 */
void itsdk_loop() {

    #if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
	   wdg_refresh();
	#endif
	#if ITSDK_TIMER_SLOTS > 0
	   itsdk_stimer_run();
	#endif
	#if ITSDK_SHEDULER_TASKS > 0
	   itdt_sched_execute();
	#endif
	#if ITSDK_DRIVERS_WITH_ACCEL_DRIVER == __ENABLE
	   accel_process_loop();
    #endif
	#if ITSDK_DRIVERS_WITH_GNSS_DRIVER == __ENABLE
	   gnss_process_loop(BOOL_FALSE);
	#endif
	project_loop();
	#if ITSDK_WITH_CONSOLE == __ENABLE
	   itsdk_console_loop();
	#endif
	#if ITSDK_TIMER_SLOTS > 0
		if ( itsdk_stimer_isLowPowerSwitchAutorized() ) {
	#endif
			lowPower_switch();
	#if ITSDK_TIMER_SLOTS > 0
		}
	#endif
}




