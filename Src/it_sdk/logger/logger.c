/* ==========================================================
 * logger.c - Helper for easy logging
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
 */

/* Init the logger structure from a given configuration
 * The configuration is a 32bits field for each of the
 * 4 possible logging option with the 4 level of trace.
 * This configuration is optimize to be stored in the application
 * configuration and context to be easily saved & restore when
 * going deep sleep.
 * Format is:
 *  +-------------------------------------------------------------------+
 *  | FILE output |      Serial1      | Serial2 output |   DEBUG Link   |
 *  +-------------------------------------------------------------------+
 *  |   D I W E   |      D I W E      |     D I W E    |    D I W E     |
 *  +-------------------------------------------------------------------+
 *  D : DEBUG Level - activated when 1 / Mask when 0
 *  I : INFO Level - activated when 1 / Mask when 0
 *  W : WARN Level - activated when 1 / Mask when 0
 *  E : ERROR Level - activated when 1 / Mask when 0
 *
 *  Depends on platform all the target are not implemented
 * ==========================================================
 */
#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/time/time.h>


__t_log __log;

/**
 * Setup the logging level and target
 */
bool log_init(uint16_t config) {

  __log.logError  = (( config & LOGGER_CONFIG_ERROR_LVL_MASK  ) > 0)?1:0;
  __log.logWarn   = (( config & LOGGER_CONFIG_WARN_LVL_MASK   ) > 0)?1:0;
  __log.logInfo   = (( config & LOGGER_CONFIG_INFO_LVL_MASK   ) > 0)?1:0;
  __log.logDebug  = (( config & LOGGER_CONFIG_DEBUG_LVL_MASK  ) > 0)?1:0;
  __log.onSerial1 = (( config & LOGGER_CONFIG_SERIAL1_MASK    ) > 0)?1:0;
  __log.onSerial2 = (( config & LOGGER_CONFIG_SERIAL2_MASK    ) > 0)?1:0;
  __log.onDebug   = (( config & LOGGER_CONFIG_DEBUGLNK_MASK   ) > 0)?1:0;
  __log.onFile    = (( config & LOGGER_CONFIG_FILE_MASK       ) > 0)?1:0;

  // Init the loggers
  if (__log.onFile) {
	  // Init file logger
	  // @TODO support file logger
  }
  __log.logConf = config;
  __log.ready = true;
  return true;
}


/**
 * Terminate the logging. This should be called before going deep-sleep to flush
 * Current log processing.
 */
uint16_t logger_close() {
  if ( __log.onFile) {
	  // Init file logger
	  // @TODO support file logger
  }


  if ( __log.onSerial1) {
	  serial1_flush();
  }
  if ( __log.onSerial2) {
	  serial2_flush();
  }
  if ( __log.onDebug) {
	  debug_flush();
  }
  __log.ready=false;
  return __log.logConf;
}

/**
 * Print the log file over the serial line. As this is usually called during the
 * sleeping loop the SPIFF is supposed to be closed. The function try to manage this and
 * restore the initial state.
 */
void log_cat() {
  // @TODO support file logger
}

/**
 * Purge the log file
 * sleeping loop the SPIFF is supposed to be closed. The function try to manage this and
 * restore the initial state.
 */
void log_clean() {
  // @TODO support file logger
}

/**
 * Log an error according to the configuration on the different
 * possible logger
 */
void log_error(char *format, ...) {
#if ITSDK_LOGGER_CONF > 0
  va_list args;
  char 	fmtBuffer[LOGGER_MAX_BUF_SZ]; 				// buffer for log line formating before printing
  if ( __log.logError && __log.ready ) {
    va_start(args,format);
    vsnprintf(fmtBuffer,LOGGER_MAX_BUF_SZ,format,args);
    va_end(args);

    if ( __log.onSerial1 ) {
      serial1_print(fmtBuffer);
    }

    if ( __log.onSerial2 ) {
      serial2_print(fmtBuffer);
    }

    if ( __log.onDebug  ) {
      debug_print(DEBUG_PRINT_ERROR,fmtBuffer);
    }

    if ( __log.onFile ) {
      // @ TODO logfile_printf("%lu [error] ",time_get_ms());
      //logfile_print(fmtBuffer);
    }

  }
#endif
}

/**
 * Log a warning according to the configuration on the different
 * possible logger
 */
void log_warn(char *format, ...) {
#if ITSDK_LOGGER_CONF > 0
  va_list args;
  char 	fmtBuffer[LOGGER_MAX_BUF_SZ]; 				// buffer for log line formating before printing
  if ( __log.logWarn  && __log.ready ) {
    va_start(args,format);
    vsnprintf(fmtBuffer,LOGGER_MAX_BUF_SZ,format,args);
    va_end(args);

    if ( __log.onSerial1 ) {
    	serial1_print(fmtBuffer);
    }

    if ( __log.onSerial2 ) {
    	serial2_print(fmtBuffer);
    }

    if ( __log.onDebug ) {
    	debug_print(DEBUG_PRINT_WARNING,fmtBuffer);
    }

    if ( __log.onFile ) {
        // @ TODO logfile_printf("%lu [warn] ",time_get_ms());
        //logfile_print(fmtBuffer);
    }
  }
#endif
}


/**
 * Log a info according to the configuration on the different
 * possible logger
 */
void log_info(char *format, ...) {
#if ITSDK_LOGGER_CONF > 0
  va_list args;
  char 	fmtBuffer[LOGGER_MAX_BUF_SZ]; 				// buffer for log line formating before printing
  if ( __log.logInfo  && __log.ready ) {
    va_start(args,format);
    vsnprintf(fmtBuffer,LOGGER_MAX_BUF_SZ,format,args);
    va_end(args);

    if ( __log.onSerial1 ) {
    	serial1_print(fmtBuffer);
    }

    if ( __log.onSerial2 ) {
    	serial2_print(fmtBuffer);
    }

    if ( __log.onDebug ) {
    	debug_print(DEBUG_PRINT_INFO, fmtBuffer);
    }

    if ( __log.onFile ) {
        // @ TODO logfile_printf("%lu [info] ",time_get_ms());
        //logfile_print(__log.fmtBuffer);
    }
  }
#endif
}

/**
 * Log a debug according to the configuration on the different
 * possible logger
 */
void log_debug(char *format, ...) {
#if ITSDK_LOGGER_CONF > 0
  va_list args;
  char 	fmtBuffer[LOGGER_MAX_BUF_SZ]; 				// buffer for log line formating before printing
  if ( __log.logDebug  && __log.ready ) {
    va_start(args,format);
    vsnprintf(fmtBuffer,LOGGER_MAX_BUF_SZ,format,args);
    va_end(args);

    if ( __log.onSerial1 ) {
    	serial1_print(fmtBuffer);
    }

    if ( __log.onSerial2 ) {
    	serial2_print(fmtBuffer);
    }

    if ( __log.onDebug ) {
    	debug_print(DEBUG_PRINT_DEBUG,fmtBuffer);
    }

    if ( __log.onFile ) {
        // @ TODO logfile_printf("%lu [debg] ",time_get_ms());
        //logfile_print(__log.fmtBuffer);
    }
  }
#endif
}

/**
 * Log a debug according to the configuration on the different
 * possible logger
 */
void log_any(char *format, ...) {
#if ITSDK_LOGGER_CONF > 0
  va_list args;
  char 	fmtBuffer[LOGGER_MAX_BUF_SZ]; 				// buffer for log line formating before printing

  va_start(args,format);
  vsnprintf(fmtBuffer,LOGGER_MAX_BUF_SZ,format,args);
  va_end(args);

  if ( __log.onSerial1 ) {
	  serial1_print(fmtBuffer);
  }

  if ( __log.onSerial2 ) {
	  serial2_print(fmtBuffer);
  }

  if ( __log.onDebug ) {
	  debug_print(DEBUG_PRINT_ANY,fmtBuffer);
  }

  if ( __log.onFile ) {
	  // @ TODO logfile_printf("%lu [any ] ",time_get_ms());
	  //logfile_print(__log.fmtBuffer);
  }
#endif
}

