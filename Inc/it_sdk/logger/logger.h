/* ==========================================================
 * logger.h - 
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
 * 
 *
 * ==========================================================
 */

#ifndef IT_SDK_LOGGER_LOGGER_H_
#define IT_SDK_LOGGER_LOGGER_H_

#include <stdbool.h>
#include <stdint.h>

#define LOGGER_MAX_BUF_SZ             80

#define LOGGER_CONFIG_FILE_MASK       0xF000
#define LOGGER_CONFIG_SERIAL1_MASK    0x0F00
#define LOGGER_CONFIG_SERIAL2_MASK    0x00F0
#define LOGGER_CONFIG_DEBUGLNK_MASK   0x000F
#define LOGGER_CONFIG_DEBUG_LVL_MASK  0x8888
#define LOGGER_CONFIG_INFO_LVL_MASK   0x4444
#define LOGGER_CONFIG_WARN_LVL_MASK   0x2222
#define LOGGER_CONFIG_ERROR_LVL_MASK  0x1111

#define LOGGER_FILE_MAX_SIZE          200000    // 200k - Max log file size - after this size the log file is deleted

bool log_init(uint16_t config);
uint16_t log_close();
void log_error(char *format, ...);
void log_warn(char *format, ...);
void log_info(char *format, ...);
void log_debug(char *format, ...);
void log_any(char *format, ...);

void log_cat();
void log_clean();

typedef struct __s_log {
	  bool   	ready:1;         // Initialization has been done
	  bool   	logError:1;      // Error log level reported somewhere
	  bool   	logWarn:1;       // Warn log level reported somewhere
	  bool   	logInfo:1;       // Info log level reported somewhere
	  bool   	logDebug:1;      // Debug log level reported somewhere
	  bool   	onSerial1:1;     // Some logs are reported on Serial Line
	  bool   	onSerial2:1;     // Some logs are reported on Serial1 Line
	  bool   	onDebug:1; 	   	 // Some logs are reported on SoftwareSerial Line
	  bool   	onFile:1;        // Some logs are reported on Falsh file
	  uint16_t  logConf;  	   	 // Detailed log level
} __t_log;

#define log_info_array(name,array,sz)	{												\
									      log_info(name);								\
									      for ( int __i = 0; __i < sz ; __i++ ) {       \
									    	  log_info(" %02X",array[__i]);				\
									      }												\
										  log_info("\r\n");								\
										}

#endif /* IT_SDK_LOGGER_LOGGER_H_ */
