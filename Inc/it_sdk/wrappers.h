/* ==========================================================
 * wrappers.h - 
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
 * 
 *
 * ==========================================================
 */

#ifndef STM32L_SDK_WRAPPERS_H_
#define STM32L_SDK_WRAPPERS_H_

#include <stdbool.h>

// Serial wrappers
void serial1_flush();
void serial2_flush();
void debug_flush();

void serial1_print(char * msg);
void serial2_print(char * msg);
void debug_print(char * msg);
void logfile_print(char * msg);

void serial1_println(char * msg);
void serial2_println(char * msg);
void debug_println(char * msg);
void logfile_println(char * msg);

// watchdog
void wdg_setupWithMaxMs(uint32_t ms);
void wdg_refresh();

// eeprom
bool _eeprom_write(uint8_t bank, uint32_t offset, void * data, int len);
bool _eeprom_read(uint8_t bank, uint32_t offset, void * data, int len);

// adc
#define ADC_TEMPERATURE_ERROR		-500
int16_t adc_getTemperature();
uint16_t adc_getVdd();
uint16_t adc_getValue(uint32_t pin);

// misc_wrapper
void itsdk_reset();


#endif /* STM32L_SDK_WRAPPERS_H_ */
