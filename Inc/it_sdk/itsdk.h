/* ==========================================================
 * itsdk.h - Common Headers
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
 *
 * ==========================================================
 */

#ifndef IT_SDK_ITSDK_H_
#define IT_SDK_ITSDK_H_

typedef enum {
	BOOL_FALSE=0,
	BOOL_TRUE,
} itsdk_bool_e;

#include <stdbool.h>
#include <stdint.h>
#include <it_sdk/config.h>
#include <it_sdk/wrappers.h>


// ------------------------------------------------------------------------
// Function every project needs to implement
void project_setup();
void project_loop();

// ------------------------------------------------------------------------
// SDK Public Interface
void itsdk_setup();
void itsdk_loop();
void itsdk_restart();

// Deprecated !!! DO NOT USE
// replaced by ITSDK_ERROR_REPORT
//void itsdk_error_handler(char * file, int line);

// -----
#define ITSDK_INVALID_VALUE_32B	0x0FFFFFFF
#define ITSDK_INVALID_VALUE_16B 0x7FFF

// ------------------------------------------------------------------------
// Tool.c
uint8_t itsdk_randomByte(void);
uint32_t itsdk_computeCRC32(const uint8_t *data, uint16_t length);
void itsdk_inlineCRC32_init();
uint32_t itsdk_inlineCRC32_next(uint32_t c, uint8_t size);
uint32_t itsdk_isqtr(uint32_t n);
uint32_t itsdk_pgcd(uint32_t a, uint32_t b);
char itdt_convertHalfInt2HexChar(uint8_t v,itsdk_bool_e upper);
void itdt_convertInt2HexChar(uint8_t v, char * dest, itsdk_bool_e upper);
void itdt_convertIntTab2Hex(char * dest, uint8_t * tab, int len, itsdk_bool_e upper);
bool itdt_isHexChar(char c, bool upper);
bool itdt_isHexString(char * str,int n,itsdk_bool_e upper);
uint8_t itdt_convertNumChar2Int(char c);
uint8_t itdt_convertHexChar2HalfInt(char c);
uint8_t itdt_convertHexChar2Int(char * v);
uint32_t itdt_convertHexChar8Int(char * v);
uint16_t itdt_convertHexChar4Int(char * v);
int16_t itdt_convertDecChar4Int(char * v);
uint16_t itdt_convertDecChar3UInt(char * v);
int32_t itdt_convertDecCharNInt(char * v, int sz);
void itdt_convertHexStr2IntTab(char * hexstr,uint8_t * tab, int len);
void itdt_macToString(char * str, uint8_t * mac);
uint32_t itdt_align_32b(uint32_t v);
uint8_t itdt_count_bits_1(uint32_t v);

#endif /* IT_SDK_ITSDK_H_ */
