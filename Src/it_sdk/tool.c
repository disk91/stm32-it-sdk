/* ==========================================================
 * tool.c - Misc tooling functions
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 16 sept. 2018
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
#include <it_sdk/itsdk.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/time/time.h>
#include <stdbool.h>

// =======================================================================================
// Random stuff
// =======================================================================================

uint8_t itsdk_randomByte(void) {
	// Von Neumann algorithm
	uint8_t v = 0;
	for ( int i = 0 ; i < 8 ; i++ ) {
		uint8_t a;
		do {
			a = itsdk_randomBit() | (itsdk_randomBit() << 1);
		} while ( a == 0 || a==3 );
		v = (v << 1) + (a >> 1);
	}
	return v;
}


// =======================================================================================
// CRC stuff
// =======================================================================================


/**
 * Return CRC32 value for data.
 */
uint32_t itsdk_computeCRC32(const uint8_t *data, uint16_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

/**
 * Compute a CRC from a stream data after data.
 * The size is the number of bit of the given value
 */
uint32_t __itsdk_crc;

void itsdk_inlineCRC32_init() {
	__itsdk_crc = 0xffffffff;
}

uint32_t itsdk_inlineCRC32_next(uint32_t c, uint8_t size) {
	for (uint32_t i = (1 << (size-1)) ; i > 0; i >>= 1) {
	   bool bit = __itsdk_crc & 0x80000000;
	   if (c & i) {
	      bit = !bit;
	   }
	   __itsdk_crc <<= 1;
	   if (bit) {
		   __itsdk_crc ^= 0x04c11db7;
	   }
	}
	return __itsdk_crc;
}

// =======================================================================================
// Integer sqrt
// =======================================================================================

/**
 * Integer sqrt, about 10x faster than double sqrt on a stm32L
 */
uint32_t itsdk_isqtr(uint32_t n) {
    uint32_t c = 0x8000;
    uint32_t g = 0x8000;
    for(;;) {
      if( g*g > n) g ^= c;
      c >>= 1;
      if(c == 0) return g;
      g |= c;
    }
}

// =======================================================================================
// PGCD
// =======================================================================================

uint32_t itsdk_pgcd(uint32_t a, uint32_t b) {
 uint32_t r;
 while (b != 0) {
     r = a%b;
     a = b;
     b = r;
 }
 return a;
}

// =======================================================================================
// Converters
// =======================================================================================

/* -----------------------------------------------------------
 * Convert a 0-16 value to a upper/lower Char
 */
char itdt_convertHalfInt2HexChar(uint8_t v,itsdk_bool_e upper) {
  if ( v >= 0 && v <= 9 ) return '0'+v;
  if ( v >= 10 && v <= 15 ) return (upper==BOOL_TRUE)?'A'+(v-10):'a'+(v+10);
  return 0;
}

/* -----------------------------------------------------------
 * Convert 0-9 char to 0-9 value
 */
uint8_t itdt_convertNumChar2Int(char c) {
	  if ( c >= '0' && c <= '9' ) return c-'0';
	  return 0xFF;
}

/* -----------------------------------------------------------
 * Convert a 0-F char to a 0-16 value
 */
uint8_t itdt_convertHexChar2HalfInt(char c) {
  if ( c >= '0' && c <= '9' ) return c-'0';
  if ( c >= 'a' && c <= 'f' ) return 10+c-'a';
  if ( c >= 'A' && c <= 'F' ) return 10+c-'A';
  return 0;
}


/* -----------------------------------------------------------
 * Convert a 0-256 value to a 2 byte upper/lower case string
 */
void itdt_convertInt2HexChar(uint8_t v, char * dest, itsdk_bool_e upper) {
  uint8_t q0 = (v & 0xF0) >> 4;
  uint8_t q1 = (v & 0x0F);
  dest[0] = itdt_convertHalfInt2HexChar(q0,upper);
  dest[1] = itdt_convertHalfInt2HexChar(q1,upper);
}

/* -----------------------------------------------------------
 * Convert a "0"-"FF" value to 0-255 uint8_t value
 */
uint8_t itdt_convertHexChar2Int(char * v) {
  uint8_t q0 = itdt_convertHexChar2HalfInt(v[0]);
  uint8_t q1 = itdt_convertHexChar2HalfInt(v[1]);
  return (q0*16)+q1;
}

/* -----------------------------------------------------------
 *  Convert a 32bit hex string value into uint32_t value
 */
uint32_t itdt_convertHexChar8Int(char * v) {
  uint32_t ret = itdt_convertHexChar2Int(&v[0]);
  ret <<= 8;
  ret += itdt_convertHexChar2Int(&v[2]);
  ret <<= 8;
  ret += itdt_convertHexChar2Int(&v[4]);
  ret <<= 8;
  ret += itdt_convertHexChar2Int(&v[6]);
  return ret;
}

/* -----------------------------------------------------------
 *  Convert a 16bit hex string value into uint16_t value
 */
uint16_t itdt_convertHexChar4Int(char * v) {
  uint16_t ret = itdt_convertHexChar2Int(&v[0]);
  ret <<= 8;
  ret += itdt_convertHexChar2Int(&v[2]);
  return ret;
}

/* -----------------------------------------------------------
 *  Convert a 4 char decimal (+ sign) string value into uint16_t value
 */
int16_t itdt_convertDecChar4Int(char * v) {
   int32_t ret = itdt_convertDecCharNInt(v,4);
   if ( ret == ITSDK_INVALID_VALUE_32B ) return ITSDK_INVALID_VALUE_16B;
   return ret;
}

/* -----------------------------------------------------------
 *  Convert a 3 char decimal string value into uint8_t value
 *  Format 001
 */
uint16_t itdt_convertDecChar3UInt(char * v) {

  int32_t ret = itdt_convertDecCharNInt(v,3);
  if ( ret == ITSDK_INVALID_VALUE_32B ) return ITSDK_INVALID_VALUE_16B;
  return (uint16_t)ret;
}


/**
 * Convert a signed decimal number from a string to
 * an Int32 value.
 */
int32_t itdt_convertDecCharNInt(char * v, int sz) {
	int sign = 1;
	if ( *v == '-' ) {
	   sign = -1;
	   v++;
	   sz--;
	}
	uint16_t ret = 0;
	for ( int i = 0 ; i < sz ; i++ ) {
	  ret *= 10;
	  uint8_t c = itdt_convertNumChar2Int(*v);
	  if ( c == 0xFF ) return ITSDK_INVALID_VALUE_32B;
	  v++;
	  ret+= c;
	 }
	 ret *= sign;
	 return ret;
}

/* -----------------------------------------------------------
 * Convert a 8bytes table to Upper Hex upper/lower string
 */
void itdt_convertIntTab2Hex(char * dest, uint8_t * tab, int len, itsdk_bool_e upper) {
  int i;
  for ( i = 0; i < len ; i++ ) {
	  itdt_convertInt2HexChar(tab[i],&dest[2*i],upper);
  }
  dest[2*len]='\0';
}

/* ----------------------------------------------------------
 * Convert a Char String to a hex value tab entries
 */
void itdt_convertHexStr2IntTab(char * hexstr,uint8_t * tab, int len) {

  int i;
  for ( i = 0; i < len ; i++ ) {
    tab[i] = itdt_convertHexChar2Int(&hexstr[2*i]);
  }

}

/* ----------------------------------------------------------
 * Verify a char is an Hex Char
 */
bool itdt_isHexChar(char c, bool upper) {
  if (    (c >= '0' && c <= '9' )
       || (c >= 'A' && c <= 'F' )
     || (!upper   && c >= 'a' && c <= 'f')
    ) {
    return true;
  }
  return false;
}

/* ----------------------------------------------------------
 * Verify a string is a valid Hex string with given size
 */
bool itdt_isHexString(char * str,int n,itsdk_bool_e upper) {
  int i = 0;
  while ( i < n && str[i] != 0 ) {
    if (    (str[i] >= '0' && str[i] <= '9' )
       || (str[i] >= 'A' && str[i] <= 'F' )
       || (!upper && str[i] >= 'a' && str[i] <= 'f')
       ) {
      i++;
    } else {
      return false;
    }
  }
  return ( i == n )?true:false;
}

/* ----------------------------------------------------
 *  Convert a 6 x uint8_t mac data into a String
 *  XX:XX:XX:XX:XX:XX into the str buffer. The str
 *  buffer size must be 18 bytes long or more
 */
void itdt_macToString(char * str, uint8_t * mac) {
  for ( int j = 0 ; j < 6 ; j++ ) {
	  itdt_convertInt2HexChar(mac[j],&str[3*j],true);
     if ( j < 5 ) str[3*j+2]=':';
  }
  str[17]='\0';
}

/* ---------------------------------------------------
 * Align a value on the next 32b value
 * This is for NVM size whare offset need to be aligned
 * on 32b values
 */
uint32_t itdt_align_32b(uint32_t v) {
	if ( (v & 3) != 0 ) {
		v &= 0xFFFFFFFC;
		v += 4;
	}
	return v;
}


/* ---------------------------------------------------
 * Count the number of bit at 1 in a given word
 */
uint8_t itdt_count_bits_1(uint32_t v) {
	uint8_t ret = 0;
	while ( v > 0 ) {
		if ( v & 1 ) ret++;
		v >>= 1;
	}
	return ret;
}

