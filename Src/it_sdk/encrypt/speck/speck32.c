/* ==========================================================
 * speck32.c - SPECK32 encryption library
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 09 dec. 2018
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
 * Library for SPECK encryption
 * - from https://github.com/shreyasj2006/Speck32-64
 *
 * ==========================================================
 */
#include <it_sdk/config.h>
#include <it_sdk/encrypt/speck/speck.h>

/**
 * Encrypt a bloc of len data with the given key
 */
void speck32_encrypt(uint8_t * key, uint8_t * data, uint8_t len) {
	uint16_t subkeys[22];
	uint16_t l[24];

	subkeys[0] = ( key[6] << 8 ) + key[7];
	l[0] =  (key[4]<<8) + key[5];
	l[1] =  (key[2]<<8) + key[3];
	l[2] =  (key[0]<<8) + key[1];
	int m = 4;
	for(int i = 0; i < 21; ++i) {
	    uint32_t temp1, temp2, temp3;
	    temp1 = subkeys[i];
	    temp2 = l[i];
        l[i+m-1] = (uint16_t) ((temp1 + ((temp2 >> 7) | (temp2 << (16-7)))) ^ i);
	    temp3 = l[i+m-1];
        subkeys[i+1] = (uint16_t) ((uint16_t)((temp1 << 2) | (temp1 >> (16-2))) ^ temp3);
    }

	for ( int i = 0 ; i < len ; i+= 4) {
		uint32_t block = data[i];
		block = (block << 8) + data[i+1];
		block = (block << 8) + data[i+2];
		block = (block << 8) + data[i+3];

		uint16_t p1 = ( data[i  ] << 8 ) + data[i+1];
		uint16_t p2 = ( data[i+2] << 8 ) + data[i+3];

		uint32_t temp1 = (uint32_t)p1;
		uint32_t temp2 = (uint32_t)p2;
		uint32_t temp3;
        for(int i = 0; i < 22; ++i) {
            temp3 = (uint32_t)subkeys[i];
            p1 = (uint16_t) ((((temp1 >> 7) | (temp1 << (16-7))) + temp2) ^ temp3);
            temp1  = (uint32_t)p1;
            p2 = (uint16_t) (((temp2 << 2) | (temp2 >> (16-2))) ^ temp1);
            temp2 = (uint32_t)p2;
        }

        data[i]   = (p1 & 0xFF00) >> 8;
        data[i+1] = (p1 & 0xFF);
        data[i+2] = (p2 & 0xFF00) >> 8;
        data[i+3] = (p2 & 0xFF);
	}

	return;
}



