/* ==========================================================
 * s2lp_idRetriever.c - S2LP IdRetriever (reverse ingineering) implementation
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 12 nov. 2018
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
 * This is a reverse on IDRetriever library if you want to migrate
 * from the ST EEPROM to another NVM storage
 *
 * ==========================================================
 */

#include <it_sdk/config.h>
#ifdef OUT
#if ITSDK_SIGFOX_LIB == __SIGFOX_S2LP

#include <it_sdk/itsdk.h>
#include <it_sdk/sigfox/sigfox.h>
#include <it_sdk/logger/logger.h>
#include <drivers/sigfox/sigfox_retriever.h>

#if ITSDK_SIGFOX_NVM_SOURCE	== __SFX_NVM_M95640
	#include <drivers/eeprom/m95640/m95640.h>
#endif



void f_l12(uint8_t *d, uint8_t *s) {

	for ( int i=0; i < 16 ; i++ ) {
		d[i]=s[i];
	}

}



void f_copy16bytes(uint8_t *d, uint8_t *s) {

	for ( int i=0; i < 16 ; i++ ) {
		d[i]=s[i];
	}

}


struct s_global * ptr;


void f_l37(uint8_t * buff, uint8_t * key, uint32_t len ) {
	// appel avec &strcut[36] / &Block2[16] / 16
	uint32_t v1;			// 0
	uint8_t key1[16]; 		// 4

	v1 = len & 0xF;
	f_copy16bytes(key1,key);
	s_0c8.key1 = key1;

	// Un peu bidon tout ca surtout avec len qui vaut 16
	// ai-je compris ?
	uint8_t * k1 = key1;
	uint8_t * k  = key;
	for ( int i = 0 ; i < len ; i+=16 ) {
		f_copy16bytes(k1,k);
		s_0c8.key1 = k1;
		f_l28();
		{ // l28
			r0 = 10
			f_l13(10);  // utilise les 2 strcutures
			r4 = 9
         do {
			f_l50() {
				r0 = v8000bc8;
				r0 += 60 ;


			}
			f_l39();
			r0 = r4
		    f_l13(9);
			f_l74();
			R4--
         } while ( r4 > 0 )
		 f_l50();
		 f_l39();
		 return 0;

		}

		=> R0 modifié au cas où
		f_l29(k1);
		s_0c8.key2 = k;
		k1+=16;
		k+=16;
	}



	f_l33(key1,key,len);


	//uint32_t v2;			// 20
	//uint32_t v3;			// 24
	//r6=0
    //r0=&v3;

}


struct s_s688 {
  uint8_t  d[4];		// 688, 689, 68A, 68B - 0
  uint8_t u68c[11][4];  // 4
  uint8_t e[16];  		// 8
  uint8_t v[64];    	// 64
  int8_t * v64;			// 0x688+64
} s688;
struct s_s688 * p688;
						// 20000008 pour le premier
							// data en 20...58->98
						// 2000012c pour le second


uint8_t s68c[];
void f_l33() {
	// Real Stack
	uint8_t	k[4];

    uint8_t * v1 = p688->v64;
    R0 recoit le pointeur vers structure en 20000008
	R4 recoit le pointeur vers une structure en 2000012C
	à revoir avec ce nouvel element...

    // Recopie 4x4 byte d'un pointeur vers une structure memoire interne
    int i = 0;
    while (  i < 3 ) {
		p688->u68c[4*i+0] = v1[4*i+0];
		p688->u68c[4*i+1] = v1[4*i+1];
		p688->u68c[4*i+2] = v1[4*i+2];
		p688->u68c[4*i+3] = v1[4*i+3];
		i++;
    }
    do {
	    memcpy(&k,&p688->u68c[4*(i-1)],4);
	    if ( i & 0x3  == 0 ) {
	    	int8_t j = k[0];
	    	k[0] = k[1];
	    	k[1] = k[2];
	    	k[2] = k[3];
	    	k[3] = j;

	    	k[0] = p688->u68c[k[0]+4];
	    	k[1] = p688->u68c[k[1]+4];
	    	k[2] = p688->u68c[k[2]+4];
	    	k[3] = p688->u68c[k[3]+4];

	    	k[0] ^= (&54c[i/4]);		// Une clef pour chaque bloc de 4 octets
	    }
	  	p688->u68c[4*i+0] = k[0] ^ p688->u68c[(4*i)-16+0];	// 4*3 = 12 - 16 ?!?
		p688->u68c[4*i+1] = k[1] ^ p688->u68c[(4*i)-16+1];  // donc en premier on
		p688->u68c[4*i+2] = k[2] ^ p688->u68c[(4*i)-16+2];  // tape sur d[x]
		p688->u68c[4*i+3] = k[3] ^ p688->u68c[(4*i)-16+3];
		i++;
 	} while ( i < 44 ) ;		// 11 * 32 bits i ou 4*i ?

}


struct s_s0c8 {
	uint8_t			unk0[60];		// 0 		- 0x0C8
	uint8_t 	*   key1;			// 60		- 0x104
	uint8_t		*	unk2;			// 64		- 0x108
	uint8_t		*   key2;			// 68		- 0x10C
} s_0c8;

void *	v8000bc8 = &s_0c8;		// 20000008



struct s_11f0 { // offset - 0x200000c8
	uint8_t 	unk0[4];			// 0		- 00 00 00 07
	uint8_t 	unk01;				// 4		- 0x07
	uint8_t		unk02[3];			// 5		- 00 00 00
	uint8_t		pac[8];				// 8 		- 36 61 3F 89 44 0B D6 47
	uint32_t	deviceID;			// 16		- BA 78 4D 00
	uint8_t		unk1[16];			// 20		- 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
	uint8_t		pkey[16];			// 36 privkey    - FB B3 7E 1D 8F FE 64 50 8E 1A 29 BE 5F 9D 31 E7
	uint8_t		unk11[8];			// 52		- 00 00 00 00 00 00 00 00
	uint8_t		unk12[4];			// 60		- AC 1E 00 20 => pointer - variable
	uint32_t  * key2;				// 64		- EC 00 00 20 => pointer => vers unk1+4
	uint32_t	unk2;				// 68 		- 18 1F 00 20 => pointer - variable
	uint32_t  * key1;				// 72		- DC 00 00 20 => pointer => vers un bout du devID ?
	uint32_t  * key3;				// 76		- EC 00 00 20 => pointer => vers unk1+4

	uint8_t	    pubKey[16];			// 80 		- 00 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF
	uint8_t		unk4[16];			// 96		- 01 23 45 67 89 AB CD EF 01 23 45 67 89 AB CD EF
									// @ 0x20000138
} v11f0;

struct s_11f0 * v8000ef8 = v11f0; // valeur 20000008

struct s_epromData {			// 32 bytes
	uint32_t	deviceId;		// 0
	uint8_t		pac[8];			// 4
	uint8_t		rcz;			// 12
	uint8_t		unk[3];			// 13
	uint8_t		key[16];		// 16
};



uint8_t enc_utils_retrieve_data(uint32_t * id, uint8_t * pac, uint8_t * rcz) {
	uint32_t			checksum;			// sp + 0
	uint32_t			s2lpConfData[2];	// sp + 4
	uint32_t			unk1[4];			// sp + 12
	uint32_t			unk2[4];			// sp + 28
	uint32_t			unk3;				// sp + 44
	struct s_epromData  epromData;			// sp + 48
											// sp + 80

	// Load the Data
	eeprom_m95640_read(&ITSDK_DRIVERS_M95640_SPI,0x1F0, 4, (uint8_t *)&checksum);
	eeprom_m95640_read(&ITSDK_DRIVERS_M95640_SPI,0x200, 32, (uint8_t *)epromData);
	eeprom_m95640_read(&ITSDK_DRIVERS_M95640_SPI,0x6, 8, (uint8_t *)s2lpConfData);

	// Ensure the data checksum is correct
	uint32_t sum = s2lpConfData[0];
	sum += s2lpConfData[1];
	for ( int i = 0 ; i < 8 ; i++){
		sum += ((uint32_t*)epromData)[i];
	}
	if ( sum != checksum ) {
		log_error("Eeprom configuration is invalid\r\n");
		return 1;
	}

	// Copy and verify deviceID
	v11f0.deviceID = epromData.deviceId;
	if ( v11f0.deviceID == 0 || v11f0.deviceID == 0xFFFFFFFF ) {
		v11f0.deviceID = 0;
		return 1;
	}
	*id = v11f0.deviceID;

	// Copy PAC
	for ( int i = 0 ; i < 8 ; i++ ) {
		pac[i] = epromData.pac[i];
		v11f0.pac = epromData.pac[i];
	}

	// Extract RCZ
	*rcz = ( (epromData.rcz & 0x0F) == 0 ) ? 1 : epromData.rcz & 0x0F;

	// Let's play with something else
	for ( int i = 0 ; i < 4 ; i++ ) {
		unk1[i]=v11f0.deviceID;
	}

	unk2[0] = v11f0.deviceID;
	unk2[1] = s2lpConfData[0];
	unk2[2] = s2lpConfData[1];
	unk2[3] = v11f0.deviceID;

	// Operation between RCZ & deviceID value .. ???
    if ( epromData.rcz < 0x87 ) {
    	((uint8_t *)&unk2[3])[3] |= epromData.rcz;
    } else {
    	((uint8_t *)&unk2[3])[3] &= epromData.rcz;
    }

    if ( epromData.unk[1] < 0x19 ) {
    	((uint8_t *)&unk2[3])[0] |= epromData.unk[1];
    } else {
    	((uint8_t *)&unk2[3])[0] &= epromData.unk[1];
    }

    if ( epromData.unk[2] < 0x80 ) {
    	((uint8_t *)&unk1[2])[0] |= epromData.unk[2];
    } else {
    	((uint8_t *)&unk1[2])[0] &= epromData.unk[2];
    }

    if ( epromData.unk[3] < 0x80 ) {
    	((uint8_t *)&unk1[2])[3] |= epromData.unk[3];
    } else {
    	((uint8_t *)&unk1[2])[3] &= epromData.unk[3];
    }

    uint8_t v = epromData.unk[3] & 0x0F;
    if ( v < 2  ) v = 2;
    if ( v > 13 ) v = 13;
    v11f0.unk01 = v;

    v11f0.key1 = &unk1[0];
    v11f0.key2 = &unk2[0];
    f_37(&v11f0.unk11[0],&epromData.key,16);



}



08000e74 <enc_utils_set_public_key>:
 8000e74:	4920      	ldr	r1, [pc, #128]	; (8000ef8 <.text_38>)
 8000e76:	2800      	cmp	r0, #0
 8000e78:	d004      	beq.n	8000e84 <enc_utils_set_public_key+0x10>
 8000e7a:	2010      	movs	r0, #16
 8000e7c:	000a      	movs	r2, r1
 8000e7e:	3250      	adds	r2, #80	; 0x50
 8000e80:	2301      	movs	r3, #1
 8000e82:	e002      	b.n	8000e8a <enc_utils_set_public_key+0x16>
 8000e84:	78c8      	ldrb	r0, [r1, #3]
 8000e86:	6cca      	ldr	r2, [r1, #76]	; 0x4c
 8000e88:	2300      	movs	r3, #0
 8000e8a:	708b      	strb	r3, [r1, #2]
 8000e8c:	e00b      	b.n	8000ea6 <.text_34>


#endif // ITSDK_SIGFOX_LIB test
#endif
