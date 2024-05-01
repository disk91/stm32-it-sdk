/* ==========================================================
 * eeprom.c - Manage NVM (EEPROM)
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
#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32WLE
#include <string.h>

#include <it_sdk/wrappers.h>
#include <stm32l_sdk/eeprom/eeprom.h>
#include <it_sdk/debug.h>
#include <stdbool.h>
#include <it_sdk/logger/error.h>


#define EEPROM_START_ADDR  ( EEPROM_END_ADDR -  (EEPROM_TOTAL_PAGES * EEPROM_PAGE_SIZE) )
#define EEPROM_SIZE ITSDK_EPROM_SIZE


// ================================================================
// PAGE MANAGEMENT
// ================================================================

#if ( EEPROM_SIZE_WITH_OVERHEAD % EEPROM_PAGE_SIZE ) == 0
   #define EEPROM_TOTAL_PAGES  (( EEPROM_SIZE_WITH_OVERHEAD / EEPROM_PAGE_SIZE ))
#else
   #define EEPROM_TOTAL_PAGES  (( EEPROM_SIZE_WITH_OVERHEAD / EEPROM_PAGE_SIZE ) + 1 )
#endif
#define EEPROM_START_PAGE  ( EEPROM_START_ADDR / EEPROM_PAGE_SIZE )

// Get the starting address of a page
#define EEPROM_PAGE_ADDR(__p__) (EEPROM_START_ADDR + (__p__) * EEPROM_PAGE_SIZE )


typedef struct {
							// 64 bit header structure
	uint16_t magic;			// identify a EEPROM page

	uint16_t aging:14;		// number of time this page has been cleared (could be used to indicated the page have defect with 0x3FFF in a later version)
	uint16_t reserved_1:2;	// for a later usage

	uint8_t version;		// EEPROM version for later migration
	uint8_t reserved_2[3];	// for a later usage

	uint64_t state_h;		// State High Word
	uint64_t state_l;		// State Low Word
	uint64_t reserved_3;	// alignement to 4x 64b

} __eeprom_page_header_t;	// Total Size => 32B 4x64b

// Magic to identify the EEPROM page initialization
#define EEPROM_MAGIC	0x916E

// Current EEPROM Management version
#define EEPROM_VERSION 	0x01

// Different possible page state ( only 0xFF -> Any -> 0X00 is possible for a 64b block)
#define EEPROM_PAGE_STATE_FREE_H		0xFFFFFFFFFFFFFFFFL
#define EEPROM_PAGE_STATE_FREE_L		0xFFFFFFFFFFFFFFFFL

#define EEPROM_PAGE_STATE_INIT_EMPTY_H	0xFFFFFFFFFFFFFFFFL
#define EEPROM_PAGE_STATE_INIT_EMPTY_L	0x9999999999999999L

#define EEPROM_PAGE_STATE_MOVE_IN_H		0xFFFFFFFFFFFFFFFFL
#define EEPROM_PAGE_STATE_MOVE_IN_L		0x0000000000000000L

#define EEPROM_PAGE_STATE_READY_H		0xA55AA55AA55AA55AL
#define EEPROM_PAGE_STATE_READY_L		0x0000000000000000L

#define EEPROM_PAGE_STATE_MOVE_OUT_H	0x0000000000000000L
#define EEPROM_PAGE_STATE_MOVE_OUT_L	0x0000000000000000L

typedef enum  {
	STATE_FREE, STATE_INIT_EMPTY, STATE_MOVE_IN, STATE_READY, STATE_MOVE_OUT
} ___eeprom_state_t;

// Aging
#define EEPROM_PAGE_MAX_AGE	 	0x3FFE


/**
 * Clear a page, use the eeprom driver page numbering to be mapped with STM32 Page numbering
 */
bool __eeprom_page_clear(int page) {
	FLASH_EraseInitTypeDef s_eraseinit;
	uint32_t page_error = 0U;

	s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
	s_eraseinit.NbPages     = 1;
	s_eraseinit.Page        = EEPROM_START_PAGE+page;
	uint32_t r = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
	if ( r != HAL_OK) {
		#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
			log_error("Failed to clear page %d eq %d \r\n",page,EEPROM_START_PAGE+page);
		#endif
		return false;
	}
	return true;
}


/**
 * Init one page, if the page has been initialized previously clear it and recreate it
 * If the page is fresh new, init it from scratch. As Status bits can be flipped 64 blocs
 * per 64 bocks, manage specific byte change. Protect aging over changes
 * As a paramete, the final expected state for the page is provided.
 */
bool __eeprom_page_init(int page, ___eeprom_state_t targetState ) {
	if ( page > EEPROM_TOTAL_PAGES ) return false;

	// Compose the new page header
	__eeprom_page_header_t nh;
	nh.magic = EEPROM_MAGIC;
	nh.version = EEPROM_VERSION;
	nh.aging = 0;
	switch(targetState) {
		default:
		case STATE_FREE:
			nh.state_h = EEPROM_PAGE_STATE_FREE_H;
			nh.state_l = EEPROM_PAGE_STATE_FREE_L;
			break;
		case STATE_INIT_EMPTY:
			nh.state_h = EEPROM_PAGE_STATE_INIT_EMPTY_H;
			nh.state_l = EEPROM_PAGE_STATE_INIT_EMPTY_L;
			break;
		case STATE_MOVE_IN:
			nh.state_h = EEPROM_PAGE_STATE_MOVE_IN_H;
			nh.state_l = EEPROM_PAGE_STATE_MOVE_IN_L;
			break;
		case STATE_READY:
			nh.state_h = EEPROM_PAGE_STATE_READY_H;
			nh.state_l = EEPROM_PAGE_STATE_READY_L;
			break;
		case STATE_MOVE_OUT:
			nh.state_h = EEPROM_PAGE_STATE_MOVE_OUT_H;
			nh.state_l = EEPROM_PAGE_STATE_MOVE_OUT_L;
			break;
	}
	nh.reserved_1 = 3;
	nh.reserved_2[0] = 0xFF;nh.reserved_2[1] = 0xFF;nh.reserved_2[2] = 0xFF;
	nh.reserved_3 = 0xFFFFFFFFFFFFFFFFL;

	// Get the current page header
	uint32_t pAddr = EEPROM_PAGE_ADDR(page);
	__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);

	// In case the page is already in the expected status, does not change it and return success,
	// this is a way to test a state also
	if ( h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION &&  h.state_h == nh.state_h && h.state_l == nh.state_l ) return true;

	#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
		log_debug("EEprom init page %d from %04X:%04X to %04X:%04X\r\n",page,(uint16_t)h.state_h,(uint16_t)h.state_l,(uint16_t)nh.state_h,(uint16_t)nh.state_l);
	#endif

	// There are two transitions where we need to clear the page
	// target is FREE_PAGE or INIT_EMPTY or Source is not initialized
	if (   ( h.magic != EEPROM_MAGIC || h.version != EEPROM_VERSION )				// STATE_FREE equivalent
		|| targetState == STATE_FREE												// STATE_FREE
		|| targetState == STATE_INIT_EMPTY
	) {
		if ( h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION ) {
			// page have been init previsouly, keep aging and increase it
			nh.aging = (h.aging == EEPROM_PAGE_MAX_AGE)?EEPROM_PAGE_MAX_AGE:h.aging+1;
			if ( nh.aging == EEPROM_AGING_REPORT ) {
				#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
					log_warn("EEprom Page %d reach the aging limit\r\n",page);
				#endif
				// call a user callback function to inform the user level of this potential problem
				__eeprom_onFlashErrorCallback( EEPROM_WARN_PAGE_AGE_IS_HIGH );

			}
		}
		// clear the page
		if ( ! __eeprom_page_clear(page) ) {
			#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
				log_error("EEprom Page %d failed to clear\r\n",page);
			#endif
			__eeprom_onFlashErrorCallback( EEPROM_ERR_FAILED_TO_RESET_PAGE );
			return false;
		}
	} else {
		nh.aging = h.aging;
	}


	// Write the header, only change the w64 that are modified, so the state change
	// will be possible if we respect the allowed transitions
	h = (*(volatile __eeprom_page_header_t*)pAddr);
	uint64_t * _target = (uint64_t *)&nh;
	uint64_t * _source = (uint64_t *)&h;
	uint32_t _pAddr = pAddr;
	for ( int i = 0 ; i < sizeof(__eeprom_page_header_t) ; i+= sizeof(uint64_t) ) {
		if ( *_target != *_source ) {	// this includes 0xFF...FF will not be written
			if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _pAddr, *_target) != HAL_OK ) {
				// write failed, try again
				if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _pAddr, *_target) != HAL_OK ) {
					// error will be reported on read verification
					break;
				}
			}
		}
		_target++;
		_source++;
		_pAddr+=sizeof(uint64_t);
	}

	// verify the header
	h = (*(volatile __eeprom_page_header_t*)pAddr);
	uint32_t * _t = (uint32_t*)&nh;
	uint32_t * _r = (uint32_t*)&h;
	for ( int i = 0 ; i < sizeof(__eeprom_page_header_t) ; i+= sizeof(uint32_t) ) {
		if ( *_t != *_r ) {
			// failure
			#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
				log_error("EEprom Page init failure for page %d\r\n",page);
			#endif
			return false;
		}
		_t++;
		_r++;
	}
	return  true;
}

/**
 * Return true when a page state is READY, page is a 2K page
 * 0 to 256 as a maximum
 */
bool __eeprom_is_page_ready( uint8_t page ) {
	if ( page >= EEPROM_TOTAL_PAGES ) return false; // out of range

	uint32_t pAddr = EEPROM_PAGE_ADDR(page);
	__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);

	return (   h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION
		    && h.state_h == EEPROM_PAGE_STATE_READY_H &&  h.state_l == EEPROM_PAGE_STATE_READY_L );
}




/**
 * Find a free page and return it's number or 0xFF if not found
 * The returned page will a in the INIT state
 */
uint8_t __eeprom_get_free_page() {
	for ( int page = 0; page < EEPROM_TOTAL_PAGES ; page++ ) {
		uint32_t pAddr = EEPROM_PAGE_ADDR(page);
		__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
		if ( ( h.magic == 0xFFFF && h.version == 0xFF ) ) {
			// init the page before return it
			if ( ! __eeprom_page_init(page,STATE_INIT_EMPTY) ) {
				#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
				 log_error("EEPROM impossible to init a free page page %d\r\n",page);
				#endif
				continue; // try another one
			}
			return page;
		}
		if (   h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION
			&& (    ( h.state_h == EEPROM_PAGE_STATE_INIT_EMPTY_H && h.state_l == EEPROM_PAGE_STATE_INIT_EMPTY_L)
				 || ( h.state_h == EEPROM_PAGE_STATE_FREE_H && h.state_l == EEPROM_PAGE_STATE_FREE_L)
			   )
		) {
			return page;
		}
	}
	return 0xFF;
}


/**
 * Count free pages and return how many exists
 */
uint8_t __eeprom_count_free_page() {
	uint8_t c = 0;
	for ( int page = 0; page < EEPROM_TOTAL_PAGES ; page++ ) {
		uint32_t pAddr = EEPROM_PAGE_ADDR(page);
		__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
		if (   ( h.magic == 0xFFFF && h.version == 0xFF )
			|| (   h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION
			    && (    ( h.state_h == EEPROM_PAGE_STATE_INIT_EMPTY_H && h.state_l == EEPROM_PAGE_STATE_INIT_EMPTY_L)
				     || ( h.state_h == EEPROM_PAGE_STATE_FREE_H && h.state_l == EEPROM_PAGE_STATE_FREE_L)
			       )
			   )
		) {
			c++;
		}
	}
	return c;
}

// ================================================================
// LINE MANAGEMENT
// ================================================================

// Address field is only even addresses so 2^0 bit is removed ; Address 0 is prohibited so expected offset is incremented
// the addresses are for bloc of 14 Bytes so we align address on the first address of a such block
#define EEPROM_ADDR_TO_LINE_ADDR(__addr__) (( ((__addr__) / _EEPROM_BYTE_PER_LINE) ) + 1)

// Reversed operation
#define EEPROM_LINE_ADDR_TO_ADDR(__addr__) (((__addr__) - 1) * _EEPROM_BYTE_PER_LINE)

#define _EEPROM_BYTE_PER_LINE	14

// Get the starting address of lines in a page
#define EEPROM_FIRST_LINE_ADDR_IN_PAGE(__p__) ( EEPROM_START_ADDR + (__p__) * EEPROM_PAGE_SIZE + sizeof(__eeprom_page_header_t) )



typedef struct {
							// 16 control word
	uint16_t addr:10;		// 10 bits internal bloc addresse * 14 + 14
	uint16_t updates:2;		// 2 bits for number of time the line has been overided (max 3 - for identifing 0)
	uint16_t check:4;		// 4 bits "checksum" - count 1 bits


	uint8_t  data[_EEPROM_BYTE_PER_LINE];		// 14 bytes data
} __eeprom_line_t;			// Total Size => 16B ->  2x64b


/**
 * search a line over the different page, if > EEPROM_START_ADDR, returns the physical address where found or the first empty line.
 * if < EEPROM_START_ADDR, means not found return the new page number where the line can be added or the page to be created when this page
 * is not ready. If the EEPROM is full and no more page can be created, value 0xFF is returned.
 */
uint32_t __eeprom_find_virtual_line( uint32_t vAddr ) {

	// Floor the vAddr to match the line offset
	uint32_t _vAddr = EEPROM_ADDR_TO_LINE_ADDR(vAddr);
	uint8_t firstPageWithEmptyLine = 0xFF;
	uint8_t freePages = 0;
	uint8_t firstFreePage = 0xFF;

	for ( int page = 0; page < EEPROM_TOTAL_PAGES ; page ++ ) {
		// check if init, else need to init a new one to add this entry
		if ( __eeprom_is_page_ready(page) ) {
			// search in page
			uint32_t lAddr = EEPROM_FIRST_LINE_ADDR_IN_PAGE(page);
			for ( int i = 0 ; i < EEPROM_LINE_PER_PAGE ; i++ ) {
				__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr);	// to optimize, here we read the whole line but we just need addr
				if ( l.addr == 0x3FF ) {
					if ( firstPageWithEmptyLine == 0xFF ) firstPageWithEmptyLine = page;
					break;
				}
				if ( l.addr == _vAddr ) return lAddr;
				lAddr += sizeof(__eeprom_line_t);
			}
		} else {
			// as we search in all the pages, let's identify a potential target to store the line if not existing
			// making sure we keep a page for compaction
			freePages++;
			if (firstFreePage==0xFF) { firstFreePage = page; }
		}
	}
	// when not found, we return a ready page where to store the line
	if ( firstPageWithEmptyLine != 0xFF ) return firstPageWithEmptyLine;
	// if not we need to init a new page if exists
	// but if we just have a single one, we need to compact memory first
	if ( freePages == 1 ) return 0xFF;
	// else we return the page to init
	return firstFreePage;

}

/**
 * search for an empty line where we can write a new line.
 * if < EEPROM_START_ADDR, means not found return the new page number where the line can be added or the page to be created when this page
 * is not ready. If the EEPROM is full and no more page can be created, value 0xFF is returned.
 * If pageSearch parameter is 0xFF, search in all the pages, if different, it target a page where we know an empty line exists
 */
uint32_t __eeprom_find_empty_line(uint8_t pageSearch) {

	// search of a free line from the initialized pages
	int page = (pageSearch==0xFF)?0:pageSearch;

	while ( page < EEPROM_TOTAL_PAGES ) {
		// check if init, else need to init a new one to add this entry
		if ( __eeprom_is_page_ready(page) ) {
			// search in page
			uint32_t lAddr = EEPROM_FIRST_LINE_ADDR_IN_PAGE(page);
			for ( int i = 0 ; i < EEPROM_LINE_PER_PAGE ; i++ ) {
				__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr);	// to optimize, here we read the whole line but we just need addr
				if ( l.addr == 0x3FF ) return lAddr;
				lAddr += sizeof(__eeprom_line_t);
			}
		}
		page++;
	}

	// if not found, let see if we have a free page available
	if ( __eeprom_count_free_page() > 1 ) {
	   return __eeprom_get_free_page();
	}
	return 0xFF; // we need to call garbage collector to compact it

}



/**
 * Verification code for the line, including the address field and the data field
 * rest is not mandatory in check
 */
uint8_t __eeprom_compute_check(__eeprom_line_t * line) {
	uint16_t a = line->addr;
	uint8_t one = 0;
	for ( int i = 0 ; i < 16 ; i++ ) {
		one += a & 1;
		a >>= 1;
	}
	for ( int k = 0 ; k < _EEPROM_BYTE_PER_LINE ; k++ ) {
		uint8_t v = line->data[k];
		for ( int i = 0 ; i < 8 ; i++ ){
			one += v & 1;
			v >>= 1;
		}
	}
	return one & 0xF;	// only 4 bits considered
}


/**
 * Clear a line by writing 0x000 in the line 64bits bloc per 64bits blocs
 *
 */
void __eeprom_clear_line(uint32_t _lAddr) {
	// write the line per 64 bits blocks
	for ( int i = 0 ; i < sizeof(__eeprom_line_t) ; i+= sizeof(uint64_t) ) {
		if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _lAddr, 0L) != HAL_OK ) {
			// write failed, try again
			if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _lAddr, 0L) != HAL_OK ) {
				#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
					log_warn("Clear Failure at address %08X with value (%X)\r\n",_lAddr,HAL_FLASH_GetError());
				#endif
			}
		}
		_lAddr+=sizeof(uint64_t);
	}
}

/**
 * Write a line at a given address, verify it and return true if everything is ok
 * if not, burn the line for not being reused shortly and return false
 */
bool __eeprom_write_and_verify(__eeprom_line_t * target, uint32_t lAddr) {
	// write the line per 64 bits blocks
	uint64_t * _target = (uint64_t *)target;
	uint32_t _lAddr = lAddr;
	for ( int i = 0 ; i < sizeof(__eeprom_line_t) ; i+= sizeof(uint64_t) ) {
		if ( *_target != 0xFFFFFFFFFFFFFFFFL ) {
			// don't write if default value FF..FF because if done, can't write 0 later
			if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _lAddr, *_target) != HAL_OK ) {
				// write failed, try again
				if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _lAddr, *_target) != HAL_OK ) {
					// error will be reported on read verification
					#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
						log_warn("Write Failure at address %08X with value (%X)\r\n",_lAddr,HAL_FLASH_GetError());
					#endif
					break;
				}
			}
		}
		_target++;
		_lAddr+=sizeof(uint64_t);
	}

	// verify the line if difference report it, clear the line and a retry will be performed on a different zone
	__eeprom_line_t result = (*(volatile __eeprom_line_t*)lAddr);
	uint32_t * _t = (uint32_t*)target;
	uint32_t * _r = (uint32_t*)&result;
	bool failure = false;
	for ( int i = 0 ; i < sizeof(__eeprom_line_t) ; i+= sizeof(uint32_t) ) {
		if ( *_t != *_r ) {
			// failure
			// ITSDK_ERROR_REPORT(ITSDK_ERROR_EERPOM_WRITEFAILED,0); dangerous, risk of loop
			#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
				log_warn("Verif Failure at address %08X for vAddress %08X\r\n",lAddr,target->addr);
			#endif
			failure = true;
			break;
		}
		_t++;
		_r++;
	}

	// in case it is different, the failure variable will force a retry into a different zone
	// now we try to clean it
	if ( failure ) {
		__eeprom_clear_line(lAddr);
		return false;
	}
	return true;
}


/**
 * Return true when the page is empty (no line into it)
 */
bool __eeprom_is_empty_page(int page) {
	uint32_t lAddr = EEPROM_FIRST_LINE_ADDR_IN_PAGE(page);
	for ( int i = 0 ; i < EEPROM_LINE_PER_PAGE ; i++ ) {
		__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr);	// to optimize, here we read the whole line but we just need addr
		if ( l.addr != 0x3FF ) return false;
		lAddr += sizeof(__eeprom_line_t);
	}
	return true;
}

// ===============================================================================
// EEPROM STATS
// ===============================================================================


typedef struct {
	uint8_t 	pages;				// max number of pages
	uint8_t		free_pages;			// pages with state FREE
	uint8_t		init_empty_pages;	// pages with state INIT_EMPTY
	uint8_t 	move_in_pages;		// pages with state MOVE_INT
	uint8_t		ready_pages;		// pages with state READY
	uint8_t		move_out_pages;		// pages with state MOVE_OUT
	uint32_t 	avg_aging;			// average aging for init pages

	uint16_t 	allocated_lines;	// total allocated (in use) lines
	uint16_t 	free_lines;			// total line never allocated
	uint16_t	trashed_lines;		// total lines cleared after being override somewhere else
	uint8_t		untouched_lines;	// total lines never override
	uint8_t		trashed_lines_per_page[EEPROM_TOTAL_PAGES];
	uint8_t		free_lines_per_page[EEPROM_TOTAL_PAGES];
	uint8_t		untouched_lines_per_page[EEPROM_TOTAL_PAGES];
} __eeprom_stat_t;


/**
 * Compute the EEPROM statistics, this is used for display info
 * but also to manage the garbage collection
 */
void __eeprom_get_stat(__eeprom_stat_t * s) {
	bzero(s,sizeof(__eeprom_stat_t));
	s->pages = EEPROM_TOTAL_PAGES;
	uint32_t aging = 0;
	for ( int page = 0; page < EEPROM_TOTAL_PAGES ; page ++ ) {
		uint32_t pAddr = EEPROM_PAGE_ADDR(page);
		__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
		if ( h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION ) {
			if ( h.state_h == EEPROM_PAGE_STATE_READY_H && h.state_l == EEPROM_PAGE_STATE_READY_L ) {
				s->ready_pages++;
			} else if ( h.state_h == EEPROM_PAGE_STATE_FREE_H && h.state_l == EEPROM_PAGE_STATE_FREE_L ) {
				s->free_pages++;
				s->free_lines += EEPROM_LINE_PER_PAGE;
				s->free_lines_per_page[page] += EEPROM_LINE_PER_PAGE;
			} else if ( h.state_h == EEPROM_PAGE_STATE_INIT_EMPTY_H && h.state_l == EEPROM_PAGE_STATE_INIT_EMPTY_L ) {
				s->init_empty_pages++;
				s->free_lines += EEPROM_LINE_PER_PAGE;
				s->free_lines_per_page[page] += EEPROM_LINE_PER_PAGE;
			} else if ( h.state_h == EEPROM_PAGE_STATE_MOVE_IN_H && h.state_l == EEPROM_PAGE_STATE_MOVE_IN_L ) {
				s->move_in_pages++;
			} else if ( h.state_h == EEPROM_PAGE_STATE_MOVE_OUT_H && h.state_l == EEPROM_PAGE_STATE_MOVE_OUT_L ) {
				s->move_out_pages++;
			}
			aging+=h.aging;

			if (    ( h.state_h == EEPROM_PAGE_STATE_READY_H && h.state_l == EEPROM_PAGE_STATE_READY_L )
				 || ( h.state_h == EEPROM_PAGE_STATE_MOVE_OUT_H && h.state_l == EEPROM_PAGE_STATE_MOVE_OUT_L )
			) {
				// this is active data
				uint32_t lAddr = EEPROM_FIRST_LINE_ADDR_IN_PAGE(page);
				for ( int i = 0 ; i < EEPROM_LINE_PER_PAGE ; i++ ) {
					__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr);
					if( l.addr == 0x3FF ) {
						// free line
						s->free_lines++;
						s->free_lines_per_page[page]++;
					} else if (l.addr == 0x000 ) {
						// trashed line
						s->trashed_lines++;
						s->trashed_lines_per_page[page]++;
					} else {
						// in use
						s->allocated_lines++;
						if ( l.updates == 0x0 ) {
							s->untouched_lines_per_page[page]++;
						}
					}
					lAddr += sizeof(__eeprom_line_t);
				}
			}

		} else {
			s->free_pages++;
			s->free_lines += EEPROM_LINE_PER_PAGE;
			s->free_lines_per_page[page] += EEPROM_LINE_PER_PAGE;
		}
	}
}

/**
 * Displays the eeprom information
 */
void _eeprom_info() {
	log_info("EEprom start address: 0x%08X\r\n",EEPROM_START_ADDR);
	log_info("EEprom pages: %d\r\n",EEPROM_TOTAL_PAGES);
	__eeprom_stat_t s;
	__eeprom_get_stat(&s);
	log_info("EEprom stats:\r\n");
	log_info(" - Pages Tot(%d) Rdy(%d) Free(%d)\r\n",s.pages,s.ready_pages,s.pages-s.ready_pages);
	log_info(" - Lines Tot(%d) Rdy(%d) Free(%d) Trash(%d)\r\n",
			EEPROM_TOTAL_PAGES*EEPROM_LINE_PER_PAGE,
			s.allocated_lines,s.free_lines, s.trashed_lines
	);
	for (int i = 0 ; i < EEPROM_TOTAL_PAGES ; i++ ) {
		uint32_t pAddr = EEPROM_PAGE_ADDR(i);
		__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);

		log_info("P[%02d] Age(%04d) St(%01X:%01X) Free(%03d) Trash(%03d) Untouched(%03d)\r\n",
				i,h.aging, (int8_t)h.state_h & 0xF, (int8_t)h.state_l & 0xF,
				s.free_lines_per_page[i], s.trashed_lines_per_page[i], s.untouched_lines_per_page[i]
		);
	}

}

// ===============================================================================
// GARBAGE COLLECTION
// ===============================================================================

// For test purpose, get the ability to simulate crashes
// 1   		- crash after a page is switched in move out state
// 2/3 		- crash after a page switched in move_in, 2 will be the first one, 3 will be the second one
// 10-99	- crash after a certain number of transfered line (10 - 0 up to 99 - 89 )
// 100		- crash before transfer terminate
// 101		- crash in middle of transfer terminate
#ifdef __EEPROM_WITH_TEST
   static uint8_t injectCrash = 0;
#endif

/**
 * Once a group of pages have been transfered to another page, we can commit these
 * modifications by moving move_in page to ready and move_out page to init_empty
 */
bool __eeprom_transfer_to_terminate() {

	#ifdef __EEPROM_WITH_TEST
		if ( injectCrash == 100 ) return false;
	#endif

	// The first step is to commit MOVE_IN page into READY page
	for ( int page = 0; page < EEPROM_TOTAL_PAGES ; page++ ) {
		uint32_t pAddr = EEPROM_PAGE_ADDR(page);
		__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
		if ( h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION ) {
			if ( h.state_h == EEPROM_PAGE_STATE_MOVE_IN_H && h.state_l == EEPROM_PAGE_STATE_MOVE_IN_L  ) {
				if ( ! __eeprom_page_init(page,STATE_READY) ) {
					#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
					 log_error("EEPROM impossible to clear move_in page %d\r\n",page);
					#endif
					return false;
				}
			}
		}
	}

	#ifdef __EEPROM_WITH_TEST
		if ( injectCrash == 101 ) return false;
	#endif

	// The second step is to commit MOVE_OUT page into INIT_EMPTY stage for being available for reusing
	for ( int page = 0; page < EEPROM_TOTAL_PAGES ; page++ ) {
		uint32_t pAddr = EEPROM_PAGE_ADDR(page);
		__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
		if ( h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION ) {
			if ( h.state_h == EEPROM_PAGE_STATE_MOVE_OUT_H && h.state_l == EEPROM_PAGE_STATE_MOVE_OUT_L  ) {
				if ( ! __eeprom_page_init(page,STATE_INIT_EMPTY) ) {
					#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
					 log_error("EEPROM impossible to clear move_out page %d\r\n",page);
					#endif
					return false;
				}
			}
		}
	}
	return true;

}


/**
 * Clear the move_in, non empty pages this means the migration failed so we need to restart it
 * Then the page can be reused. We assume this will work, otherwise we don't have a lot of
 * solutions.
 */
void __eeprom_clear_move_in_pages() {
	for ( int page = 0; page < EEPROM_TOTAL_PAGES ; page++ ) {
		uint32_t pAddr = EEPROM_PAGE_ADDR(page);
		__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
		if ( h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION ) {
			if ( h.state_h == EEPROM_PAGE_STATE_MOVE_IN_H && h.state_l == EEPROM_PAGE_STATE_MOVE_IN_L ) {
				// this page is to be cleared and restored as state init if not empty
				if ( ! __eeprom_page_init(page,STATE_INIT_EMPTY) ) {
					#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
					 log_error("EEPROM impossible to clear move_in page %d\r\n",page);
					#endif
				}
			}
		}
	}
}

/**
 * Clear the moved out pages, this is executed after the garbage collection once a
 * destination page has been completed, we can clean the page in status moved
 * before switching the destination page state to ready
 */
void __eeprom_clear_moved_out_pages() {
	for ( int page = 0; page < EEPROM_TOTAL_PAGES ; page++ ) {
		uint32_t pAddr = EEPROM_PAGE_ADDR(page);
		__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
		if ( h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION ) {
			if ( h.state_h == EEPROM_PAGE_STATE_MOVE_OUT_H && h.state_l == EEPROM_PAGE_STATE_MOVE_OUT_L ) {
				// this page is to be cleared, need to call page init to preserve the aging
				if ( ! __eeprom_page_init(page,STATE_INIT_EMPTY) ) {
					#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
					 log_error("EEPROM impossible to clear moved page %d\r\n",page);
					#endif
				}
			}
		}
	}
}


/**
 * move the data from page source to page dest, we assume the page are initialized
 * if the psrc is not switch to transfer mode, do it
 * return as q_src the number of values removed from source and as q_dst the number of line used in destination
 * (can be higher when write verification fails)
 */
bool __eeprom_move_page(int psrc, int pdst, int * q_src, int * q_dst) {

	*q_src = 0;
	*q_dst = 0;

	//log_info("** move (%d) to (%d)\r\n",psrc,pdst);

	// First step is to make sure the page status is EEPROM_PAGE_STATE_MOVE_OUT
	if ( ! __eeprom_page_init(psrc,STATE_MOVE_OUT) ) {
		#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
		  log_error("EEPROM impossible to switch page to move_out\r\n");
		#endif
		return false;
	}

	#ifdef __EEPROM_WITH_TEST
    	if ( injectCrash == 1 ) return false;
	#endif

	// Then make sure the destination state is EEPROM_PAGE_STATE_MOVE_IN or switch it from INIT only
	if ( ! __eeprom_page_init(pdst,STATE_MOVE_IN) ) {
		#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
		  log_error("EEPROM impossible to switch page to move_in\r\n");
		#endif
		return false;
	}

	#ifdef __EEPROM_WITH_TEST
		if ( injectCrash == 2 ) return false;
		if ( injectCrash == 3 ) injectCrash--; // will crash the next time
	#endif


	// Search first free line in destination page
	uint32_t lAddr_d = EEPROM_FIRST_LINE_ADDR_IN_PAGE(pdst);
	for ( int i = 0 ; i < EEPROM_LINE_PER_PAGE ; i++ ) {
		__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr_d);	// to optimize, here we read the whole line but we just need addr
		if ( l.addr == 0x3FF ) break;
		lAddr_d += sizeof(__eeprom_line_t);
	}

	// Count how many should be transfered
	int toMove = 0;
	uint32_t lAddr_s = EEPROM_FIRST_LINE_ADDR_IN_PAGE(psrc);
	for ( int i = 0 ; i < EEPROM_LINE_PER_PAGE ; i++ ){
		__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr_s);	// to be optimized, here we read the whole line but we just need addr
		if ( l.addr != 0x3FF && l.addr != 0x000 ) {
			toMove++;
		}
		lAddr_s += sizeof(__eeprom_line_t);
	}

	// Nothing to be done, all entries to be trashed
	if ( toMove == 0 ) {
		*q_src = EEPROM_LINE_PER_PAGE;
		return true;
	}

	// Now we can transfer the lines to destination
	lAddr_s = EEPROM_FIRST_LINE_ADDR_IN_PAGE(psrc);
	int moved = 0;
	while ( lAddr_s < EEPROM_PAGE_ADDR(psrc+1) && lAddr_d < EEPROM_PAGE_ADDR(pdst+1) ) {
		__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr_s);	// to optimize, here we read the whole line but we just need addr
		if ( l.addr != 0x3FF && l.addr != 0x000 ) {

			#ifdef __EEPROM_WITH_TEST
				if ( injectCrash == 10 ) return false;
				if ( injectCrash > 10 && injectCrash < 100 ) injectCrash--; // will crash later
			#endif

			if ( ! __eeprom_write_and_verify(&l,lAddr_d) ) {
				// when write error, go to next line... that's a problem
				#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
				  log_error("EEPROM garbage line transfer failed\r\n");
				#endif
			} else {
				lAddr_s += sizeof(__eeprom_line_t);
				moved++;
				*q_src = *q_src+1;
			}
			*q_dst = *q_dst+1;
			lAddr_d += sizeof(__eeprom_line_t);
		} else {
			lAddr_s += sizeof(__eeprom_line_t);
			*q_src = *q_src+1;
		}
	}
	// page is transfered
	return ( moved == toMove );
}

/*
 * Detect initialization to run a first garbage test
 */
static uint8_t __eeprom_state_verified = 0;

/**
 * Garbage collection
 * 1 - terminate the existing move if any ( this is when the mcu reset during garbaging )
 *     if we have page in status move_in we can clear them and restart the process
 *     if we have page in status move_out but no page in move_in status, it means the transfer has been made but these pages haven not been
 *     reset. do it.
 * 2 - get a free page as a destination page
 * 3 - restart the previous garbage execution if failed by moving the move_out page into the destination page
 * 4 - normal processing when looking for pages to move into the destination
 *     search for 1 page where we have a maximum line we can free in one shot and iterate until the target page can't merge a full page
 * 5 - then have a look if we can merge two pages into a single one, this could be interesting and executed as cherry on the cake
 *
 * As a parameter, justCommit will execute garbage collection only if there are pending page_in or page_out, this is used on start to make
 * sure the eeprom state is ok
 *
 * To test the garbage collection, we can inject errors and recall the garbage manually to ensure it restore
 * it correctly
 */
void __eeprom_garbage_collection(bool justCommit) {
	__eeprom_stat_t s;

	//log_info("** Garbage is running\r\n");
	__eeprom_get_stat(&s);

	__eeprom_state_verified = 1;
	if ( justCommit && s.move_in_pages == 0 && s.move_out_pages == 0 ) return;

	// -- 1 --
	if ( s.move_in_pages > 0 ) {
		// We have page in move_in state, meaning the garbage collection process failed
		// this page can be cleared and restart the move (should never be > 1, but what to do ?!?
		#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
		  if ( s.move_in_pages > 1 ) log_error("EEPROM multiple move_in page\r\n");
		#endif
		__eeprom_clear_move_in_pages();
	}

	if ( s.move_out_pages > 0 && s.move_in_pages == 0 ) {
		// in this case, some of the to move_out pages have not been reset but the pages have been already transfered
		// just clear them
		__eeprom_clear_moved_out_pages();
	}

	// -- 2 --
	int freeLine_dst = EEPROM_LINE_PER_PAGE;
	uint8_t p_dst = __eeprom_get_free_page();
	if ( p_dst == 0xFF ) {
		// we should not be in this situation
		#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
		  log_error("EEPROM no page to init for garbaging\r\n");
		#endif
		__eeprom_onFlashErrorCallback(EEPROM_ERR_NO_FREE_PAGE);
		goto garbage_failed;
	} else {

		// -- 3 --
		if ( s.move_out_pages > 0 && s.move_in_pages > 0 ) {
			// We have page with initiated transfer, so we need to replay the transfer and terminate it.
			// loop on state move out pages
			for ( int p_src = 0; p_src < EEPROM_TOTAL_PAGES ; p_src++ ) {
				int q_src;
				int q_dst;
				uint32_t pAddr = EEPROM_PAGE_ADDR(p_src);
				__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
				if (   h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION
					&& h.state_h == EEPROM_PAGE_STATE_MOVE_OUT_H && h.state_l == EEPROM_PAGE_STATE_MOVE_OUT_L
				){
					int toMove_src = (EEPROM_LINE_PER_PAGE - s.trashed_lines_per_page[p_src] - s.free_lines_per_page[p_src]);
					if ( freeLine_dst >= toMove_src ) {
						// else we may have a problem to copy this page as it is higher, try skip it
						if ( ! __eeprom_move_page(p_src,p_dst,&q_src,&q_dst) ) {
							// problem during page copy
							#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
							  log_error("EEPROM error garbage copy failed\r\n");
							#endif
							__eeprom_onFlashErrorCallback(EEPROM_ERR_PAGE_MOVE_FAILED);
							goto garbage_failed;
						} else {
							freeLine_dst -= q_dst;
							s.trashed_lines_per_page[p_src] = EEPROM_LINE_PER_PAGE;
							s.free_lines_per_page[p_src] = 0;
						}
					} else {
						#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
						  log_error("EEPROM stopped repair moveout before end\r\n");
						#endif
						__eeprom_onFlashErrorCallback(EEPROM_ERR_GARBAGE_REPAIR_FAILED);
					}
				}
			}
		}

		// -- 4 --
		// Here is the normal case where we just need to free pages.
		bool _exit = false;
		while ( ! _exit ) {

			// see if we can add a page in the current page
			int p_src = 0xFF;
			uint16_t maxTrashed = 0;
			for ( int i = 0 ; i < EEPROM_TOTAL_PAGES ; i++ ) {
				uint32_t pAddr = EEPROM_PAGE_ADDR(i);
				__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
				if (   h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION
					&& h.state_h == EEPROM_PAGE_STATE_READY_H && h.state_l == EEPROM_PAGE_STATE_READY_L
				){
					int usedPage_src = (EEPROM_LINE_PER_PAGE - s.trashed_lines_per_page[i] - s.free_lines_per_page[i]);
					if ( freeLine_dst >= usedPage_src ) {
						// this page is candidate, search the one with more trashed as possible
						if ( maxTrashed < s.trashed_lines_per_page[i] ){
							maxTrashed = s.trashed_lines_per_page[i] ;
							p_src = i;
						}
					}
				}
			}
			if ( p_src != 0xFF ) {
				// we have a page to move
				int q_src;
				int q_dst;
				if ( ! __eeprom_move_page(p_src,p_dst,&q_src,&q_dst) ) {
					// problem during page copy
					#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
					  log_error("EEPROM error garbage copy failed\r\n");
					#endif
					__eeprom_onFlashErrorCallback(EEPROM_ERR_PAGE_MOVE_FAILED);
					goto garbage_failed;
				} else {
					freeLine_dst -= q_dst;
					s.trashed_lines_per_page[p_src] = EEPROM_LINE_PER_PAGE;
					s.free_lines_per_page[p_src] = 0;
				}
			} else {
				// all movable pages done for this destination page
				// we can commit the current change
				if( freeLine_dst < EEPROM_LINE_PER_PAGE ) {
					// there is something to commit
					// switch MOVE_IN page to READY
					if ( ! __eeprom_transfer_to_terminate() ){
						// problem during page commit
						#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
						  log_error("EEPROM error garbage commit failed\r\n");
						#endif
						__eeprom_onFlashErrorCallback(EEPROM_ERR_GARBAGE_COMMIT);
						goto garbage_failed;
					}

					// -- 5 --
					// do we need to continue ?
					// this make sense if we can group 2 pages or more in a single one
					__eeprom_get_stat(&s);
					freeLine_dst = EEPROM_LINE_PER_PAGE;
					for ( int j = 0 ; j < 2 ; j++ ) {
						int maxTrashed = 0;
						p_src = 0xFF;
						for ( int i = 0 ; i < EEPROM_TOTAL_PAGES ; i++ ) {
							uint32_t pAddr = EEPROM_PAGE_ADDR(i);
							__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);

							if (   h.magic == EEPROM_MAGIC && h.version == EEPROM_VERSION
								&& h.state_h == EEPROM_PAGE_STATE_READY_H && h.state_l == EEPROM_PAGE_STATE_READY_L
							){
								int usedLine_src = EEPROM_LINE_PER_PAGE - s.trashed_lines_per_page[i] - s.free_lines_per_page[i];
								if ( freeLine_dst >= usedLine_src ) {
									// this page is candidate, search the one with more trashed as possible
									if ( maxTrashed < s.trashed_lines_per_page[i] ){
										maxTrashed = s.trashed_lines_per_page[i] ;
										p_src = i;
									}
								}
							}
						}
						if ( p_src != 0xFF ) {
							freeLine_dst -= (EEPROM_LINE_PER_PAGE - s.trashed_lines_per_page[p_src] - s.free_lines_per_page[p_src]);
						} else {
							// nothing much to do, exit
							_exit = true;
							break;
						}
					}
					if ( freeLine_dst > 0 ) {
						// when reaching this point, we have a potential of two pages we can fusion into a single one, let's do it.
						// with a new loop
						freeLine_dst = EEPROM_LINE_PER_PAGE;
						p_dst = __eeprom_get_free_page();
						if ( p_dst == 0xFF ) {
							// we should not be in this situation but as we already fusioned page, it's not blocking
							#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
							  log_error("EEPROM no page to init for garbaging\r\n");
							#endif
							__eeprom_onFlashErrorCallback(EEPROM_WARN_NO_FREE_PAGE);
							goto garbage_warning;
						}
					}
				} else {
					// nothing else to do, we did not had any transfer to that page, it is empty
					// so we need to clear the pages
					if ( ! __eeprom_transfer_to_terminate() ){
					    // problem during page commit
						#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
						  log_error("EEPROM error garbage commit failed\r\n");
						#endif
						__eeprom_onFlashErrorCallback(EEPROM_ERR_GARBAGE_COMMIT);
						goto garbage_failed;
					}
					_exit = true;
				}
			}
		}
	}

	// in case user level needs to make some check
	__eeprom_onGarbageCallback();

	// @TODO
	// have some next step to rebalance the pages when aging variability is high
	// have some next step to group the non changing lines to decrease the average aging.
	//log_info("** Garbage done\r\n");
	return;

	garbage_failed:
	// User callback about EERPOM memory corrupted
	__eeprom_onFlashErrorCallback(EEPROM_ERR_GARBAGE_FAILED);
	return;

	garbage_warning:
	// User callback about EEPROM non critical problem for
	// taking preventive action on the device
	__eeprom_onFlashErrorCallback(EEPROM_WARN_GARBAGE_FAILED);

	return;

}



// ===============================================================================
// PUBLIC API
// ===============================================================================


/**
 * Called once the GarbageCollection has been executed
 * Do nothing but user application can do some specific
 * operations related to this.
 */
#ifndef __EEPROM_WITH_TEST

__weak void __eeprom_onGarbageCallback() {}

#else
	static uint8_t __gc_counter = 0;
	void __eeprom_onGarbageCallback(){
		__gc_counter++;
		#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
		//	_eeprom_info();
		#endif
	}
#endif

/**
 * Called when an error or warning from the eeprom flash driver is
 * reported. This allows the main code to take action or to report it
 * for device management
 */
#ifndef __EEPROM_WITH_TEST

__weak void __eeprom_onFlashErrorCallback( __eeprom_error_t errCode ) { }

#else
	void __eeprom_onFlashErrorCallback( __eeprom_error_t errCode ) {
		if ( errCode == EEPROM_ERR_GARBAGE_FAILED && injectCrash > 0 ) {
			// simulate a restart
			injectCrash = 0;
			__eeprom_garbage_collection(true);
		}
	}

#endif
/**
 * Write in the eeprom the given data.
 * A bank is used as a entry index to manage multiple content.
 * Offset allows to write 1 bank in multiple operation where offset is the pointer for bank start
 *  offset is aligned on 32b words
 * Actually bank is not supported, assuming is 0
 * @TODO : manage bank
 */
bool _eeprom_write(uint8_t bank, uint32_t offset, void * data, int len) {
	if ( __eeprom_state_verified == 0 ) __eeprom_garbage_collection(true);

	uint8_t *  _data = (uint8_t *)data;

	// check address
	if ( bank != 0 || (offset+len) > ITSDK_EPROM_SIZE ) {
		#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
			log_error("EEPROM out of bounds 0x%08X\r\n",offset);
		#endif
		ITSDK_ERROR_REPORT(ITSDK_ERROR_EEPROM_OUTOFBOUNDS,len);
		return false;
	}
	if ( (offset & 0x3) != 0 ) {
		#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
			log_error("EEPROM not aligned 0x%08X\r\n",offset);
		#endif
		ITSDK_ERROR_REPORT(ITSDK_ERROR_EEPROM_NOTALIGNED,1);
		return false;
	}

	uint32_t _offset = offset;
	int _len = len;
	int retry = 0;
	do {
		uint32_t lAddr = __eeprom_find_virtual_line(_offset);
		__eeprom_line_t target;
		target.addr = EEPROM_ADDR_TO_LINE_ADDR(_offset) & 0x3FF;	// aligned address
		int shift = 0;
		bool failure = false;

		//if (1 || _offset > 0x6C4) log_info("** Wr off(%08X) lAddr(%08X) tAddr(%08X)",_offset,lAddr,target.addr);


		// We need a target to store the result, new or existing, whatever, result will be in a new line
		uint32_t tAddr=0;
		int page;
		if ( lAddr < EEPROM_START_ADDR ) {
			// find a line in the proposed page, already scanned during the line search
			// to get an optimization. eventually FF returned to search globally
			page = (uint8_t)lAddr;
			tAddr = __eeprom_find_empty_line(page);
		} else {
			// when the line has been found, search globally a target page & line
			tAddr = __eeprom_find_empty_line(0xFF);
		}
		//if (1 || _offset > 0x6C4) log_info("** tAddr (%08X)\r\n",tAddr);
		if ( tAddr == 0xFF ) {
			// in this situation we need to garbage collect the flash
			__eeprom_garbage_collection(false);
			// refresh the source address if moved
			lAddr = __eeprom_find_virtual_line(_offset);
			// then search a new target line
			tAddr = __eeprom_find_empty_line(0xFF);
			if (tAddr < EEPROM_START_ADDR ) {
				page = (uint8_t)tAddr;
				if ( page == 0xFF ) {
					// problem ... no page available after garbage
					#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
						log_error("Failed get a free page even after garbage\r\n");
					#endif
					goto write_failed;
				}
			}
		}
		// need to init this page, make sure after garbage that we still have a page (what we should have)
		if ( tAddr < EEPROM_START_ADDR ) {
			page = (uint8_t)tAddr;
			if ( !__eeprom_page_init(page,STATE_READY) ) {
				goto write_failed;
			}
			tAddr = EEPROM_FIRST_LINE_ADDR_IN_PAGE(page);
		}
		// Here we have a tAddr corresponding to an address for the target


		if ( lAddr < EEPROM_START_ADDR ) {
			// The memory line does not exists and we need to add it, in a page initialized or not ...
			shift =  _offset - EEPROM_LINE_ADDR_TO_ADDR(target.addr);	 // calculate the address shift from the beginning of the line

			// the entry does not exist, we need to create it.
			// the page is valid this is the page where we got some free space to create the new entry
			// create content
			target.updates = 0; 									// first time we create it
			int idx = 0;
			for (int i = 0 ; i < shift ; i++ ){
				target.data[idx] = 0xFF;							// unknown value, keep 0xFF, no change
				idx++;
			}
			for (int i = 0 ; i < _len && i < _EEPROM_BYTE_PER_LINE-shift ; i++) {
				target.data[idx] = *_data;							// prepare data
				_data++;
				idx++;
			}
			while ( idx < _EEPROM_BYTE_PER_LINE ) {
				target.data[idx] = 0xFF;								// keep the rest unchanged if any
				idx++;
			}
			target.check = __eeprom_compute_check(&target);

			// Write the new line
			failure = ! __eeprom_write_and_verify(&target,tAddr);

		} else {
			// the memory line exist at the given address
			// is that unchanged ? else we need to move it...move it

			__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr);
			target.updates = (l.updates != 3)?l.updates+1:3;
			shift = _offset - EEPROM_LINE_ADDR_TO_ADDR(target.addr);								// calculate the address shift from the beginning of the line

			// create the new line with the previously existing content when required
			int idx = 0;
			for (int i = 0 ; i < shift ; i++ ){
				target.data[idx] = l.data[idx];							// copy the previous data if any
				idx++;
			}
			for (int i = 0 ; i < _len && i < _EEPROM_BYTE_PER_LINE-shift ; i++) {
				target.data[idx] = *_data;								// write new data
				_data++;
				idx++;
			}
			while ( idx < _EEPROM_BYTE_PER_LINE ) {
				target.data[idx] = l.data[idx];							// keep the rest unchanged if any
				idx++;
			}
			target.check = __eeprom_compute_check(&target);

			// compare lines
			bool same = true;
			for ( int i = 0 ; i < _EEPROM_BYTE_PER_LINE ; i++ ) {
				if (target.data[i] != l.data[i]) {
					same = false;
					break;
				}
			}

			if( !same ) {
				// Write the new line
				failure = ! __eeprom_write_and_verify(&target,tAddr);
				// Clear the old line
				__eeprom_clear_line(lAddr);
			}

		}


		// Process next line
		if ( !failure ) {
			// jump to the next line
			_offset += _EEPROM_BYTE_PER_LINE-shift;
			_len -= _EEPROM_BYTE_PER_LINE-shift;
			retry = 0;
		} else if ( retry < 3 ) {
			// retry to write this line into a different block of data
			retry++;
		} else {
			#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
				log_error("Failed to complete EEPROM Write for offset %08X and len %d\r\n",offset,len);
			#endif
			return false;
		}
		// log_info("** _off(%08X) off(%08X) len(%d) shift(%d)\r\n",_offset,offset,len,shift);
	} while ( _offset < offset+len );

	return true;

write_failed:

	return false;

}

/**
 * Read a block of data from the EEPROM
 * Offset is to add an offset to bank start - Offset is aligned don 32b word
 */
bool _eeprom_read(uint8_t bank, uint32_t offset, void * data, int len) {
	if ( __eeprom_state_verified == 0 ) __eeprom_garbage_collection(true);

	uint8_t  * _data = (uint8_t *)data;

	if ( bank != 0 || (offset + len) > EEPROM_SIZE) {
	    ITSDK_ERROR_REPORT(ITSDK_ERROR_EEPROM_OUTOFBOUNDS,len);
	}

	// alignment is not required for virtual eeprom but it's better to
	// keep the verification for compatibility with aligned eeprom
	if ( (offset & 0x3) != 0 ) {
	    ITSDK_ERROR_REPORT(ITSDK_ERROR_EEPROM_NOTALIGNED,0);
	    return false;
	}

	int read = 0;
	uint32_t _offset = offset;
	while ( read < len ) {
		uint32_t lAddr = __eeprom_find_virtual_line(_offset);
		if ( lAddr > EEPROM_START_ADDR ) {
			// this address exist in EEPROM
			__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr);
			int shift = _offset - EEPROM_LINE_ADDR_TO_ADDR(l.addr);
			for ( int i = shift ; read < len && i < _EEPROM_BYTE_PER_LINE ; i++ ) {
				_data[read] = l.data[i];
				read++;
				_offset++;
			}
		} else {
			// this address does not exists
			#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
				log_error("Failed to read EEPROM invalid offset %08X\r\n",_offset);
			#endif
			ITSDK_ERROR_REPORT(ITSDK_ERROR_EEPROM_NOTEXISTING,0);
			return false;
		}
	}
	return true;
}


#ifdef __EEPROM_WITH_TEST

// ======================================================================
// Test Function
// ======================================================================


// structure for test with an occupation of 11 lines
// structure verification with sum field, making a sum of the other field
// fields not aligned on lines
typedef struct {
	uint8_t		u8_1;	// 64b								8B
	uint16_t 	u16_1;
	uint32_t	u32_1;
	uint8_t		u8_2;

	uint8_t		u8_3[6];	// end of line 1			//  6B

	uint32_t	u32_2[10];								// 40B
	uint16_t	u16_2;		// end of line 2,3,4,5		//  2B

	uint16_t 	u16_3;									//  2B
	uint32_t	u32_3[10];	// end of line 6,7,8,9		// 40B

	uint32_t	u32_4[4];	// end of line 10 + 4Bytes	// 16B

	uint32_t	sum;		// in line 11				//  4B
	uint16_t	u16_4;		// 32Bits alignment			//  2B

} __test_struct;										// 120B

// create a content driven by a single value
void ___init_test(__test_struct * s, uint8_t v) {
	s->u8_1 = v;
	s->u16_1 = ((v ^ 0xA5) << 8) | ( v ^ 0x5A );
	s->u32_1 = ((v ^ 0xFF) << 24) | (( v ^ 0xFF ) << 16) | s->u16_1;
	s->u8_2 = v;

	for ( int i = 0 ; i < 6 ; i++ ) s->u8_3[i] = v;
	for ( int i = 0 ; i < 10 ; i++ ) s->u32_2[i] = s->u32_1;
	s->u16_2 = s->u16_1;
	s->u16_3 = s->u16_1;
	for ( int i = 0 ; i < 10 ; i++ ) s->u32_3[i] = s->u32_1;
	for ( int i = 0 ; i < 4 ; i++ ) s->u32_4[i] = s->u32_1;
	s->sum = 8*v + 3*s->u16_1 + 25*s->u32_1;
	s->u16_4 = 0xFFFF;
}

// calculate the structure checksum for verification
uint32_t ___sum_test(__test_struct * s) {
	uint32_t sum = 0;
	sum = s->u8_1 + s->u16_1 + s->u32_1 + s->u8_2 + s->u16_2 + s->u16_3;
	for ( int i = 0 ; i < 6 ; i++ ) sum += s->u8_3[i];
	for ( int i = 0 ; i < 10 ; i++ ) sum += s->u32_2[i] + s->u32_3[i];
	for ( int i = 0 ; i < 4 ; i++ ) sum += s->u32_4[i];
	return sum;
}

// alter the structure, only change 4 lines including the sum line (1,5,6,11)
void ___alter_struct(__test_struct * s, uint8_t v) {
	s->u16_1 = ((v ^ 0xA5) << 8) | ( v ^ 0x5A );
	s->u16_2 = s->u16_1;
	s->u16_3 = s->u16_1;
	s->sum = ___sum_test(s);
}




/**
 * For a verification of the eeprom implementation, run tis function from
 * the main, this should just be a one time run when this file has been modified
 * never run this on prod !
 * run debug test with
 * #define ITSDK_WITH_ERROR_RPT	  	__DISABLE
 * #define ITSDK_ERROR_USE_EPROM  	__DISABLE
 * #define ITSDK_WITH_SECURESTORE 	__DISABLE
 * #define ITSDK_WITH_CONSOLE	  	__DISABLE
 * #define ITSDK_CONFIGURATION_MODE	__CONFIG_MEMORY
 */
void _eeprom_test() {

	// TEST-1
	// Clear Flash area
	log_info("EE-TEST-1 - eeprom start addr %08X\r\n",EEPROM_START_ADDR);

	// clear all the pages to get started
	for ( int p = 0 ; p < EEPROM_TOTAL_PAGES ; p++ ){
		if ( !__eeprom_page_clear(p) ) {
			log_error("EE-TEST - Failed to clear page %d\r\n",p);
			goto failed;
		}
	}
	log_info("EE-TEST-1 - pages cleared\r\n");

	// --- TEST2
	// write 1 char with value 0xA5 at address 4, this may create a first page and a first line with values
	// FF FF FF FF A5 FF FF FF FF FF FF FF FF FF
	// in memory (R for reserved)
	// Page : 11 00000000000000 916E  FF FF FF 01   FFFF A55A   FFFFFFFF
	//        R  Aging          Magic R	 R  R  Ver  R    Ready  R
	//
	// Line : FF FF 1101 00 00.0000.0001 | FF A5 FF FF | FF FF FF FF | FF FF FF FF
	//              D    0     0    1
	//        D1 D0 Chk  U  Address      | D5 D4 D3 D2 | D9 D8 D7 D6 | DD DC DB DA

	uint8_t data[1] = { 0xA5 };
	uint32_t addr = 0x04;
	if( ! _eeprom_write(0, addr, data, sizeof(data) ) ) {
		log_error("EE-TEST-2 - Failed to write a single byte at %d\r\n",addr);
		goto failed;
	}

	// verify page creation and headers
	// should get at eeprom start addr a first page with 0 aging
	uint32_t pAddr = EEPROM_START_ADDR;
	__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
	if (   h.magic != EEPROM_MAGIC || h.version != EEPROM_VERSION
		|| h.state_h != EEPROM_PAGE_STATE_READY_H || h.state_l != EEPROM_PAGE_STATE_READY_L
		|| h.aging != 0
	){
		log_error("EE-TEST-2 - Failed to create the first page header\r\n");
		goto failed;
	}
	log_info("EE-TEST-2 - first page creation success\r\n");

	// verify the memory content, should be at address EEPROM_START_ADDR + sizeof(__eeprom_page_header_t)
	__eeprom_line_t l = (*(volatile __eeprom_line_t*)(EEPROM_START_ADDR + sizeof(__eeprom_page_header_t)));
	if ( EEPROM_LINE_ADDR_TO_ADDR(l.addr) != 0x0 || l.addr != 0x01|| l.updates != 0 ) {
		log_error("EE-TEST-2 - Failed validate first line header content\r\n");
		goto failed;
	}
	if ( __eeprom_compute_check(&l) != l.check ) {
		log_error("EE-TEST-2 - Failed to verify the line 1 checksum\r\n");
		goto failed;
	}
	if ( l.data[addr] != data[0] ) {
		log_error("EE-TEST-2 - Failed to find value 0xA5 at offset %d, got value 0x%02X \r\n",addr,l.data[addr]);
		goto failed;
	}
	log_info("EE-TEST-2 - 0x%d write success, verified\r\n",addr);

	// ---- TEST 3
	// create a full line, aligned address @ 4*_EEPROM_BYTE_PER_LINE ( for alignment )
	// Line : 01 00 1011 00 00.0000.0101 | 05 04 03 02 | 09 08 07 06 | 0D 0C 0B 0A
	//              B    0     0    5
	//        D1 D0 Chk  U  Address      | D5 D4 D3 D2 | D9 D8 D7 D6 | DD DC DB DA

	uint8_t data2[_EEPROM_BYTE_PER_LINE];
	for ( int i = 0 ; i < _EEPROM_BYTE_PER_LINE ; i++ ) {
		data2[i] = i;
	}
	addr = 4*_EEPROM_BYTE_PER_LINE;
	if ( ! _eeprom_write(0,addr, data2, sizeof(data2)) ) {
		log_error("EE-TEST-3 - Failed to write an aligned line %d\r\n",addr);
		goto failed;
	}
	// should be at the second line in eeprom
	l = (*(volatile __eeprom_line_t*)(EEPROM_START_ADDR + sizeof(__eeprom_page_header_t) + sizeof(__eeprom_line_t) ));
	if (   EEPROM_LINE_ADDR_TO_ADDR(l.addr) != addr
		|| l.addr != 0x05
	    || l.updates != 0x00
		|| __eeprom_compute_check(&l) != l.check
	) {
		log_error("EE-TEST-3 - Failed validate full aligned line\r\n");
		goto failed;
	}
	log_info("EE-TEST-3 - 0x%d aligned write success, verified\r\n",addr);

	// --- TEST 4
	// read the aligned value
	uint8_t rdata2[_EEPROM_BYTE_PER_LINE];
	if ( !_eeprom_read(0,addr,rdata2,sizeof(rdata2)) ) {
		log_error("EE-TEST-4 - read return an error\r\n");
		goto failed;
	}
	for ( int i = 0 ; i < sizeof(rdata2) ; i++ ) {
		if ( data2[i] != rdata2[i] ) {
			log_error("EE-TEST-4 - at %d read %x expected %x\r\n",i,rdata2[i],data2[i]);
			goto failed;
		}
	}
	log_info("EE-TEST-4 - 0x%d aligned read success\r\n",addr);


	// --- TEST 5
	// create a full line, not aligned address @ 2*_EEPROM_BYTE_PER_LINE+4 (aligned)
	// Line : FF FF 1010 00 00.0000.0011 | 81 80 FF FF | 85 84 83 82 | 89 88 87 86
	//              A    0     0    3
	//        D1 D0 Chk  U  Address      | D5 D4 D3 D2 | D9 D8 D7 D6 | DD DC DB DA
	for ( int i = 0 ; i < _EEPROM_BYTE_PER_LINE ; i++ ) {
		data2[i] = 0x80+i;
	}
	addr = 2*_EEPROM_BYTE_PER_LINE+4;
	if ( ! _eeprom_write(0,addr, data2, sizeof(data2)) ) {
		log_error("EE-TEST-5 - Failed to write an not aligned line %d\r\n",addr);
		goto failed;
	}
	// this should create two lines
	l = (*(volatile __eeprom_line_t*)(EEPROM_START_ADDR + sizeof(__eeprom_page_header_t) + 2*sizeof(__eeprom_line_t) ));
	if (   EEPROM_LINE_ADDR_TO_ADDR(l.addr) != (addr-4)
		|| l.addr != 0x003
	    || l.updates != 0x00
		|| __eeprom_compute_check(&l) != l.check
	) {
		log_error("EE-TEST-5 - Failed validate 1st line of not aligned\r\n");
		log_error("            check %02X vs %02X\r\n",__eeprom_compute_check(&l),l.check);
		goto failed;
	}
	for ( int i = 0 ; i < 4 ; i++ ) {
		if ( l.data[i] != 0xFF ) {
			log_error("EE-TEST-5 - Invalid value, expected 0xFF\r\n");
			goto failed;

		}
	}
	for ( int i = 4 ; i < _EEPROM_BYTE_PER_LINE ; i++ ) {
		if ( l.data[i] != data2[i-4] ) {
			log_error("EE-TEST-5 - Invalid value, expected %x get %x \r\n",data2[i-4],l.data[i]);
			goto failed;

		}
	}
	l = (*(volatile __eeprom_line_t*)(EEPROM_START_ADDR + sizeof(__eeprom_page_header_t) + 3*sizeof(__eeprom_line_t) ));
	if (   EEPROM_LINE_ADDR_TO_ADDR(l.addr) != (addr-4)+_EEPROM_BYTE_PER_LINE
		|| l.addr != 0x004
	    || l.updates != 0x00
		|| __eeprom_compute_check(&l) != l.check
	) {
		log_error("EE-TEST-5 - Failed validate 2nd line of not aligned\r\n");
		goto failed;
	}
	for ( int i = 0 ; i < 4 ; i++ ) {
		if ( l.data[i] != data2[i+_EEPROM_BYTE_PER_LINE-4] ) {
			log_error("EE-TEST-5 - Invalid value l2, expected %x get %x \r\n",data2[i+_EEPROM_BYTE_PER_LINE-4],l.data[i]);
			log_error("            check %02X vs %02X\r\n",__eeprom_compute_check(&l),l.check);
			goto failed;
		}
	}
	for ( int i = 4 ; i < _EEPROM_BYTE_PER_LINE ; i++ ) {
		if ( l.data[i] != 0xFF ) {
			log_error("EE-TEST-5 - Invalid value l2, expected 0xFF\r\n");
			goto failed;

		}
	}
	log_info("EE-TEST-5 - 0x%d write success, verified\r\n",addr);

	// --- TEST 6
	// read non aligned value
	if ( !_eeprom_read(0,addr,rdata2,sizeof(rdata2)) ) {
		log_error("EE-TEST-6 - read return an error\r\n");
		goto failed;
	}
	for ( int i = 0 ; i < sizeof(rdata2) ; i++ ) {
		if ( data2[i] != rdata2[i] ) {
			log_error("EE-TEST-6 - at %d read %x expected %x\r\n",i,rdata2[i],data2[i]);
			goto failed;
		}
	}
	log_info("EE-TEST-6 - 0x%d non aligned read success\r\n",addr);


	// --- TEST 7
	// add a new value in the first line, not modifying the first one
	data[0] = 0xA6;
	addr = 0x08;
	if( ! _eeprom_write(0, addr, data, sizeof(data) ) ) {
		log_error("EE-TEST-7 - Failed to write a single byte at %d\r\n",addr);
		goto failed;
	}
	l = (*(volatile __eeprom_line_t*)(EEPROM_START_ADDR + sizeof(__eeprom_page_header_t)));
	if ( l.addr != 0 ) {
		log_error("EE-TEST-7 - Line 0 should be cleared with address equal to 0\r\n");
		goto failed;
	}
	l = (*(volatile __eeprom_line_t*)(EEPROM_START_ADDR + sizeof(__eeprom_page_header_t)+ 4*sizeof(__eeprom_line_t)));
	if (   EEPROM_LINE_ADDR_TO_ADDR(l.addr) != 0
		|| l.addr != 0x001
	    || l.updates != 0x01
		|| __eeprom_compute_check(&l) != l.check
	) {
		log_error("EE-TEST-7 - Failed validate updated line\r\n");
		goto failed;
	}
	if ( l.data[4] != 0xA5 || l.data[8] != 0xA6 ) {
		log_error("EE-TEST-7 - Failed to verify data\r\n");
		goto failed;
	}
	log_info("EE-TEST-7 - line modification success\r\n",addr);

	// --- TEST 8
	// change the first value in the first line
	data[0] = 0x5A;
	addr = 0x04;
	if( ! _eeprom_write(0, addr, data, sizeof(data) ) ) {
		log_error("EE-TEST-8 - Failed to update a single byte at %d\r\n",addr);
		goto failed;
	}
	l = (*(volatile __eeprom_line_t*)(EEPROM_START_ADDR + sizeof(__eeprom_page_header_t)+ 4*sizeof(__eeprom_line_t)));
	if ( l.addr != 0 ) {
		log_error("EE-TEST-8 - Line 4 should be cleared with address equal to 0\r\n");
		goto failed;
	}
	l = (*(volatile __eeprom_line_t*)(EEPROM_START_ADDR + sizeof(__eeprom_page_header_t)+ 5*sizeof(__eeprom_line_t)));
	if (   EEPROM_LINE_ADDR_TO_ADDR(l.addr) != 0
		|| l.addr != 0x001
	    || l.updates != 0x02
		|| __eeprom_compute_check(&l) != l.check
	) {
		log_error("EE-TEST-8 - Failed validate updated line\r\n");
		goto failed;
	}
	if ( l.data[4] != 0x5A || l.data[8] != 0xA6 ) {
		log_error("EE-TEST-8 - Failed to verify data\r\n");
		goto failed;
	}
	log_info("EE-TEST-8 - value modification success\r\n",addr);



	// --- TEST 9
	// Fill most of the first page to have next struct be between 1st & 2nd page
	for ( int i = 0 ; i < EEPROM_LINE_PER_PAGE - 8 ; i++ ) {
		addr = ((5+i) * _EEPROM_BYTE_PER_LINE + 8) & 0xFFFFFFFC; // make sure we have alignment
		data[0] = 0x5A;
		wdg_refresh();
		if( ! _eeprom_write(0, addr, data, sizeof(data) ) ) {
			log_error("EE-TEST-9 - Failed to fill the first page at %d\r\n",addr);
			goto failed;
		}
	}
	log_info("EE-TEST-9 - filling the first page success\r\n");

	// Create a larger structure that will move on a second page
	// more than 10 lines need to be used
	__test_struct ts;
	___init_test(&ts,0xC9);
	addr = ((5+EEPROM_LINE_PER_PAGE-8) *_EEPROM_BYTE_PER_LINE + 12) & 0xFFFFFFFC;
	if( ! _eeprom_write(0, addr, &ts, sizeof(ts) ) ) {
		log_error("EE-TEST-9 - Failed to write test structure at %d\r\n",addr);
		goto failed;
	}

	// verify page creation and headers
	// should get at eeprom start addr a first page with 0 aging
	pAddr = EEPROM_START_ADDR+EEPROM_PAGE_SIZE;
	h = (*(volatile __eeprom_page_header_t*)pAddr);
	if (   h.magic != EEPROM_MAGIC || h.version != EEPROM_VERSION
		|| h.state_h != EEPROM_PAGE_STATE_READY_H || h.state_l != EEPROM_PAGE_STATE_READY_L
		|| h.aging != 0 ) {
		log_error("EE-TEST-9 - Failed to create the second page header\r\n");
		goto failed;
	}
	log_info("EE-TEST-9 - second page creation success\r\n");

	// read the structure back and verify values
	__test_struct tsr;
	if ( !_eeprom_read(0,addr,&tsr,sizeof(tsr)) ) {
		log_error("EE-TEST-9 - read return an error\r\n");
		goto failed;
	}
	if ( tsr.sum != ts.sum || tsr.sum != ___sum_test(&tsr) ) {
		log_error("EE-TEST-9 - failed to verify struct integrity\r\n");
		goto failed;
	}

	log_info("EE-TEST-9 - multiple page structure write & read success\r\n",addr);

	// --- TEST 10
	// Modify the structure but only some lines may be modified and not all as most are unchanged
	___alter_struct(&ts,0x91);
	addr = ((5+EEPROM_LINE_PER_PAGE-8) *_EEPROM_BYTE_PER_LINE + 12) & 0xFFFFFFFC;
	if( ! _eeprom_write(0, addr, &ts, sizeof(ts) ) ) {
		log_error("EE-TEST-10 - Failed to update test structure at %d\r\n",addr);
		goto failed;
	}
	// read the structure back and verify values
	if ( !_eeprom_read(0,addr,&tsr,sizeof(tsr)) ) {
		log_error("EE-TEST-10 - read return an error\r\n");
		goto failed;
	}
	if ( tsr.sum != ts.sum || tsr.sum != ___sum_test(&tsr) ) {
		log_error("EE-TEST-10 - failed to verify struct integrity\r\n");
		goto failed;
	}
	log_info("EE-TEST-10 - multiple page structure update success\r\n",addr);


	// --- TEST 11
	// make sure the stats are corresponding, confirming test 10 only modified the targeted line on the global structure
	_eeprom_info();
	__eeprom_stat_t stat;
	__eeprom_get_stat(&stat);
	if ( stat.ready_pages != 2 || stat.trashed_lines != 5 || stat.allocated_lines != 132 ) {
		log_error("EE-TEST-11 - eeprom stats are not coherent\r\n");
		goto failed;
	}
	log_info("EE-TEST-11 - number of trashed lines & allocate page is coherent\r\n");


	// --- TEST 12
	// Create a large set of structure, make then updating to generate garbage collection and verify each of them
	__test_struct multTest[10]; // about 1+ full page of data
	__test_struct multTestR[10]; // to read

	uint32_t baseAddr = ( addr + sizeof(__test_struct) + 14 ) & 0xFFFFFFFC;
	for ( int i = 0 ; i < 10 ; i++ ) {
		wdg_refresh();
		___init_test(&multTest[i],i*16+i);
		if( ! _eeprom_write(0, baseAddr + i*sizeof(__test_struct), &multTest[i], sizeof(__test_struct) ) ) {
			log_error("EE-TEST-12 - Failed to write test struct %d at %d\r\n",i,addr);
			goto failed;
		}
		if ( !_eeprom_read(0,baseAddr + i*sizeof(__test_struct), &multTestR[i],sizeof(__test_struct)) ) {
			log_error("EE-TEST-12 - read return an error %d\r\n",i);
			goto failed;
		}
		if ( multTestR[i].sum != multTest[i].sum || multTest[i].sum != ___sum_test(&multTestR[i]) ) {
			log_error("EE-TEST-12 - failed to verify struct integrity %d\r\n",i);
			goto failed;
		}
		log_info("EE-TEST-12 - created and verified structure at address %d sz %d\r\n",baseAddr + i*sizeof(__test_struct), sizeof(__test_struct));
	}


	bool tested = false;
	for ( int k = 0 ; k < 20 ; k ++ ) {
		// update the structure content and verify it
		for ( int i = 0 ; i < 10 ; i++ ) {
			wdg_refresh();
			___alter_struct(&multTest[i],i*16+i+k);
			if( ! _eeprom_write(0, baseAddr + i*sizeof(__test_struct), &multTest[i], sizeof(__test_struct) ) ) {
				log_error("EE-TEST-12 - Failed to update test struct %d at %d\r\n",i,addr);
				goto failed;
			}
			// wait for garbage collection
			if ( __gc_counter == 1 && ! tested ) {
				tested = true;
				log_info("EE-TEST-12 - garbage collection detected\r\n");
				// At this point we expect Page 1, 3 and 4 moved to 4, status should be init_empty
				uint32_t pAddr = EEPROM_PAGE_ADDR(1);
				__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
				if (    h.magic != EEPROM_MAGIC || h.version != EEPROM_VERSION
					 || h.state_h != EEPROM_PAGE_STATE_INIT_EMPTY_H ||  h.state_l != EEPROM_PAGE_STATE_INIT_EMPTY_L
					 || h.aging != 1
				){
					log_error("EE-TEST-12 - page 1 should be INIT_EMPTY with age 1\r\n");
					goto failed;
				}
				pAddr = EEPROM_PAGE_ADDR(0);
				h = (*(volatile __eeprom_page_header_t*)pAddr);
				if (    h.magic != EEPROM_MAGIC || h.version != EEPROM_VERSION
					 || h.state_h != EEPROM_PAGE_STATE_READY_H ||  h.state_l != EEPROM_PAGE_STATE_READY_L
					 || h.aging != 0
				){
					log_error("EE-TEST-12 - page 0 should be READY with age 0\r\n");
					goto failed;
				}
				pAddr = EEPROM_PAGE_ADDR(2);
				h = (*(volatile __eeprom_page_header_t*)pAddr);
				if (    h.magic != EEPROM_MAGIC || h.version != EEPROM_VERSION
					 || h.state_h != EEPROM_PAGE_STATE_INIT_EMPTY_H ||  h.state_l != EEPROM_PAGE_STATE_INIT_EMPTY_L
					 || h.aging != 1
				){
					log_error("EE-TEST-12 - page 2 should be INIT_EMPTY with age 1\r\n");
					goto failed;
				}
				pAddr = EEPROM_PAGE_ADDR(3);
				h = (*(volatile __eeprom_page_header_t*)pAddr);
				if (    h.magic != EEPROM_MAGIC || h.version != EEPROM_VERSION
					 || h.state_h != EEPROM_PAGE_STATE_INIT_EMPTY_H ||  h.state_l != EEPROM_PAGE_STATE_INIT_EMPTY_L
					 || h.aging != 1
				){
					log_error("EE-TEST-12 - page 3 should be INIT_EMPTY with age 1\r\n");
					goto failed;
				}
				pAddr = EEPROM_PAGE_ADDR(4);
				h = (*(volatile __eeprom_page_header_t*)pAddr);
				if (    h.magic != EEPROM_MAGIC || h.version != EEPROM_VERSION
					 || h.state_h != EEPROM_PAGE_STATE_READY_H ||  h.state_l != EEPROM_PAGE_STATE_READY_L
					 || h.aging != 0
				){
					log_error("EE-TEST-12 - page 4 should be READY with age 0\r\n");
					goto failed;
				}

			}

			if ( !_eeprom_read(0,baseAddr + i*sizeof(__test_struct),&multTestR[i],sizeof(__test_struct)) ) {
				log_error("EE-TEST-12 - read return an error %d\r\n",i);
				goto failed;
			}
			if ( multTestR[i].sum != multTest[i].sum || multTestR[i].sum != ___sum_test(&multTest[i]) ) {
				log_error("EE-TEST-12 - failed to verify struct integrity %d\r\n",i);
				goto failed;
			}
		}
		log_info("EE-TEST-12 - multiple modification and garbaging success loop %d\r\n",k);
	}
	log_info("EE-TEST-12 - multiple modification and garbaging all success\r\n");

	// --- TEST 13
	// Test Garbage collection crash & restore based on the previous structures

	injectCrash = 0;
	int test = 0;
	for ( int k = 0 ; k < 200 ; k ++ ) {

		// wait until crash is executed
		if ( injectCrash == 0 ) {
			test++;
			log_info("EE-TEST-13 - Executing test 13.%d\r\n",test);
			switch(test) {
			case 1:	injectCrash=1; break;			// crash right after move out switch
			case 2: injectCrash=2; break;			// crash after first page switched in move_in
			case 3: injectCrash=3; break;			// crash after second page switched in move_in
			case 4: injectCrash=10; break;			// crash after 1st line transfered
			case 5: injectCrash=25; break;			// crash after 15th line transfered
			case 6: injectCrash=47; break;			// crash after 37th line transfered
			case 7: injectCrash=78; break;			// crash after 68th line transfered
			case 8: injectCrash=95; break;			// crash after 85th line transfered
			case 9: injectCrash=100; break;			// crash before transfer terminate
			case 10: injectCrash=101; break;		// crash in middle of transfer terminate
			default:
			case 11: injectCrash=254; break;	// nothing
			}
		}

		// update the structure content and verify it
		for ( int i = 0 ; i < 10 ; i++ ) {
			wdg_refresh();
			___alter_struct(&multTest[i],i*16+i+k);
			if( ! _eeprom_write(0, baseAddr + i*sizeof(__test_struct), &multTest[i], sizeof(__test_struct) ) ) {
				log_error("EE-TEST-13 - Failed to update test struct %d at %d\r\n",i,addr);
				goto failed;
			}
			if ( !_eeprom_read(0,baseAddr + i*sizeof(__test_struct),&multTestR[i],sizeof(__test_struct)) ) {
				log_error("EE-TEST-13 - read return an error %d\r\n",i);
				goto failed;
			}
			if ( multTestR[i].sum != multTest[i].sum || multTestR[i].sum != ___sum_test(&multTest[i]) ) {
				log_error("EE-TEST-13 - failed to verify struct integrity %d\r\n",i);
				goto failed;
			}
		}
	}
	log_info("EE-TEST-13 - multiple modif & garb with garbaging crashing all success\r\n");


	log_info("EE-TEST - End success\r\n");
	return;

	failed:
	 log_error("EE-TEST - End with failure\r\n");
	 return;

}
#endif // __EEPROM_WITH_TEST

#endif
