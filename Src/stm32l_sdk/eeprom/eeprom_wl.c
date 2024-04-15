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


#if ( EEPROM_SIZE_WITH_OVERHEAD % EEPROM_PAGE_SIZE ) == 0
   #define EEPROM_TOTAL_PAGES  (( EEPROM_SIZE_WITH_OVERHEAD / EEPROM_PAGE_SIZE ))
#else
   #define EEPROM_TOTAL_PAGES  (( EEPROM_SIZE_WITH_OVERHEAD / EEPROM_PAGE_SIZE ) + 1 )
#endif

#define EEPROM_START_ADDR  ( EEPROM_END_ADDR -  (EEPROM_TOTAL_PAGES * EEPROM_PAGE_SIZE) )
#define EEPROM_START_PAGE  ( EEPROM_START_ADDR / EEPROM_PAGE_SIZE )
#define EEPROM_SIZE ITSDK_EPROM_SIZE

// Address field is only even addresses so 2^0 bit is removed ; Adress 0 is prohibited so expected offset is incremented
// the adresses are for bloc of 14 Bytes so we align address on the first address of a such block
#define EEPROM_ADDR_TO_LINE_ADDR(__addr__) (((_EEPROM_BYTE_PER_LINE * (__addr__ / _EEPROM_BYTE_PER_LINE)) >> 1) + 1)

// Reversed operation
#define EEPROM_LINE_ADDR_TO_ADDR(__addr__) ((__addr__ - 1) << 1)

// Magic to identify the EEPROM page initialization
#define EEPROM_MAGIC	0x916E

// Current EEPROM Management version
#define EEPROM_VERSION 	0x01




/**
 * internal Structure
 */

#define _EEPROM_BYTE_PER_LINE	14

typedef struct {
							// 16 control word
	uint16_t addr:10;		// 10 bits internal bloc addresse * 14 + 14
	uint16_t updates:2;		// 2 bits for number of time the line has been overided (max 3 - for identifing 0)
	uint16_t check:4;		// 4 bits "checksum" - count 1 bits


	uint8_t  data[_EEPROM_BYTE_PER_LINE];		// 14 bytes data
} __eeprom_line_t;			// Total Size => 2x64b

typedef struct {
							// 64 bit header structure
	uint16_t magic;			// identify a EEPROM page

	uint16_t aging:14;		// number of time this page has been cleared
	uint16_t reserved_1:2;	// for a later usage

	uint8_t version;		// EEPROM version for later migration
	uint8_t reserved_2[3];	// for a later usage

	uint16_t state;			// FFFF  - initialization in progress / pending activation
							// A55A  - ready for being used
							// 0000  - to be transfered
	uint16_t reserved_3;	// for later usage
	uint32_t reserved_4;	// for later usage

} __eeprom_page_header_t;	// Total Size => 2x64b


typedef struct {
	uint8_t 	pages;				// max number of pages
	uint8_t 	allocated_pages;	// already init pages
	uint8_t		unactivated_pages;	// page not activated
	uint8_t		tobemove_pages;		// pages with transfer pending
	uint32_t 	avg_aging;			// average aging for init pages
	uint16_t 	allocated_lines;	// total allocated (in use) lines
	uint16_t 	free_lines;			// total line never allocated
	uint16_t	trashed_lines;		// total lines cleared after being overiden somewhere else
	uint8_t		untouched_lines;	// total lines never overiden
	uint8_t		trashed_and_free_lines_per_page[EEPROM_TOTAL_PAGES];
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
		uint32_t pAddr = EEPROM_START_ADDR + page * EEPROM_PAGE_SIZE;
		__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
		if ( h. magic == EEPROM_MAGIC && h.version == EEPROM_VERSION ) {
			if ( h.state == 0xA55A) {
				s->allocated_pages++;
				aging+=h.aging;
			} else if ( h.state == 0xFFFF ) {
				s->unactivated_pages++;
			} else if ( h.state == 0x0000 ) {
				s->tobemove_pages++;
				aging+=h.aging;
			}
			if ( h.state == 0xA55A || h.state == 0x0000 ) {
				// this is active data
				uint32_t lAddr = EEPROM_START_ADDR + page * EEPROM_PAGE_SIZE + sizeof(__eeprom_page_header_t);
				for ( int i = 0 ; i < EEPROM_LINE_PER_PAGE ; i++ ) {
					__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr);
					if( l.addr == 0x3FF ) {
						// free line
						s->free_lines++;
						s->trashed_and_free_lines_per_page[page]++;
					} else if (l.addr == 0x000 ) {
						// trashed line
						s->trashed_lines++;
						s->trashed_and_free_lines_per_page[page]++;
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
			s->free_lines += EEPROM_LINE_PER_PAGE;
			s->trashed_and_free_lines_per_page[page] += EEPROM_LINE_PER_PAGE;
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
	log_info(" - Pages Tot(%d) All(%d) Free(%d)\r\n",s.pages,s.allocated_pages,s.pages-s.allocated_pages);
	log_info(" - Lines Tot(%d) All(%d) Free(%d) Trash(%d)\r\n",
			EEPROM_TOTAL_PAGES*EEPROM_LINE_PER_PAGE,
			s.allocated_lines,s.free_lines, s.trashed_lines
	);
}


/**
 * Clear a page, use the internal page numbering
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
 * Init a page, if the page has been initialized previously clear it and recreate it
 */
bool __eeprom_page_init(int page) {
	if ( page > EEPROM_TOTAL_PAGES ) return false;
	uint32_t pAddr = EEPROM_START_ADDR + page * EEPROM_PAGE_SIZE;
	__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
	__eeprom_page_header_t nh;
	nh.magic = EEPROM_MAGIC;
	nh.version = EEPROM_VERSION;
	nh.state = 0xA55A;
	nh.aging = 0;
	nh.reserved_1 = 3;
	nh.reserved_2[0] = 0xFF;nh.reserved_2[1] = 0xFF;nh.reserved_2[2] = 0xFF;
	nh.reserved_3 = 0xFFFF;
	nh.reserved_4 = 0xFFFFFFFF;

	if ( h. magic == EEPROM_MAGIC ) {
		// this page has been initialized previously, the header will be reused
		nh.aging = (h.aging == 0x3FFF)?0x3FFF:h.aging+1;
		if ( nh.aging == EEPROM_AGING_REPORT ) {
			#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
				log_warn("Page %d reach the aging limit\r\n",page);
			#endif
			// call a user callback function to inform the user level of this problem
			// @TODO
		}
	}

	// clear the page
	if ( ! __eeprom_page_clear(page) ) return false;

	// write header per 64 bits blocks
	uint64_t * _target = (uint64_t *)&nh;
	uint32_t _pAddr = pAddr;
	for ( int i = 0 ; i < sizeof(__eeprom_page_header_t) ; i+= sizeof(uint64_t) ) {
		if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _pAddr, *_target) != HAL_OK ) {
			// write failed, try again
			if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _pAddr, *_target) != HAL_OK ) {
				// error will be reported on read verification
				break;
			}
		}
		_target++;
		_pAddr+=sizeof(uint64_t);
	}

	// verify the header
	__eeprom_page_header_t result = (*(volatile __eeprom_page_header_t*)pAddr);
	uint32_t * _t = (uint32_t*)&nh;
	uint32_t * _r = (uint32_t*)&result;
	bool failure = false;
	for ( int i = 0 ; i < sizeof(__eeprom_page_header_t) ; i+= sizeof(uint32_t) ) {
		if ( *_t != *_r ) {
			// failure
			#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
				log_error("Page init failure for page %d\r\n",page);
			#endif
			failure = true;
			break;
		}
		_t++;
		_r++;
	}

	return  !failure;
}


/**
 * return true when a page is initialized, page is a 2K page
 * 0 to 256 as a maximum
 */
bool __eemprom_is_page_ready( uint8_t page ) {
	if ( page >= EEPROM_TOTAL_PAGES ) return false; // out of range

	uint32_t pAddr = EEPROM_START_ADDR + page * EEPROM_PAGE_SIZE;

	__eeprom_page_header_t h = (*(volatile __eeprom_page_header_t*)pAddr);
	if ( h. magic == EEPROM_MAGIC && h.version == EEPROM_VERSION && h.state == 0xA55A) {
		return true;
	}
	return false;
}


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
		if ( __eemprom_is_page_ready(page) ) {
			// search in page
			uint32_t lAddr = EEPROM_START_ADDR + page * EEPROM_PAGE_SIZE + sizeof(__eeprom_page_header_t);
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
		if ( __eemprom_is_page_ready(page) ) {
			// search in page
			uint32_t lAddr = EEPROM_START_ADDR + page * EEPROM_PAGE_SIZE + sizeof(__eeprom_page_header_t);
			for ( int i = 0 ; i < EEPROM_LINE_PER_PAGE ; i++ ) {
				__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr);	// to optimize, here we read the whole line but we just need addr
				if ( l.addr == 0x3FF ) return lAddr;
				lAddr += sizeof(__eeprom_line_t);
			}
		}
		page++;
	}

	// if not found, let see if we have a free page available
	int freePages = 0;
	int firstFreePage = -1;
	for ( int page = 0; page < EEPROM_TOTAL_PAGES ; page ++ ) {
		if ( ! __eemprom_is_page_ready(page) ) {
			if ( firstFreePage == -1 ) firstFreePage = page;
			freePages++;
		}
	}
	if ( freePages == 1 ) return 0xFF; // compaction required
	return firstFreePage;

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
		if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _lAddr, 0) != HAL_OK ) {
			// write failed, try again
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _lAddr, 0);
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
		if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _lAddr, *_target) != HAL_OK ) {
			// write failed, try again
			if ( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _lAddr, *_target) != HAL_OK ) {
				// error will be reported on read verification
				break;
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
				log_warn("Write Failure at address %08X for vAddress %08X\r\n",lAddr,target->addr);
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
 * Write in the eeprom the given data.
 * A bank is used as a entry index to manage multiple content.
 * Offset allows to write 1 bank in multiple operation where offset is the pointer for bank start
 *  offset is aligned on 32b words
 * Actually bank is not supported, assuming is 0
 * @TODO : manage bank
 */
bool _eeprom_write(uint8_t bank, uint32_t offset, void * data, int len) {
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

		if ( lAddr < EEPROM_START_ADDR ) {
			// The memory line does not exists and we need to add it, in a page initialized or not ...

			uint8_t page = (uint8_t)lAddr;
			shift =  _offset - EEPROM_LINE_ADDR_TO_ADDR(target.addr);	 // calculate the address shift from the beginning of the line

			// page valid page left, we need to compact the memory first
			if ( page == 0xFF ) {


			}
			// the page is not valid, we need to create a new one
			if ( ! __eemprom_is_page_ready(page) ) {
				__eeprom_page_init(page);
				// @TODO - what if failed - should panic
			}
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

			// we need to find the line in the page
			uint32_t _lAddr = __eeprom_find_empty_line(page);
			if ( _lAddr < EEPROM_START_ADDR ) {
				#if ( ITSDK_LOGGER_MODULE & __LOG_MOD_EEPROM ) > 0
					log_error("Failed to get line in page supposed to have some in page %d for vAdd %08X\r\n",page,_offset);
				#endif
				return false;
			}

			// Write the new line
			failure = ! __eeprom_write_and_verify(&target,_lAddr);


		} else {
			// the memory line exist at the given address
			// is that unchanged ? else we need to move it...move it

			__eeprom_line_t l = (*(volatile __eeprom_line_t*)lAddr);
			target.updates = (l.updates != 3)?l.updates+1:3;
			shift =  _offset - EEPROM_LINE_ADDR_TO_ADDR(target.addr);								// calculate the address shift from the beginning of the line

			// Search where to save the new line, scan the whole eeprom if required
			uint32_t tAddr = __eeprom_find_empty_line(0xFF);

			if ( tAddr < EEPROM_START_ADDR ) {
				uint8_t page = (uint8_t)lAddr;
				if ( page == 0xFF ) {
					// need to compact the flash


				} else {
					// need to init this page
					__eeprom_page_init(page);
					// @TODO - what if failed

				}
			}

			// create the new line with the previously existing content when required
			int idx = 0;
			for (int i = 0 ; i < shift ; i++ ){
				target.data[idx] = l.data[idx];							// unknown value, keep 0xFF, no change
				idx++;
			}
			for (int i = 0 ; i < _len && i < _EEPROM_BYTE_PER_LINE-shift ; i++) {
				target.data[idx] = *_data;							// prepare data
				_data++;
				idx++;
			}
			while ( idx < _EEPROM_BYTE_PER_LINE ) {
				target.data[idx] = l.data[idx];								// keep the rest unchanged if any
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
	} while ( _offset < offset+len );

	return true;
}

/**
 * Read a block of data from the EEPROM
 * Offset is to add an offset to bank start - Offset is aligned don 32b word
 */
bool _eeprom_read(uint8_t bank, uint32_t offset, void * data, int len) {
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



// ======================================================================
// Test Function
// ======================================================================


// structure for test with an occupation of 11 lines
// structure verification with sum field, making a sum of the other field
// fields not aligned on lines
typedef struct {
	uint8_t		u8_1;	// 64b
	uint16_t 	u16_1;
	uint32_t	u32_1;
	uint8_t		u8_2;

	uint8_t		u8_3[6];	// end of line 1

	uint32_t	u32_2[10];
	uint16_t	u16_2;		// end of line 2,3,4,5

	uint16_t 	u16_3;
	uint32_t	u32_3[10];	// end of line 6,7,8,9

	uint32_t	u32_4[4];	// end of line 10 + 4Bytes

	uint32_t	sum;		// in line 11

} __test_struct;

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

	log_info("EE-TEST-1 - eeprom start addr %08X\r\n",EEPROM_START_ADDR);

	// clear all the pages to get started
	for ( int p = 0 ; p < EEPROM_TOTAL_PAGES ; p++ ){
		if ( !__eeprom_page_clear(p) ) {
			log_error("EE-TEST - Failed to clear page %d\r\n",p);
			goto failed;
		}
	}
	log_info("EE-TEST-1 - pages cleared\r\n");

	// write 1 char with value 0xA5 at address 4, this may create a first page and a first line with values
	// FF FF FF FF A5 FF FF FF FF FF FF FF FF FF
	// in memory (R for reserved)
	// Page : 11 00000000000000 916E  FF FF FF 01   FFFF A55A   FFFFFFFF
	//        R  Aging          Magic R	 R  R  Ver  R    Ready  R
	//
	// Line : FF FF 1101 00 00.0000.0001 | FF A5 FF FF | FF FF FF FF | FF FF FF FF
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
	if ( h.magic != EEPROM_MAGIC || h.version != EEPROM_VERSION || h.state != 0xA55A || h.aging != 0 ) {
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

	// ----
	// create a full line, aligned address @ 4*_EEPROM_BYTE_PER_LINE ( for alignment )
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
		|| l.addr != 0x01D
	    || l.updates != 0x00
		|| __eeprom_compute_check(&l) != l.check
	) {
		log_error("EE-TEST-3 - Failed validate full aligned line\r\n");
		goto failed;
	}
	log_info("EE-TEST-3 - 0x%d aligned write success, verified\r\n",addr);

	// ---
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


	// ---
	// create a full line, not aligned address @ 2*_EEPROM_BYTE_PER_LINE+4 (aligned)
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
		|| l.addr != 0x00F
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
		|| l.addr != 0x016
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

	// ---
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
	if ( h.magic != EEPROM_MAGIC || h.version != EEPROM_VERSION || h.state != 0xA55A || h.aging != 0 ) {
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
	if ( stat.allocated_pages != 2 || stat.trashed_lines != 6 || stat.allocated_lines != 133 ) {
		log_error("EE-TEST-11 - eeprom stats are not coherent\r\n");
		goto failed;
	}
	log_info("EE-TEST-11 - number of trashed lines & allocate page is coherent\r\n",addr);





	log_info("EE-TEST - End success\r\n");
	return;

	failed:
	 log_error("EE-TEST - End with failure\r\n");

}


#endif
