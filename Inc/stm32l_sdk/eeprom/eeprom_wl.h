/* ==========================================================
 * eeprom_wl.h - Manage EEPROM (NVM) for stm32WL
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 03 may. 2024
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018  Disk91
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

#ifndef STM32L_SDK_EEPROM_EEPROM_WL_H_
#define STM32L_SDK_EEPROM_EEPROM_WL_H_
#include <stdbool.h>


//#define __EEPROM_WITH_TEST										// disable the test code, only uncomment if eeprom driver modifications are made

#define EEPROM_PAGE_SIZE 		2048							// 2K pages - hardware related, not a parameter
#define EEPROM_LINE_PER_PAGE	126								// Number of Lines (memory block per page)
#define EEPROM_SIZE_WITH_OVERHEAD (ITSDK_EPROM_SIZE + 4096)		// Add 4K (2 pages, for page switch and structure overhead, works with 2K-8K eeprom)
																// @TODO - improve this with not fixed value but parameter including the aging
																// currently, best is to commission more memory than required.
#define EEPROM_AGING_REPORT		8000							// when a page has been reset this amount of time, report it to user

#if ( EEPROM_SIZE_WITH_OVERHEAD % EEPROM_PAGE_SIZE ) == 0
   #define EEPROM_TOTAL_PAGES  (( EEPROM_SIZE_WITH_OVERHEAD / EEPROM_PAGE_SIZE ))
#else
   #define EEPROM_TOTAL_PAGES  (( EEPROM_SIZE_WITH_OVERHEAD / EEPROM_PAGE_SIZE ) + 1 )
#endif


/**
 * Get the statistics from the NVM management
 */
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
	uint8_t		trashed_lines_per_page[EEPROM_TOTAL_PAGES];		// # of lines updated and now trashed
	uint8_t		free_lines_per_page[EEPROM_TOTAL_PAGES];		// # of free line allocable
	uint8_t		untouched_lines_per_page[EEPROM_TOTAL_PAGES];	// # of line containing address never updated
	uint16_t	age[EEPROM_TOTAL_PAGES]; 						// page age
	uint8_t		state[EEPROM_TOTAL_PAGES];						// page status F(ree) I(nit) M(ove IN) R(eady) (Move OU)T
} __eeprom_stat_t;

bool _eeprom_stats(__eeprom_stat_t * s);

// callback used by garbage collector (for virtual eeprom only)
void __eeprom_onGarbageCallback();

// callback used when a flash memory issue appears
typedef enum {
	EEPROM_WARN_UNKNOWN,
	EEPROM_WARN_PAGE_AGE_IS_HIGH,			// When the page age is passing EEPROM_AGING_REPORT, risk of flash hw failure soon
	EEPROM_WARN_NO_FREE_PAGE,				// Impossible to get a free page but this is not blocking
	EEPROM_WARN_GARBAGE_FAILED,				// Problem during garbage collection, not critical, eventually see other error fired previously

	EEPROM_ERR,								// used to have a ERR vs WARN easy test (never used)
	EEPROM_ERR_FAILED_TO_RESET_PAGE,		// Impossible to reset the flash page
	EEPROM_ERR_NO_FREE_PAGE,				// Impossible to get a free page for garbaging
	EEPROM_ERR_PAGE_MOVE_FAILED,			// Impossible to move a page to another for garbaging
	EEPROM_ERR_GARBAGE_COMMIT,				// Failed to commit the garbage collection, page not switched ready
	EEPROM_ERR_GARBAGE_FAILED,				// Critical error during garbage collection, eventually see other error fired previously
	EEPROM_ERR_GARBAGE_REPAIR_FAILED,		// Failed to repair eepom after garbage collection
} __eeprom_error_t;

void __eeprom_onFlashErrorCallback( __eeprom_error_t errCode );



#endif /* STM32L_SDK_EEPROM_EEPROM_WL_H_ */
