/* ==========================================================
 * time.c - Functions about counting time
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
#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/time/time.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
	#include <stm32l_sdk/rtc/rtc.h>
	#include <stm32l_sdk/time/time.h>
#endif

volatile uint64_t __timeus = 0;
uint8_t  __time_has_overrun = 0;
uint8_t  __time_overrun_cnt = 0;

uint64_t __time_epoc_reference_uS = 0;		// store the local time reference corresponding to 00:00:00 utc time
uint32_t __time_epoc_atreference_S = 0;		// store the UTC time corresponding to the local reference

uint64_t __time_utc_reference_uS = 0;		// store the local time reference corresponding to 00:00:00 utc time
uint32_t __time_utc_atreference_S = 0;		// store the UTC time corresponding to the local reference



/**
 * Add the given number of uS to the uS global timer
 */
void itsdk_time_add_us(uint32_t us) {
	// apply correction
	#if ITSDK_CLK_CORRECTION != 0
	us = us + ( (int64_t)us * ITSDK_CLK_CORRECTION ) / 1000;
	#endif
	uint64_t n = __timeus + us;
	if ( n < __timeus  ) {
		__time_has_overrun=1;
		__time_overrun_cnt++;
	}
	__timeus = n;
}

/**
 * Set current time in ms
 */
void itsdk_time_set_ms(uint64_t ms) {
	uint64_t n = ms * 1000L;
	if ( (__timeus - n) > 1000000L   ) {	// difference is > 1m assuming the counter has restarted
		__time_has_overrun=1;
		__time_overrun_cnt++;
	}
	__timeus = n;
}

/**
 * Get current time in ms
 */
uint64_t itsdk_time_get_ms() {
	return __timeus / 1000;
}

/**
 * Get current time in us
 */
uint64_t itsdk_time_get_us() {
	return __timeus;
}


/**
 * Sync the UTC Time reference
 * With the second count since midnight
 */
void itsdk_time_sync_UTC_s( uint32_t utc_s ){
	__time_utc_reference_uS = itsdk_time_get_us();
	__time_utc_atreference_S = utc_s;
}

/**
 * Get the current UTC time
 * Return the time in S when utc reference have been set 0 otherwise
 */
uint32_t itsdk_time_get_UTC_s(){
	if ( __time_utc_atreference_S == 0 ) return 0;
	uint64_t t = (itsdk_time_get_us() - __time_utc_reference_uS) / 1000000;
	return __time_utc_atreference_S + t;
}

/**
 * Get the current hour of the day
 */
uint8_t itsdk_time_get_UTC_hour(){
	if ( __time_utc_atreference_S == 0 ) return 0;
	uint64_t t = (itsdk_time_get_us() - __time_utc_reference_uS) / 1000000;
	t = __time_utc_atreference_S + t;

	t /= 3600;
	t %= 24;

	return t;
}

/**
 * Get the current minute of the day
 */
uint8_t itsdk_time_get_UTC_min(){
	if ( __time_utc_atreference_S == 0 ) return 0;
	uint64_t t = (itsdk_time_get_us() - __time_utc_reference_uS) / 1000000;
	t = __time_utc_atreference_S + t;

	t /= 60;
	t %= 60;

	return t;
}

/**
 * Get the current minute of the day
 */
uint8_t itsdk_time_get_UTC_sec(){
	if ( __time_utc_atreference_S == 0 ) return 0;
	uint64_t t = (itsdk_time_get_us() - __time_utc_reference_uS) / 1000000;
	t = __time_utc_atreference_S + t;
	t %= 60;
	return t;
}




/**
 * Get the current UTC time
 * Return true when utc reference have been and store the current time in
 * the given parameter if not null. Return false if not set and param contains the
 * duration in S since reset
 */
itsdk_bool_e itsdk_time_is_UTC_s(uint32_t * destTime) {
	if ( destTime != NULL ) {
		uint64_t t = (itsdk_time_get_us() - __time_utc_reference_uS) / 1000000;
		*destTime = (uint32_t)(__time_utc_atreference_S + t);
	}
	if ( __time_utc_atreference_S == 0 ) return BOOL_FALSE;
	return BOOL_TRUE;
}





/**
 * Sync the EPOC Time reference
 * With the second count since EPOC
 */
void itsdk_time_sync_EPOC_s( uint32_t utc_s ){
	__time_epoc_reference_uS = itsdk_time_get_us();
	__time_epoc_atreference_S = utc_s;
}

/**
 * Get the current EPOC time
 * Return the time in S when utc reference have been set 0 otherwise
 */
uint32_t itsdk_time_get_EPOC_s(){
	if ( __time_epoc_atreference_S == 0 ) return 0;
	uint64_t t = (itsdk_time_get_us() - __time_epoc_reference_uS) / 1000000;
	return __time_epoc_atreference_S + t;
}



/**
 * Get the current EPOC time
 * Return true when EPOC reference have been and store the current time in
 * the given parameter if not null. Return false if not set and param contains the
 * duration in S since reset
 */
itsdk_bool_e itsdk_time_is_EPOC_s(uint32_t * destTime) {
	if ( destTime != NULL ) {
		uint64_t t = (itsdk_time_get_us() - __time_epoc_reference_uS) / 1000000;
		*destTime = (uint32_t)(__time_epoc_atreference_S + t);
	}
	if ( __time_epoc_atreference_S == 0 ) return BOOL_FALSE;
	return BOOL_TRUE;
}



/**
 * Reset the time to 0
 */
void itsdk_time_reset() {
    #if ITSDK_WITH_RTC != __RTC_NONE
      #if ITSDK_PLATFORM == __PLATFORM_STM32L0
  		rtc_resetTime();
  	  #else
		#error "platform not supported"
	  #endif
    #endif
	__timeus = 0;
}

/**
 * Init time functions
 */
void itsdk_time_init() {
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
  #if ITSDK_WITH_RTC != __RTC_NONE
	rtc_resetTime();
	rtc_adjustTime();
  #endif
	systick_adjustTime();
  #if ITSDK_WITH_RTC != __RTC_NONE
	itsdk_time_set_ms(rtc_getTimestampMs());
  #endif
#else
	#error "platform not supported"
#endif
}
