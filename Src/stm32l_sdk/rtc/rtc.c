/* ==========================================================
 * rtc.c -  Real Time Clock Peripheral
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
 * For clock adjstement, TIM21 must be enabled in cubMX setting clock source to InternalClock
 *
 * ==========================================================
 */
#include <it_sdk/config.h>
#if ITSDK_WITH_RTC != __RTC_NONE
#include <it_sdk/time/time.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/lowpower/lowpower.h>
#include <stm32l_sdk/rtc/rtc.h>
#include "time.h"
#if ITSDK_WITH_CLK_ADJUST > 0
#include <it_sdk/time/timer.h>
#endif


/**
 * Configure the RTC source clock for running LowPower
 */
void rtc_configure4LowPower(uint16_t ms) {
	rtc_prepareSleepTime();
	if ( ms > 0 ) {
		rtc_runRtcUntil(ms);
	}
}

/**
 * Deactivate the WakeUpTimer for not having the IRQ looping
 */
void rtc_disable4LowPower() {
	rtc_disableWakeUp();
    rtc_updateTimeAfterSleepTime();
}


/**
 * Run Rtc for a given time in ticks
 * Max is 16s
 */
void rtc_runRtcUntil(uint16_t ms) {
	// Issue #48
	// it seems that timer is limited to 16bis (even if 32 bits in the HAL code)
	// So this is limiting in about 30s of sleeping time
	uint32_t ticks = rtc_getTicksFromDuration((uint32_t)ms);
	if ( ticks < 65536 ) {
	    rtc_runRtcUntilTicks(ticks);
	} else {
		// If larger than 65535 we change the RTC cloc to have a 1s timabase and a longer period of time
		rtc_runRtcUntilMs(ms);
	}
}

/*
 * Convert a duration in ticks (Wake-Up Clock only)
 */
uint32_t rtc_getTicksFromDuration(uint32_t ms) {
    return (ms * (ITSDK_RTC_CLKFREQ/16)) / 1000;
}

/*
 * Convert a tick duration in ms (Wake-Up Clock only)
 */
int32_t rtc_getMsFromTicks(uint32_t ticks) {
	return (ticks * 1000) / (ITSDK_RTC_CLKFREQ/16);
}

/**
 * Run the RTC for a given number of ticks
 */
void rtc_runRtcUntilTicks(uint32_t ticks) {
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ticks, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}

/**
 * Run the RTC for a given number of ms
 */
void rtc_runRtcUntilMs(uint32_t ms) {
	// the scale is 1 second
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, (ms / 1000), RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
}


void rtc_disableWakeUp() {
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
}

/**
 * Get the current timestamp in uS (this is not considering any specific date stuff...)
 * This function costs a lot a flash byte ... not to be used with small devices
 */
#if ITSDK_WITH_CLK_ADJUST > 0
uint8_t		__rtc_init=0;								// clock ratio is already initialized
uint64_t 	__rtc_offset;								// time offset for current ratio
uint32_t	__rtc_currentRatio;							// default clock ratio
#endif

#ifndef __WE_HAVE_A_LOT_OF_FLASH
uint32_t __rtc_days = 0;			// day index since the begining
uint32_t __rtc_lastTick = 0;		// time in ms in the day
#endif
uint64_t rtc_getTimestampMs() {
	return rtc_getTimestampMsRaw(true);
}

uint64_t rtc_getTimestampMsRaw(bool adjust) {
#ifdef __WE_HAVE_A_LOT_OF_FLASH
	RTC_DateTypeDef _date;
	RTC_TimeTypeDef _time;
	struct tm currTime;
	time_t timestamp;

	HAL_RTCEx_GetTimeStamp(&hrtc, &_time, &_date, RTC_FORMAT_BCD);
	currTime.tm_year = _date.Year + 100;  // In fact: 2000 + 18 - 1900
	currTime.tm_mday = _date.Date;
	currTime.tm_mon  = _date.Month - 1;

	currTime.tm_hour = _time.Hours;
	currTime.tm_min  = _time.Minutes;
	currTime.tm_sec  = _time.Seconds;

	timestamp = mktime(&currTime);
	uint64_t ms = (timestamp*1000) + ((1000*(uint32_t)(_time.SecondFraction-_time.SubSeconds))/_time.SecondFraction+1);
#else
	RTC_TimeTypeDef _time;
	RTC_DateTypeDef _date;
	uint64_t ms;
	HAL_RTC_GetTime(&hrtc, &_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &_date, RTC_FORMAT_BIN);
	ms  = (uint32_t)_time.Hours*3600*1000;
	ms += (uint32_t)_time.Minutes*60*1000;
	ms += (uint32_t)_time.Seconds*1000;
	ms += ((1000*(uint32_t)(_time.SecondFraction-_time.SubSeconds))/_time.SecondFraction+1);

	if ( ms < __rtc_lastTick ) {
		// day has changed
		__rtc_days++;
	}
	__rtc_lastTick = ms;
	ms = ( uint64_t )((uint64_t)__rtc_days*3600000L*24L)+(uint64_t)ms;
#endif
	// apply the RTC clock correction and add previous offset
	#if ITSDK_WITH_CLK_ADJUST > 0
		if (adjust && __rtc_init > 0) {
			ms = (ms * (uint64_t)__rtc_currentRatio) / 1000L;
			ms += __rtc_offset;
		}
	#else
		ms = (adjust)?(ms * ITSDK_CLK_CORRECTION) / 1000L:ms;
	#endif
	return ms;
}


/**
 * Reset RTC to 00:00:00.00 at startup
 */
void rtc_resetTime() {
	RTC_DateTypeDef _date;
	_date.Year = 0;
	_date.Month = 1;
	_date.Date = 1;
	HAL_RTC_SetDate(&hrtc,&_date,RTC_FORMAT_BIN);

	RTC_TimeTypeDef _time;
	_time.Hours 		 = 0x0;
	_time.Minutes 		 = 0x0;
	_time.Seconds 		 = 0x0;
	_time.SubSeconds  	 = 0x00;
	_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	_time.StoreOperation = RTC_STOREOPERATION_RESET;
	HAL_RTC_SetTime(&hrtc, &_time, RTC_FORMAT_BIN);
	__rtc_lastTick = 0;
}


/**
 * Call before any sleep in case there is something to prepare with RTC
 * or others.
 */
void rtc_prepareSleepTime() {
	__enable_systick=false;

//	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
//	HAL_PWR_DisableWakeUpPin( PWR_WAKEUP_PIN1 );
//	HAL_PWR_DisableWakeUpPin( PWR_WAKEUP_PIN2 );

	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}



/**
 * Get the sleep duration based on RTC counter
 */
void rtc_updateTimeAfterSleepTime() {

	itsdk_time_set_ms(rtc_getTimestampMs());
	__enable_systick=true;
}


/**
 * RCT Interrupt handler allowing to chain different function
 */
rtc_irq_chain_t __rtc_irq_chain = { NULL, NULL };
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	rtc_irq_chain_t * c = &__rtc_irq_chain;
	while ( c != NULL ) {
		void (*p)(RTC_HandleTypeDef *h) = c->irq_func;
		if ( p != NULL ) {
			p(hrtc);
		}
		c = c->next;
	}
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__lowPower_wakeup_reason=LOWPWR_WAKEUP_RTC;
}

/**
 * Add an action to the chain, the action **must be** static
 */
void rtc_registerIrqAction(rtc_irq_chain_t * chain) {
	rtc_irq_chain_t * c = &__rtc_irq_chain;
	while ( c->next != NULL && c->irq_func != chain->irq_func ) {
	  c = c->next;
	}
	if ( c->irq_func != chain->irq_func ) {
		// the Action is not already existing
		c->next=chain;
		chain->next = NULL;
	}

}

/**
 * Remove an action to the chain, the action **must be** static
 */
void rtc_removeIrqAction(rtc_irq_chain_t * chain) {
	rtc_irq_chain_t * c = &__rtc_irq_chain;
	while ( c != NULL && c->next != chain ) {
	  c = c->next;
	}
	if ( c != NULL ) {
		c->next = c->next->next;
	}
}

/**
 * Search for an existing action
 */
bool rtc_existAction(rtc_irq_chain_t * chain) {
	rtc_irq_chain_t * c = &__rtc_irq_chain;
	while ( c != NULL && c->next != chain ) {
	  c = c->next;
	}
	if ( c != NULL ) {
		return true;
	}
	return false;
}



/** ========================================================================
 * Adjust the RTC frequency based on LSI (Timer) clock
 */

/**
 * Return the last computed ratio
 */
uint32_t rtc_getClockAdjustement() {
#if ITSDK_WITH_CLK_ADJUST > 0
	if (__rtc_init > 0) {
		return __rtc_currentRatio;
	} else {
		return rtc_calcClockRatio();
	}
#else
	return ITSDK_CLK_CORRECTION;
#endif
}

/**
 * Manage rtc clock adjustement / (re)evaluate the clock ratio
 * Can be called at anytime to reajust
 */
void rtc_adjustTime() {
#if ITSDK_WITH_CLK_ADJUST > 0
	uint32_t newRatio=rtc_calcClockRatio();
	if (__rtc_init > 0) {
		__rtc_offset = rtc_getTimestampMs();
		rtc_resetTime();
	} else {
		__rtc_offset=0;
	}
	__rtc_init=1;
	__rtc_currentRatio=newRatio;
#endif
}


/**
 * Return the corrected clockRatio => realClock = (calcClockRatio * seenClock)/1000
 */
uint32_t rtc_calcClockRatio() {
#if ITSDK_WITH_CLK_ADJUST > 0 && ITSDK_CLK_BEST_SOURCE == __CLK_BEST_SRC_CLK

	// timer test
	uint64_t start = rtc_getTimestampMsRaw(false);
	itsdk_hwtimer_sync_run(
			200,
			NULL,
			0
	);
	uint64_t stop = rtc_getTimestampMsRaw(false);
	uint64_t ratio = (1000*200)/(stop-start);

	// Protection against value too bad, sounds like a problem
	if ( ratio > 1400 || ratio < 600 ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_RTC_INVALID_CLKRATIO,(uint16_t)ratio);
		ratio = 1000;
	}

	return (uint32_t)ratio;

#else
	return ITSDK_CLK_CORRECTION;
#endif
}

#endif
