/* ==========================================================
 * rtc.c -  Real Time Clock Peripheral
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 2 sept. 2018
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
 * For clock adjstement, TIM21 must be enabled in cubMX setting clock source to InternalClock
 *
 * ==========================================================
 */
#include <it_sdk/config.h>
#include <it_sdk/time/time.h>
#include <it_sdk/logger/logger.h>
#include <stm32l_sdk/rtc/rtc.h>
#include "time.h"
#if ITSDK_WITH_CLK_ADJUST > 0
#include "tim.h"
#endif

/**
 * Configure the RTC source clock for running LowPower
 */
void rtc_configure4LowPower(uint16_t ms) {
	rtc_prepareSleepTime();
	rtc_runRtcUntil(ms);
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
    uint32_t _time = (((uint32_t)ms) * 2314) / 1000;
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, _time, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}

/*
 * Convert a duration in ticks
 */
uint32_t rtc_getTicksFromDuration(uint32_t ms) {
    return (ms * 2314) / 1000;
}

int32_t rtc_getMsFromTicks(uint32_t ticks) {
	return (ticks * 1000) / 2314;
}

/**
 * Run the RTC for a given number of tics
 */
void rtc_runRtcUntilTicks(uint16_t ticks) {
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ticks, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}


void rtc_disableWakeUp() {
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
}

/**
 * Get the current timestamp in uS (this is not considering any specific date stuff...)
 * This function costs a lot a flash byte ... not to be used with small devices
 */
#ifndef __WE_HAVE_A_LOT_OF_FLASH
uint32_t days = 0;			// day index since the begining
uint32_t lastTick = 0;		// time in ms in the day
#endif
uint64_t rtc_getTimestampMs() {
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
	return ms;
#else
	RTC_TimeTypeDef _time;
	RTC_DateTypeDef _date;
	uint32_t ms;
	HAL_RTC_GetTime(&hrtc, &_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &_date, RTC_FORMAT_BIN);
	ms  = (uint32_t)_time.Hours*3600*1000;
	ms += (uint32_t)_time.Minutes*60*1000;
	ms += (uint32_t)_time.Seconds*1000;
	ms += ((1000*(uint32_t)(_time.SecondFraction-_time.SubSeconds))/_time.SecondFraction+1);

	if ( ms < lastTick ) {
		// day has changed
		days++;
	}
	lastTick = ms;
	return ( uint64_t )(days*3600000L*24L)+ms;

#endif
}


/**
 * Reset RTC to 00:00:00.00 at startup
 */
void rtc_resetTime() {
	log_info("Reset Time\r\n");
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
	lastTick = 0;
}


/**
 * Call before any sleep in case there is something to prepare with RTC
 * or others.
 */
void rtc_prepareSleepTime() {
	__enable_systick=false;
}



/**
 * Get the sleep duration based on RTC counter
 */
void rtc_updateTimeAfterSleepTime() {

	itsdk_time_set_ms(rtc_getTimestampMs());
/*
	RTC_TimeTypeDef _time;
	RTC_DateTypeDef _date;
	uint32_t	ms;
	HAL_RTC_GetTime(&hrtc, &_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &_date, RTC_FORMAT_BIN);	// unlock date after time read

	ms  = 0; // (uint32_t)_time.Hours*3600*1000; -- we should never sleep more than 1 hour ...
	ms += (uint32_t)_time.Minutes*60*1000;
	ms += (uint32_t)_time.Seconds*1000;
	ms += ((1000*(uint32_t)(_time.SecondFraction-_time.SubSeconds))/_time.SecondFraction+1);
	itsdk_time_add_us(ms*1000);
*/
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




/**
  * Measure the LSI frequency to adjust the RTC and other peripheral working with LSI
  */

#if ITSDK_WITH_CLK_ADJUST > 0

uint16_t __tmpCC4[2] = {0, 0};
__IO uint32_t __uwLsiFreq = 0;
__IO uint32_t __uwCaptureNumber = 0;

uint32_t rtc_getRealRtcFrequency()
{
  htim21.Instance = TIM21;
  htim21.Init.Prescaler         = 0;
  htim21.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim21.Init.Period            = 0xFFFF;
  htim21.Init.ClockDivision     = 0;
  HAL_TIM_IC_Init(&htim21);

  HAL_TIMEx_RemapConfig(&htim21, TIM21_TI1_LSI);
  TIM_IC_InitTypeDef    TIMInput_Config;
  TIMInput_Config.ICPolarity  = TIM_ICPOLARITY_RISING;
  TIMInput_Config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  TIMInput_Config.ICPrescaler = TIM_ICPSC_DIV8;
  TIMInput_Config.ICFilter    = 0;
  HAL_TIM_IC_ConfigChannel(&htim21, &TIMInput_Config, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim21, TIM_CHANNEL_1);
  while(__uwCaptureNumber != 2) { }

  HAL_TIM_IC_Stop_IT(&htim21, TIM_CHANNEL_1);
  HAL_TIM_IC_DeInit(&htim21);

  return __uwLsiFreq;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint32_t lsiperiod = 0;

  __tmpCC4[__uwCaptureNumber++] = HAL_TIM_ReadCapturedValue(&htim21, TIM_CHANNEL_1);
  if (__uwCaptureNumber >= 2)
  {
    lsiperiod = (uint16_t)(0xFFFF - __tmpCC4[0] + __tmpCC4[1] + 1);
    __uwLsiFreq = (uint32_t) SystemCoreClock / lsiperiod;
    __uwLsiFreq *= 8;
  }
}

#endif


