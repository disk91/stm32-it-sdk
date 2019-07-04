 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic lora driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    timeserver.c
  * @author  MCD Application Team
  * @brief   Time server infrastructure
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <it_sdk/itsdk.h>
#include <it_sdk/time/timer.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>

#include <time.h>
#include <drivers/lorawan/timeServer.h>


/*!
 * Timers list head pointer
 */
static TimerEvent_t *TimerListHead = NULL;

/*!
 * \brief Adds or replace the head timer of the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be become the new head
 * \param [IN]  remainingTime Remaining time of the previous head to be replaced
 */
//static void TimerInsertNewHeadTimer( TimerEvent_t *obj );

/*!
 * \brief Adds a timer to the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be added to the list
 * \param [IN]  remainingTime Remaining time of the running head after which the object may be added
 */
static void TimerInsertTimer( TimerEvent_t *obj );

/*!
 * \brief Sets a timeout with the duration "timestamp"
 *
 * \param [IN] timestamp Delay duration
 */
//static void TimerSetTimeout( TimerEvent_t *obj );

/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */
static bool TimerExists( TimerEvent_t *obj );



/** ***********************************************************************
 * List Manipulation
 */

/**
 * Remove a element from the list
 */
static void removeFromList( TimerEvent_t *obj ) {
	if( TimerListHead == obj ) {
		TimerListHead = TimerListHead->Next;
	} else {
		TimerEvent_t* cur = TimerListHead;
		while( cur != NULL ) {
		   if( cur->Next == obj ) {
			   cur->Next = obj->Next;
			   break;
		   }
		   cur = cur->Next;
		}
	}
}

/**
 * Check if the Object to be added is not already in the list
 */
static bool TimerExists( TimerEvent_t *obj )
{
  TimerEvent_t* cur = TimerListHead;

  while( cur != NULL ) {
    if( cur == obj ) {
      return true;
    }
    cur = cur->Next;
  }
  return false;

}

/**
 * Insert the timer, as we manage it with no order, just put it on the head
 */
static void TimerInsertTimer( TimerEvent_t *obj)
{
	obj->Next = TimerListHead;
	TimerListHead = obj;
}



/** *********************************************************************************
 * This is the callback used for all the timer, it calls the callback function
 */
static void TimerCallback( uint32_t value ) {

	TimerEvent_t *obj = (TimerEvent_t *)value;
	LOG_DEBUG_LORAWAN(("TimerCallback (%d)\r\n",obj->ReloadValue));
	obj->IsStarted = false;
	if (obj->Callback != NULL) {
		obj->Callback(obj->Context);
	} else {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_TIME_NOCALLBACK,0);
	}
	removeFromList(obj);
}

/** ***********************************************************************************
 * Add / remove timer ...
 */

void TimerInit( TimerEvent_t *obj, void ( *callback )( void *context ) )
{
  LOG_DEBUG_LORAWAN(("TimerInit\r\n"));

  obj->Timestamp = 0;
  obj->ReloadValue = 0;
  obj->IsStarted = false;
  obj->IsNext2Expire = false;
  obj->Callback = callback;
  obj->Context = NULL;
  obj->Next = NULL;
}



/**
 * This is changing the parameter pass to the callback function at timer expiration
 * Here we search the itsdk timer structure based on previous context value then we
 * change it. Both structure are kept up-to-date.
 */
void TimerSetContext( TimerEvent_t *obj, void* context )
{
	LOG_DEBUG_LORAWAN(("TimerSetContext\r\n"));
	obj->Context = context;
}

/**
 * This is changing the duration of the timer. The value is given in ms.
 * We search for the itsdk timer structure and update it when running.
 * If not running the timestamp field will keep the value in ms.
 */
void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
	LOG_DEBUG_LORAWAN(("TimerSetValue %d\r\n",value));
	// search the real timer based on the context
	itsdk_stimer_slot_t * t = itsdk_stimer_get(TimerCallback,(uint32_t)obj);
	if ( t != NULL ) {
		// best is to stop the timer and restart it with the new duration
		TimerStop(obj);
		obj->Timestamp = value;
		obj->ReloadValue = value;
		TimerStart(obj);
	} else {
		// the timer is not running, so we just need to update the local structure
		obj->Timestamp = value;
		obj->ReloadValue = value;
	}
}

/**
 * Sets a timeout with the duration "timestamp"
 * Assuming this is just called internally for setting up the RTC Alarm on next event
 */


/**
 * Add a Timer in the list and start it using the it_sdk timer module
 */
void TimerStart( TimerEvent_t *obj )
{
	LOG_INFO_LORAWAN(("St %d ms\r\n",obj->ReloadValue));

	itsdk_enterCriticalSection();
	// do not add a timer already existing
	if( ( obj == NULL ) || ( TimerExists( obj ) == true ) ) {
		itsdk_leaveCriticalSection();
		ITSDK_ERROR_REPORT(ITSDK_ERROR_STIMER_ALREADY_SET,0);
	    return;
	}
	obj->Timestamp = obj->ReloadValue;
	obj->IsStarted = true;
	obj->IsNext2Expire = false;

	if( TimerListHead == NULL ) {
		obj->Next = NULL;
		TimerListHead = obj;
	} else {
	  // obj->Timestamp += elapsedTime; Not needed
      TimerInsertTimer( obj);
	}
	itsdk_timer_return_t ret = itsdk_stimer_register(
									obj->ReloadValue,
									TimerCallback,
									(uint32_t)obj,
									TIMER_ACCEPT_LOWPOWER
		 	 	 	 	 	   );
	if ( ret != TIMER_INIT_SUCCESS ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_TIME_INITFLD,(uint16_t)ret);
	}
	itsdk_leaveCriticalSection();

}


/**
 *
 */
void TimerStop( TimerEvent_t *obj ) 
{
	LOG_INFO_LORAWAN(("Sp %d ms\r\n",obj->ReloadValue));

	itsdk_enterCriticalSection();
	// do not stop a non existing
	if( ( obj == NULL ) || ( TimerExists( obj ) == false ) ) {
		itsdk_leaveCriticalSection();
	    return;
	}

	if (obj->IsStarted) {
		itsdk_stimer_stop(
							TimerCallback,
							(uint32_t)obj
						);
		obj->IsStarted = false;
	}
	removeFromList(obj);
	itsdk_leaveCriticalSection();
}  


void TimerReset( TimerEvent_t *obj )
{
  TimerStop( obj );
  TimerStart( obj );
}

bool TimerIsStarted( TimerEvent_t *obj )
{
  return obj->IsStarted;
}




TimerTime_t TimerGetCurrentTime( void )
{
	return (uint32_t)itsdk_time_get_ms();
}

TimerTime_t TimerGetElapsedTime( TimerTime_t past )
{
	return TimerGetCurrentTime() - past;
}



TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature )
{
	return period;
   // return RtcTempCompensation( period, temperature );
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
