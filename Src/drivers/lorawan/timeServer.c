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

#include <time.h>
#include <drivers/lorawan/timeServer.h>
//#include "low_power.h"



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

//static void TimerInsertNewHeadTimer( TimerEvent_t *obj )
//{
//  TimerEvent_t* cur = TimerListHead;
//
//  if( cur != NULL )
//  {
//    cur->IsNext2Expire = false;
//  }
//
//  obj->Next = cur;
//  TimerListHead = obj;
//  TimerSetTimeout( TimerListHead );
//}

/*
  TimerEvent_t* cur = TimerListHead;
  TimerEvent_t* next = TimerListHead->Next;

  while (cur->Next != NULL )
  {
    if( obj->Timestamp  > next->Timestamp )
    {
        cur = next;
        next = next->Next;
    }
    else
    {
        cur->Next = obj;
        obj->Next = next;
        return;

    }
  }
  cur->Next = obj;
  obj->Next = NULL;
}
*/


/** *********************************************************************************
 * This is the callback used for all the timer, it calls the callback function
 */
static void TimerCallback( uint32_t value ) {

	TimerEvent_t *obj = (TimerEvent_t *)value;
	log_debug("TimerCallback (%d)\r\n",obj->ReloadValue);

	obj->IsStarted = false;
	if (obj->Callback != NULL) {
		obj->Callback(obj->Context);
	} else {
		log_error("Callback function is NULL \r\n");
		_Error_Handler(__FILE__,__LINE__);
	}
	removeFromList(obj);
}

/** ***********************************************************************************
 * Add / remove timer ...
 */

void TimerInit( TimerEvent_t *obj, void ( *callback )( void *context ) )
{
  log_debug("TimerInit\r\n");

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
	log_debug("TimerSetContext\r\n");
	obj->Context = context;
}

/**
 * This is changing the duration of the timer. The value is given in ms.
 * We search for the itsdk timer structure and update it when running.
 * If not running the timestamp field will keep the value in ms.
 */
void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
	log_debug("TimerSetValue %d\r\n",value);
	// search the real timer based on the context
	itsdk_stimer_slot_t * t = itsdk_stimer_get(TimerCallback,(uint32_t)obj);
	if ( t != NULL ) {
		// best is to stop the timer and restart it with the new duration
		TimerStop(obj);
		obj->Timestamp = (value*110)/100;
		obj->ReloadValue = (value*110)/100;
		TimerStart(obj);
	} else {
		// the timer is not running, so we just need to update the local structure
		obj->Timestamp = (value*110)/100;
		obj->ReloadValue = (value*110)/100;
	}

/*
  uint32_t minValue = 0;
  uint32_t ticks = HW_RTC_ms2Tick( value );

  TimerStop( obj );

  minValue = HW_RTC_GetMinimumTimeout( );

  if( ticks < minValue )
  {
    ticks = minValue;
  }

  obj->Timestamp = ticks;
  obj->ReloadValue = ticks;
  */
}

/**
 * Sets a timeout with the duration "timestamp"
 * Assuming this is just called internally for setting up the RTC Alarm on next event
 */
/*
static void TimerSetTimeout( TimerEvent_t *obj )
{
  int32_t minTicks= HW_RTC_GetMinimumTimeout( );
  obj->IsNext2Expire = true;

  // In case deadline too soon
  if(obj->Timestamp  < (HW_RTC_GetTimerElapsedTime(  ) + minTicks) )
  {
    obj->Timestamp = HW_RTC_GetTimerElapsedTime(  ) + minTicks;
  }
  HW_RTC_SetAlarm( obj->Timestamp );
}
*/

/**
 * Add a Timer in the list and start it using the it_sdk timer module
 */
void TimerStart( TimerEvent_t *obj )
{
	log_info("Start timer for %d ms\r\n",obj->ReloadValue);

	itsdk_enterCriticalSection();
	// do not add a timer already existing
	if( ( obj == NULL ) || ( TimerExists( obj ) == true ) ) {
		itsdk_leaveCriticalSection();
		log_error("Timer error\r\n");
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
									(uint32_t)obj
		 	 	 	 	 	   );
	if ( ret != TIMER_INIT_SUCCESS ) {
		log_error("Error during timer initialization %d \r\n",ret);
		_Error_Handler(__FILE__,__LINE__);
	}
	itsdk_leaveCriticalSection();

//  uint32_t elapsedTime = 0;
//
//  BACKUP_PRIMASK();
//
//  DISABLE_IRQ( );
//
//
//  if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
//  {
//    RESTORE_PRIMASK( );
//    return;
//  }
//  obj->Timestamp = obj->ReloadValue;
//  obj->IsStarted = true;
//  obj->IsNext2Expire = false;

//  if( TimerListHead == NULL )
//  {
//    HW_RTC_SetTimerContext( );
//    TimerInsertNewHeadTimer( obj ); // insert a timeout at now+obj->Timestamp
//  }
//  else
//  {
//    elapsedTime = HW_RTC_GetTimerElapsedTime( );
//    obj->Timestamp += elapsedTime;
//
//    if( obj->Timestamp < TimerListHead->Timestamp )
//    {
//      TimerInsertNewHeadTimer( obj);
//    }
//    else
//    {
//      TimerInsertTimer( obj);
//    }
//  }
//  RESTORE_PRIMASK( );
}


/**
 *
 */
void TimerStop( TimerEvent_t *obj ) 
{
	log_info("Stop a timer waiting for %d ms\r\n",obj->ReloadValue);

	itsdk_enterCriticalSection();
	// do not stop a non existing
	if( ( obj == NULL ) || ( TimerExists( obj ) == false ) ) {
		itsdk_leaveCriticalSection();
	    return;
	}

	if (obj->IsStarted) {
		itsdk_timer_return_t ret = itsdk_stimer_stop(
											TimerCallback,
											(uint32_t)obj
									);
		if (ret == TIMER_NOT_FOUND) {
			log_warn("Timer to stop is not found or completed\r\n");
		}
		obj->IsStarted = false;
	}
	removeFromList(obj);
	itsdk_leaveCriticalSection();

//  BACKUP_PRIMASK();
//
//  DISABLE_IRQ( );
//
//  TimerEvent_t* prev = TimerListHead;
//  TimerEvent_t* cur = TimerListHead;
//
//  // List is empty or the Obj to stop does not exist
//  if( ( TimerListHead == NULL ) || ( obj == NULL ) )
//  {
//    RESTORE_PRIMASK( );
//    return;
//  }

//  obj->IsStarted = false;
//
//  if( TimerListHead == obj ) // Stop the Head
//  {
//    if( TimerListHead->IsNext2Expire == true ) // The head is already running
//    {
//
//      TimerListHead->IsNext2Expire = false;
//      if( TimerListHead->Next != NULL )
//      {
//        TimerListHead = TimerListHead->Next;
//        TimerSetTimeout( TimerListHead );
//      }
//      else
//      {
//        HW_RTC_StopAlarm( );
//        TimerListHead = NULL;
//      }
//    }
//    else // Stop the head before it is started
//    {
//      if( TimerListHead->Next != NULL )
//      {
//        TimerListHead = TimerListHead->Next;
//      }
//      else
//      {
//        TimerListHead = NULL;
//      }
//    }
//  }
//  else // Stop an object within the list
//  {
//    while( cur != NULL )
//    {
//      if( cur == obj )
//      {
//        if( cur->Next != NULL )
//        {
//          cur = cur->Next;
//          prev->Next = cur;
//        }
//        else
//        {
//          cur = NULL;
//          prev->Next = cur;
//        }
//        break;
//      }
//      else
//      {
//        prev = cur;
//        cur = cur->Next;
//      }
//    }
//  }
//
//  RESTORE_PRIMASK( );
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



/*
void TimerIrqHandler( void )
{
  TimerEvent_t* cur;
  TimerEvent_t* next;



  uint32_t old =  HW_RTC_GetTimerContext( );
  uint32_t now =  HW_RTC_SetTimerContext( );
  uint32_t DeltaContext = now - old; //intentionnal wrap around

  // Update timeStamp based upon new Time Reference
  // because delta context should never exceed 2^32
  if ( TimerListHead != NULL )
  {
    for (cur=TimerListHead; cur->Next != NULL; cur= cur->Next)
    {
      next =cur->Next;
      if (next->Timestamp > DeltaContext)
      {
        next->Timestamp -= DeltaContext;
      }
      else
      {
        next->Timestamp = 0 ;
      }
    }
  }

  // execute imediately the alarm callback
  if ( TimerListHead != NULL )
  {
    cur = TimerListHead;
    TimerListHead = TimerListHead->Next;
    cur->IsStarted = false;
    exec_cb( cur->Callback, cur->Context );
  }


  // remove all the expired object from the list
  while( ( TimerListHead != NULL ) && ( TimerListHead->Timestamp < HW_RTC_GetTimerElapsedTime(  )  ))
  {
   cur = TimerListHead;
   TimerListHead = TimerListHead->Next;
   cur->IsStarted = false;
   exec_cb( cur->Callback, cur->Context );
  }

  // start the next TimerListHead if it exists AND NOT running
  if( ( TimerListHead != NULL ) && ( TimerListHead->IsNext2Expire == false ) )
  {
    TimerSetTimeout( TimerListHead );
  }

  */
//}





TimerTime_t TimerGetCurrentTime( void )
{
	return (uint32_t)itsdk_time_get_ms();

  //uint32_t now = HW_RTC_GetTimerValue( );
  //return  HW_RTC_Tick2ms(now);
}

TimerTime_t TimerGetElapsedTime( TimerTime_t past )
{

	return TimerGetCurrentTime() - past;

  //uint32_t nowInTicks = HW_RTC_GetTimerValue( );
  //uint32_t pastInTicks = HW_RTC_ms2Tick( past );
  /* intentional wrap around. Works Ok if tick duation below 1ms */
  //return HW_RTC_Tick2ms( nowInTicks- pastInTicks );
}



TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature )
{
	return period;
   // return RtcTempCompensation( period, temperature );
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
