/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board GPIO driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/**
  ******************************************************************************
  * @file    hw_rtc.h
  * @author  MCD Application Team
  * @brief   Header for driver hw_rtc.c module
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

#ifndef __HW_RTC_H__
#define __HW_RTC_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "utilities.h"
   
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

/*!
 * \brief Temperature coefficient of the clock source
 */
#define RTC_TEMP_COEFFICIENT                            ( -0.035 )

/*!
 * \brief Temperature coefficient deviation of the clock source
 */
#define RTC_TEMP_DEV_COEFFICIENT                        ( 0.0035 )

/*!
 * \brief Turnover temperature of the clock source
 */
#define RTC_TEMP_TURNOVER                               ( 25.0 )

/*!
 * \brief Turnover temperature deviation of the clock source
 */
#define RTC_TEMP_DEV_TURNOVER                           ( 5.0 )
/*!
 * @brief Initializes the RTC timer
 * @note The timer is based on the RTC
 * @param none
 * @retval none
 */
void HW_RTC_Init( void );

/*!
 * @brief Stop the Alarm
 * @param none
 * @retval none
 */
void HW_RTC_StopAlarm( void );

/*!
 * @brief Return the minimum timeout the RTC is able to handle
 * @param none
 * @retval minimum value for a timeout
 */
uint32_t HW_RTC_GetMinimumTimeout( void );

/*!
 * @brief Set the alarm
 * @note The alarm is set at Reference + timeout
 * @param timeout Duration of the Timer in ticks
 */
void HW_RTC_SetAlarm( uint32_t timeout );

/*!
 * @brief Get the RTC timer elapsed time since the last Reference was set
 * @retval RTC Elapsed time in ticks
 */
uint32_t HW_RTC_GetTimerElapsedTime( void );

/*!
 * @brief Get the RTC timer value
 * @retval none
 */
uint32_t HW_RTC_GetTimerValue( void );

/*!
 * @brief Set the RTC timer Reference
 * @retval  Timer Reference Value in  Ticks
 */
uint32_t HW_RTC_SetTimerContext( void );
  
/*!
 * @brief Get the RTC timer Reference
 * @retval Timer Value in  Ticks
 */
uint32_t HW_RTC_GetTimerContext( void );
/*!
 * @brief RTC IRQ Handler on the RTC Alarm
 * @param none
 * @retval none
 */
void HW_RTC_IrqHandler ( void );

/*!
 * @brief a delay of delay ms by polling RTC
 * @param delay in ms
 * @param none
 * @retval none
 */
void HW_RTC_DelayMs( uint32_t delay );

/*!
 * @brief calculates the wake up time between wake up and mcu start
 * @note resolution in RTC_ALARM_TIME_BASE
 * @param none
 * @retval none
 */
void HW_RTC_setMcuWakeUpTime( void );

/*!
 * @brief returns the wake up time in us
 * @param none
 * @retval wake up time in ticks
 */
int16_t HW_RTC_getMcuWakeUpTime( void );

/*!
 * @brief converts time in ms to time in ticks
 * @param [IN] time in milliseconds
 * @retval returns time in timer ticks
 */
uint32_t HW_RTC_ms2Tick( TimerTime_t timeMilliSec );

/*!
 * @brief converts time in ticks to time in ms
 * @param [IN] time in timer ticks
 * @retval returns time in timer milliseconds
 */
TimerTime_t HW_RTC_Tick2ms( uint32_t tick );

/*!
 * \brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * \param [IN] period Time period to compensate
 * \param [IN] temperature Current temperature
 *
 * \retval Compensated time period
 */
TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature );

/*!
 * \brief Get system time
 * \param [IN]   subSeconds in ms
 *               
 * \uint32_t     seconds 
 */
uint32_t HW_RTC_GetCalendarTime( uint16_t *subSeconds );

/*!
 * \brief Read from backup registers
 * \param [IN]  Data 0
 * \param [IN]  Data 1
 *               
 */
void HW_RTC_BKUPRead( uint32_t *Data0, uint32_t *Data1);

/*!
 * \brief Write in backup registers
 * \param [IN]  Data 0
 * \param [IN]  Data 1
 *               
 */

void HW_RTC_BKUPWrite( uint32_t Data0, uint32_t Data1);

#ifdef __cplusplus
}
#endif

#endif /* __HW_RTC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
