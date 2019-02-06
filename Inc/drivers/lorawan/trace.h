/**
  ******************************************************************************
  * @file    trace.h
  * @author  MCD Application Team
  * @brief   Header for dbg_trace.c
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DBG_TRACE_H
#define __DBG_TRACE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "utilities_conf.h"

/* Exported types ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief TraceInit Initializes Logging feature.
 * @brief Initializes the queue and the hardware
 * @param:  None
 * @retval: None
 */
void TraceInit( void );

/**
 * @brief TraceSend decode the strFormat and post it to the circular queue for printing
 *
 * @param:  None
 * @retval: 0 when ok, -1 when circular queue is full
 */
int32_t TraceSend( const char *strFormat, ...);

/**
 * @brief  TraceGetFileName: Return filename string extracted from full path information
 * @param  *fullPath Fullpath string (path + filename)
 * @retval char* Pointer on filename string
 */
const char *TraceGetFileName( const char *fullpath );

#ifdef __cplusplus
}
#endif

#endif /*__DBG_TRACE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
