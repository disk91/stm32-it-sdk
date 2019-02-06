/**
 ******************************************************************************
 * @file    console.h
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
#ifndef __UTIL_CONSOLE_H
#define __UTIL_CONSOLE_H

#include <stdint.h>
#include <it_sdk/config.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/time/time.h>
#include <drivers/lorawan/systime.h>
#include "trace.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ---------------------------------------------------------------------------
// ITSDK customization

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWLORAINF) > 0
#define LOG_INFO_LORAWAN(x)			log_info x
#define LOG_WARN_LORAWAN(x) 		log_warn x
#define LOG_ERROR_LORAWAN(x)		log_error x
#define LOG_DEBUG_LORAWAN(x)		log_debug x
#else
#define LOG_INFO_LORAWAN(x)
#define LOG_WARN_LORAWAN(x)
#define LOG_ERROR_LORAWAN(x)
#define LOG_DEBUG_LORAWAN(x)
#endif

#define PPRINTF(...) 			LOG_INFO_LORAWAN((__VA_ARGS__))
#define PRINTF(...)     		LOG_INFO_LORAWAN((__VA_ARGS__))
#define PRINTNOW()     			LOG_INFO_LORAWAN(("%d: ",(uint32_t)itsdk_time_get_ms()))

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWLORAINF) > 0
	#define TVL1(X)		X
#else
	#define TVL1(X)
#endif

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWLORAINF) > 0
	#define TVL2(X)		X
#else
	#define TVL2(X)
#endif


#ifdef __cplusplus
}
#endif

#endif /*__UTIL_CONSOLE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
