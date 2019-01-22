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


#define PPRINTF(...) 			log_info(__VA_ARGS__)
#define PRINTF(...)     		log_info(__VA_ARGS__)
#define PRINTNOW()     			do { 													\
									log_info("%d: ",(uint32_t)itsdk_time_get_ms());		\
								} while(0);

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWLORADBG) > 0
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
