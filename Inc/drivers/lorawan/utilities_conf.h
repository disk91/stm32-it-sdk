/**
  ******************************************************************************
  * @file    utilities_conf.h
  * @author  MCD Application Team
  * @brief   configuration for utilities
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
#ifndef __UTLITIES_CONF_H
#define __UTLITIES_CONF_H

#ifdef __cplusplus
extern "C" {
#endif
#include "vcom.h"
/*low power manager configuration*/
typedef enum
{
  LPM_APPLI_Id =    (1 << 0),
  LPM_LIB_Id =      (1 << 1),
  LPM_RTC_Id =      (1 << 2),
  LPM_GPS_Id =      (1 << 3),
  LPM_UART_RX_Id =  (1 << 4),
  LPM_UART_TX_Id =  (1 << 5),
} LPM_Id_t;

#define OutputInit  vcom_Init
#define OutputTrace vcom_Trace

#define VERBOSE_LEVEL_0 0
#define VERBOSE_LEVEL_1 1
#define VERBOSE_LEVEL_2 2

#define VERBOSE_LEVEL 0

#if ( VERBOSE_LEVEL < VERBOSE_LEVEL_2)
#define DBG_TRACE_MSG_QUEUE_SIZE 256
#else
#define DBG_TRACE_MSG_QUEUE_SIZE 512
#endif

  /* Exported types ------------------------------------------------------------*/
  /* Exported constants --------------------------------------------------------*/
  /* External variables --------------------------------------------------------*/
  /* Exported macros -----------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /*__UTLITIES_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
