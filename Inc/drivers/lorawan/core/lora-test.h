/******************************************************************************
  * @file    lora_test.h
  * @author  MCD Application Team
  * @brief   lora API to drive the lora state Machine
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

#ifndef __LORA_TEST_H__
#define __LORA_TEST_H__

#ifdef __cplusplus
 extern "C" {
#endif
   
 /* Includes ------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
   
#define CERTIF_PORT 224
/* Exported types ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
bool certif_running(void);

void certif_DownLinkIncrement( void );

void certif_linkCheck(MlmeConfirm_t *mlmeConfirm);

void certif_rx( McpsIndication_t *mcpsIndication, MlmeReqJoin_t* JoinParameters);

#ifdef __cplusplus
}
#endif

#endif /*__LORA_TEST_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
