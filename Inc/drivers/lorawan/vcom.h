/**
  ******************************************************************************
  * @file    vcom.h
  * @author  MCD Application Team
  * @brief   Header for vcom.c module
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
#ifndef __VCOM_H__
#define __VCOM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** 
* @brief  init vcom 
* @param  callback when Tx buffer has been sent
* @return None
*/
void vcom_Init(  void (*Txcb)(void) ); 
  
/** 
* @brief  send buffer @p_data of size size to vcom in dma mode
* @param  p_data data to be sent
* @param  szie of buffer p_data to be sent
* @return None
*/
void vcom_Trace(  uint8_t *p_data, uint16_t size );

   /** 
* @brief  DeInit the VCOM.
* @param  None
* @return None
*/
void vcom_DeInit(void);

   /** 
* @brief  Init the VCOM IOs.
* @param  None
* @return None
*/
void vcom_IoInit(void);
  
/** 
* @brief  DeInit the VCOM IOs.
* @param  None
* @return None
*/
void vcom_IoDeInit(void);

/** 
* @brief  last byte has been sent on the uart line
* @param  None
* @return None
*/
void vcom_IRQHandler(void);

/** 
* @brief  last byte has been sent from memeory to uart data register
* @param  None
* @return None
*/
void vcom_DMA_TX_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __VCOM_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
