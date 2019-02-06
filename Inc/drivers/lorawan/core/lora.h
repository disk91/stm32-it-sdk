 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    lora.h
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

#ifndef __LORA_MAIN_H__
#define __LORA_MAIN_H__

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/

// Commissioning contains the different KEYs & IDs and the Connection procedure
// managed differently here
//#include "Commissioning.h"

#include <drivers/lorawan/mac/LoRaMac.h>
#include <drivers/lorawan/mac/region/Region.h>

/* Exported constants --------------------------------------------------------*/
   /*!
 * LoRaWAN confirmed messages
 */

#define LORAWAN_ADR_ON                              1
#define LORAWAN_ADR_OFF                             0
/* Exported types ------------------------------------------------------------*/

/*!
 * Application Data structure
 */
typedef struct
{
  /*point to the LoRa App data buffer*/
  uint8_t* Buff;
  /*LoRa App data buffer size*/
  uint8_t BuffSize;
  /*Port on which the LoRa App is data is sent/ received*/
  uint8_t Port;
  
} lora_AppData_t;



typedef enum 
{
  LORA_RESET = 0, 
  LORA_SET = !LORA_RESET
} LoraFlagStatus;

typedef enum 
{
  LORA_ERROR = -1, 
  LORA_SUCCESS = 0
} LoraErrorStatus;

typedef enum 
{
  LORAWAN_UNCONFIRMED_MSG = 0, 
  LORAWAN_CONFIRMED_MSG = !LORAWAN_UNCONFIRMED_MSG
} LoraConfirm_t;

typedef enum 
{
  LORA_TRUE = 0, 
  LORA_FALSE = !LORA_TRUE
} LoraBool_t;

/*!
 * LoRa State Machine states 
 */
typedef enum eTxEventType
{
/*!
 * @brief AppdataTransmition issue based on timer every TxDutyCycleTime
 */
    TX_ON_TIMER,
/*!
 * @brief AppdataTransmition external event plugged on OnSendEvent( )
 */
    TX_ON_EVENT
} TxEventType_t;


/*!
 * LoRa State Machine states 
 */
typedef struct sLoRaParam
{
/*!
 * @brief Activation state of adaptativeDatarate
 */
    bool AdrEnable;
/*!
 * @brief Uplink datarate, if AdrEnable is off
 */
    int8_t TxDatarate;
/*!
 * @brief Enable or disable a public network
 *
 */
    bool EnablePublicNetwork;

   // Disk91 - modification
   // Extends with the configuration...

   // __LORAWAN_OTAA or __LORAWAN_ABP
   uint8_t   JoinType;
   uint8_t * devEui;						// Dev EUI pointer (8B)
   union {
	   struct _s_otaa {
		   uint8_t * appEui;				// OTAA App EUI pointer (8B)
		   uint8_t * appKey;		   		// OTAA App Key pointer (16B)
		   uint8_t * nwkKey;		   		// OTAA Nwk Key pointer (16B)
	   } otaa;
	   struct _s_abp {
		   uint8_t * FNwkSIntKey;			// Nwk Internal Session Key
		   uint8_t * SNwkSIntKey;			//
		   uint8_t * nwkSEncKey;			// Nwk Session Key
		   uint8_t * appSKey;				// App Session Key
		   uint32_t  devAddr;				// Dev addr
	   } abp;
   } config;

} LoRaParam_t;

/* Lora Main callbacks*/
typedef struct sLoRaMainCallback
{
/*!
 * @brief Get the current battery level
 *
 * @retval value  battery level ( 0: very low, 254: fully charged )
 */
    uint8_t ( *BoardGetBatteryLevel )( void );
/*!
 * \brief Get the current temperature
 *
 * \retval value  temperature in degreeCelcius( q7.8 )
 */
  uint16_t ( *BoardGetTemperatureLevel)( void );
/*!
 * @brief Gets the board 64 bits unique ID 
 *
 * @param [IN] id Pointer to an array that will contain the Unique ID
 */
    void    ( *BoardGetUniqueId ) ( uint8_t *id);
  /*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * @retval seed Generated pseudo random seed
 */
    uint32_t ( *BoardGetRandomSeed ) (void);
/*!
 * @brief Process Rx Data received from Lora network 
 *
 * @param [IN] AppData structure
 *
 */
    void ( *LORA_RxData ) ( lora_AppData_t *AppData);
    
/*!
 * @brief callback indicating EndNode has jsu joiny 
 *
 * @param [IN] None
 */
    void ( *LORA_HasJoined)( void );
/*!
 * @brief Confirms the class change 
 *
 * @param [IN] AppData is a buffer to process
 *
 * @param [IN] port is a Application port on wicth Appdata will be sent
 *
 * @param [IN] length is the number of recieved bytes
 */
    void ( *LORA_ConfirmClass) ( DeviceClass_t Class );
/*!
 * @brief callback indicating an uplink transmission is needed to allow
 *        a pending downlink transmission 
 *
 * @param [IN] None
 */
    void ( *LORA_TxNeeded) ( void);
/*!
 *\brief    Will be called each time a Radio IRQ is handled by the MAC
 *          layer.
 * 
 *\warning  Runs in a IRQ context. Should only change variables state.  
 */
    void ( *MacProcessNotify )( void );

    void ( *LORA_McpsDataConfirm )(void);

} LoRaMainCallback_t;



/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief Lora Initialisation
 * @param [IN] LoRaMainCallback_t
 * @param [IN] application parmaters
 * @param [IN] Region to connect to (see __LORAWAN_REGION_XXYYY )
 * @retval none
 */
void LORA_Init (LoRaMainCallback_t *callbacks, LoRaParam_t* LoRaParam, uint16_t region  );

/**
 * @brief run Lora classA state Machine 
 * @param [IN] none
 * @retval none
 */
bool LORA_send(lora_AppData_t* AppData, LoraConfirm_t IsTxConfirmed);

/**
 * @brief Join a Lora Network in classA
 * @Note if the device is ABP, this is a pass through functon
 * @param [IN] none
 * @retval none
 */
void LORA_Join( void);

/**
 * @brief Check whether the Device is joined to the network
 * @param [IN] none
 * @retval returns LORA_SET if joined
 */
LoraFlagStatus LORA_JoinStatus( void);

/**
 * @brief change Lora Class
 * @Note callback LORA_ConfirmClass informs upper layer that the change has occured
 * @Note Only switch from class A to class B/C OR from  class B/C to class A is allowed
 * @Attention can be calld only in LORA_ClassSwitchSlot or LORA_RxData callbacks
 * @param [IN] DeviceClass_t NewClass
 * @retval LoraErrorStatus
 */
LoraErrorStatus LORA_RequestClass( DeviceClass_t newClass );

/**
 * @brief get the current Lora Class
 * @param [IN] DeviceClass_t NewClass
 * @retval None
 */
void LORA_GetCurrentClass( DeviceClass_t *currentClass );
#ifdef __cplusplus
}
#endif

#endif /*__LORA_MAIN_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
