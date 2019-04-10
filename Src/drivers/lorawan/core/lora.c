/******************************************************************************
  * @file    lora.c
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

#include <it_sdk/itsdk.h>
#if ITSDK_WITH_LORAWAN_LIB == __ENABLE
#include <it_sdk/logger/logger.h>

/* Includes ------------------------------------------------------------------*/
#include <drivers/sx1276/hw.h>
#include <drivers/lorawan/timeServer.h>
#include <drivers/lorawan/mac/LoRaMac.h>
#include <drivers/lorawan/core/lora.h>
#include <drivers/lorawan/core/lora-test.h>
#include <drivers/lorawan/compiled_region.h>


/*!
 *  Select either Device_Time_req or Beacon_Time_Req following LoRaWAN version 
 *  - Device_Time_req   Available for V1.0.3 or later                          
 *  - Beacon_time_Req   Available for V1.0.2 and before                        
 */
#define USE_DEVICE_TIMING

/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000  // 10 [s] value in ms

#if defined( REGION_EU868 )

#include <drivers/lorawan/mac/LoRaMacTest.h>

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true
#endif
/*!
 * Default ping slots periodicity
 *
 * \remark periodicity is equal to 2^LORAWAN_DEFAULT_PING_SLOT_PERIODICITY seconds
 *         example: 2^3 = 8 seconds. The end-device will open an Rx slot every 8 seconds.
 */
#define LORAWAN_DEFAULT_PING_SLOT_PERIODICITY       0   

#define HEX16(X)  X[0],X[1], X[2],X[3], X[4],X[5], X[6],X[7],X[8],X[9], X[10],X[11], X[12],X[13], X[14],X[15]
#define HEX8(X)   X[0],X[1], X[2],X[3], X[4],X[5], X[6],X[7]

#ifdef LORAMAC_CLASSB_ENABLED
static LoraErrorStatus LORA_BeaconReq( void);
static LoraErrorStatus LORA_PingSlotReq( void);

#if defined( USE_DEVICE_TIMING )
static LoraErrorStatus LORA_DeviceTimeReq(void);
#else
static LoraErrorStatus LORA_BeaconTimeReq(void);
#endif /* USE_DEVICE_TIMING */
#endif /* LORAMAC_CLASSB_ENABLED */

 void TraceUpLinkFrame(McpsConfirm_t *mcpsConfirm);
 void TraceDownLinkFrame(McpsIndication_t *mcpsIndication);
#ifdef LORAMAC_CLASSB_ENABLED
static void TraceBeaconInfo(MlmeIndication_t *mlmeIndication);
#endif /* LORAMAC_CLASSB_ENABLED */





/**
 * @TODO bug la dedans il y a une ref aAppData qui n'est pas global !!!
 */
void TraceUpLinkFrame(McpsConfirm_t *mcpsConfirm)
{

    MibRequestConfirm_t mibGet;
    MibRequestConfirm_t mibReq;

    mibReq.Type = MIB_DEVICE_CLASS;
    LoRaMacMibGetRequestConfirm( &mibReq );
  
    TVL2( PRINTF("\r\n" );)
    TVL2( PRINTNOW(); PRINTF("#= U/L FRAME %lu =# Class %c, Port %d, data size %d, pwr %d, ", \
                             mcpsConfirm->UpLinkCounter, \
                             "ABC"[mibReq.Param.Class], \
                             0, /*AppData.Port,*/ \
                             0, /*AppData.BuffSize,*/ \
                             mcpsConfirm->TxPower );)

    mibGet.Type  = MIB_CHANNELS_MASK;
    if( LoRaMacMibGetRequestConfirm( &mibGet ) == LORAMAC_STATUS_OK )
    {
        TVL2( PRINTF( "Channel Mask ");)
#if defined( REGION_AS923 ) || defined( REGION_CN779 ) || \
    defined( REGION_EU868 ) || defined( REGION_IN865 ) || \
    defined( REGION_KR920 ) || defined( REGION_EU433 ) || \
    defined( REGION_RU864 )

        for( uint8_t i = 0; i < 1; i++)

#elif defined( REGION_AU915 ) || defined( REGION_US915 ) || defined( REGION_CN470 )

        for( uint8_t i = 0; i < 5; i++)
#else

#error "Please define a region in the compiler options."

#endif
        {
            TVL2( PRINTF( "%04X ", mibGet.Param.ChannelsMask[i] );)
        }
    }

    TVL2( PRINTF("\r\n\r\n" );)
} 


void TraceDownLinkFrame(McpsIndication_t *mcpsIndication)
{
 //   const char *slotStrings[] = { "1", "2", "C", "Ping-Slot", "Multicast Ping-Slot" };
 /*
    TVL2( PRINTF("\r\n" );)
    TVL2( PRINTNOW(); PRINTF("#= D/L FRAME %lu =# RxWin %s, Port %d, data size %d, rssi %d, snr %d\r\n\r\n", \
                             mcpsIndication->DownLinkCounter, \
                             slotStrings[mcpsIndication->RxSlot], \
                             mcpsIndication->Port, \
                             mcpsIndication->BufferSize, \
                             mcpsIndication->Rssi, \
                             mcpsIndication->Snr );)
                             */
}  

#ifdef LORAMAC_CLASSB_ENABLED
static void TraceBeaconInfo(MlmeIndication_t *mlmeIndication)
{
    int32_t snr = 0;
    if( mlmeIndication->BeaconInfo.Snr & 0x80 ) // The SNR sign bit is 1
    {
        // Invert and divide by 4
        snr = ( ( ~mlmeIndication->BeaconInfo.Snr + 1 ) & 0xFF ) >> 2;
        snr = -snr;
    }
    else
    {
        // Divide by 4
        snr = ( mlmeIndication->BeaconInfo.Snr & 0xFF ) >> 2;
    }  
    
    TVL2( PRINTF("\r\n" );)
    TVL2( PRINTNOW(); PRINTF("#= BEACON %lu =#, GW desc %d, rssi %d, snr %ld\r\n\r\n", \
                             mlmeIndication->BeaconInfo.Time, \
                             mlmeIndication->BeaconInfo.GwSpecific.InfoDesc, \
                             mlmeIndication->BeaconInfo.Rssi, \
                             snr );)
}
#endif /* LORAMAC_CLASSB_ENABLED */

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

