/* ==========================================================
 * lorawan.c - Driver for making semtech LoRaWan stack
 *             working in a synchronous mode for interfacing
 *             with itsdk abstraction layer
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 03 fev. 2019
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2019 Disk91
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU LESSER General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * ----------------------------------------------------------
 *
 *
 * ==========================================================
 */

#include <stdbool.h>
#include <string.h>

#include <it_sdk/config.h>
#if ITSDK_WITH_LORAWAN_LIB == __ENABLE && ITSDK_LORAWAN_STACK == __LORAWAN_SEMTECH

#include <it_sdk/itsdk.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/time/timer.h>
#include <drivers/lorawan/core/lorawan.h>
#include <drivers/lorawan/mac/LoRaMac.h>
#include <drivers/lorawan/core/lora-test.h>
#include <drivers/lorawan/compiled_region.h>


/*
#include <drivers/lorawan/core/lora.h>
#include <drivers/lorawan/phy/radio.h>
#include <it_sdk/lorawan/lorawan.h>

#include <drivers/sx1276/hw.h>
#include <drivers/lorawan/timeServer.h>
#include <drivers/lorawan/core/lora.h>
*/

/**
 * API with LoRaMAC stack
 * See details on : https://stackforce.github.io/LoRaMac-doc/
 *
 * This structure map the lorawan stack callbacks
 * MCSP => (Mac Common Part Sub Layer) => TX & RX API
 * MLME => (Mac Layer Management Entity) => Management
 * MIB => (Max Information Base) => store information
 *
 * A Request is associated with a Confirmation
 * A response is associated with an Indication
 */
static LoRaMacPrimitives_t LoRaMacPrimitives;


/**
 * API with LoRaMAC stack
 * Different function related to the hardware implementation
 * Like
 * - battery
 * - temperature
 * - nvm callback ( identify a change in an attribute subject to be stored in the nvm)
 * - MacProcessNotify : irq processing callback
 */
static LoRaMacCallback_t   LoRaMacCallbacks;

/**
 * Internal lorawan driver state
 */
static lorawan_driver_state_t __loraWanState;

/**
 * Internal API
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm );
static void McpsIndication( McpsIndication_t *mcpsIndication );
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm );
static void MlmeIndication( MlmeIndication_t *MlmeIndication );

// ======================================================================================
// Synchronous loop -> this function is called until we terminate the operation
//                     this function can be override to switch into async mode
//					   in this case it just return and you need to manage it outside
// ======================================================================================



/**
 * lorawan loop : process the LoRaMac
 * This need to be called as much as possible.
 * in Sync mode the function is call by the waitUntilEndOfExecution
 * when switch in async mode you need to call this function as much as possible
 */
void lorawan_driver_loop() {

	while ( __loraWanState.reqPending ) {
		__loraWanState.reqPending=false;
        LoRaMacProcess( );
	}

}

__weak void lorawan_driver_waitUntilEndOfExecution() {

	lorawan_driver_loop();
    wdg_refresh();
    itsdk_stimer_run();

}

// ======================================================================================
// Callback for this layer
// ======================================================================================

/**
 * Callback function on Data Reception
 * this function is based on standard element and can be overrided.
 */
__weak void lorawan_driver_onDataReception(uint8_t port, uint8_t * data, uint8_t size) {
   log_info("[LoRaWAN] data received %d bytes\r\n",size);
}


/**
 * Callback function on data reception
 * this function is based on a lorawan stack internal structure
 */
/*
static void __onReceiveData(lora_AppData_t *AppData)
{
	itsdk_lorawan_onDataReception(
			AppData->Port,
			AppData->Buff,
			AppData->BuffSize
	);
}
*/

/**
 * Callback function on JOIN Success
 */
__weak void lorawan_driver_onJoinSuccess() {
   log_info("[LoRaWAN] Join success\r\n");
}



// =======================================================================================
// Callback used by the LoRaMac
// =======================================================================================


/**
 * Return a batteryLevel from 1 to 254
 * 1 = VBAT_MIN
 * 254 = VBAT_MAX
 */
__weak uint8_t lorawan_driver_battery_level() {
	 uint16_t mv = adc_getVBat();
	 if ( mv <= ITSDK_VBAT_MIN ) return 1;
	 if ( mv >= ITSDK_VBAT_MAX ) return 254;
	 return (( (uint32_t) (mv - ITSDK_VBAT_MIN)*ITSDK_VBAT_MAX) /(ITSDK_VBAT_MAX-ITSDK_VBAT_MIN) );
}

/**
 * Return the temperature
 * temperature in fixed decimal : 8b integer + 8b decimal
 */
__weak uint16_t lorawan_driver_temperature() {
	int16_t t = adc_getTemperature();
	t = (int16_t)(((int32_t)t << 8)/100);
	return (uint16_t)t;
}

/**
 * Called after IRQ processing
 */
void lorawan_driver_macProcessNotify(void) {
  log_info("[LoRaWAN] Mac Process Notify\r\n");
  __loraWanState.reqPending=true;
}

/**
 * Called after attribute change for NVM storage
 */
void lorawan_driver_nvmContextChange(LoRaMacNvmCtxModule_t module) {
  log_info("[LoRaWAN] Nvm Change\r\n");
}

/**
 * Callback function requesting transmission
 */
__weak void lorawan_driver_onTxNeeded() {
   log_info("[LoRaWAN] Network Server is asking for an uplink transmission\r\n");
}


/**
 * Convert Datarate define from ITSDK to LoRaMac
 */
static uint8_t __convertDR(uint8_t itsdkDr) {
	switch (itsdkDr) {
	case __LORAWAN_DR_0: return DR_0;
	case __LORAWAN_DR_1: return DR_1;
	case __LORAWAN_DR_2: return DR_2;
	case __LORAWAN_DR_3: return DR_3;
	case __LORAWAN_DR_4: return DR_4;
	case __LORAWAN_DR_5: return DR_5;
	case __LORAWAN_DR_6: return DR_6;
	case __LORAWAN_DR_7: return DR_7;
	case __LORAWAN_DR_8: return DR_8;
	case __LORAWAN_DR_9: return DR_9;
	case __LORAWAN_DR_10: return DR_10;
	case __LORAWAN_DR_11: return DR_11;
	case __LORAWAN_DR_12: return DR_12;
	case __LORAWAN_DR_13: return DR_13;
	case __LORAWAN_DR_14: return DR_14;
	case __LORAWAN_DR_15: return DR_15;
	default:
		log_error("[LoRaWan] Invalid DR configuration %d \r\n",itsdkDr);
		itsdk_error_handler(__FILE__,__LINE__);
	}
	return 0;	// never reached
}

// =======================================================================================
// Some string for trace
// =======================================================================================

/*!
 * MAC event info status strings.
 */
const char* EventInfoStatusStrings[] =
{
    "OK", "Error", "Tx timeout", "Rx 1 timeout",
    "Rx 2 timeout", "Rx1 error", "Rx2 error",
    "Join failed", "Downlink repeated", "Tx DR payload size error",
    "Downlink too many frames loss", "Address fail", "MIC fail",
    "Multicast faile", "Beacon locked", "Beacon lost", "Beacon not found"
};

/*!
 * MAC status strings
 */
const char* MacStatusStrings[] =
{
    "OK", "Busy", "Service unknown", "Parameter invalid", "Frequency invalid",
    "Datarate invalid", "Freuqency or datarate invalid", "No network joined",
    "Length error", "Device OFF", "Region not supported", "Skipped APP data",
    "DutyC restricted", "No channel found", "No free channel found",
    "Busy beacon reserved time", "Busy ping-slot window time",
    "Busy uplink collision", "Crypto error", "FCnt handler error",
    "MAC command error", "ERROR"
};

const char* MlmeReqStrings[] =
{
    "MLME_JOIN",
    "MLME_REJOIN_0",
    "MLME_REJOIN_1",
    "MLME_LINK_CHECK",
    "MLME_TXCW",
    "MLME_TXCW_1",
    "MLME_SCHEDULE_UPLINK",
    "MLME_NVM_CTXS_UPDATE",
    "MLME_DERIVE_MC_KE_KEY",
    "MLME_DERIVE_MC_KEY_PAIR",
    "MLME_DEVICE_TIME",
    "MLME_BEACON",
    "MLME_BEACON_ACQUISITION",
    "MLME_PING_SLOT_INFO",
    "MLME_BEACON_TIMING,MLME_BEACON_LOST"
};


// =======================================================================================
// Init the LoRaMac
// =======================================================================================

/**
 *  lora Init - initialize the stack and the associated hardware
 *
 */
void lorawan_driver_LORA_Init(
		lorawan_driver_config_t * config
){
  __loraWanState.stackState = LORAWAN_STATE_NONE;
  LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
  LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
  LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
  LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
  LoRaMacCallbacks.GetBatteryLevel = lorawan_driver_battery_level;
  LoRaMacCallbacks.GetTemperatureLevel = lorawan_driver_temperature;
  LoRaMacCallbacks.MacProcessNotify = lorawan_driver_macProcessNotify;

  // Set the Radio configuration
  switch ( config->region ) {
		#if defined( REGION_AS923 )
        case __LORAWAN_REGION_AS923:
        	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AS923 );
        	break;
		#endif
		#if defined( REGION_AU915 )
        case __LORAWAN_REGION_AU915:
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AU915 );
            break;
		#endif
		#if defined( REGION_CN470 )
        case __LORAWAN_REGION_CN470:
        	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470 );
        	break;
		#endif
		#if defined( REGION_CN779 )
        case __LORAWAN_REGION_CN779:
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN779 );
            break;
		#endif
		#if defined( REGION_EU433 )
        case __LORAWAN_REGION_EU433:
        	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU433 );
        	break;
		#endif
		#if defined( REGION_EU868 )
        case __LORAWAN_REGION_EU868:
        	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868 );
        	LoRaMacTestSetDutyCycleOn( true );	// activate duty cycle
        	break;
		#endif
		#if defined( REGION_KR920 )
        case __LORAWAN_REGION_KR920:
        	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KR920 );
        	break;
		#endif
		#if defined( REGION_IN865 )
        case __LORAWAN_REGION_IN865:
        	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_IN865 );
        	break;
		#endif
		#if defined( REGION_US915 )
        case __LORAWAN_REGION_US915:
        	break;
		#endif
		#if defined( REGION_RU864 )
        case __LORAWAN_REGION_RU864:
        	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_RU864 );
        	break;
		#endif
        default:
        	log_error("[LoRaWan] Invalid region selected\r\n");
    		itsdk_error_handler(__FILE__,__LINE__);
        	return;
        }

		#if ITSDK_LORAWAN_REGION_ALLOWED == __LORAWAN_REGION_NONE
			#error "Please define a region in the compiler options."
		#endif


  	    // Set the default configuration to the MIB
 	  	MibRequestConfirm_t mibReq;

  	    mibReq.Type = MIB_ADR;
  	    mibReq.Param.AdrEnable = config->adrEnable;
  	    LoRaMacMibSetRequestConfirm( &mibReq );

  	    mibReq.Type = MIB_PUBLIC_NETWORK;
  	    mibReq.Param.EnablePublicNetwork = config->enablePublicNetwork;
  	    LoRaMacMibSetRequestConfirm( &mibReq );

  	    if ( config->JoinType == __LORAWAN_OTAA ) {
  	    	mibReq.Type = MIB_APP_KEY;
  	    	mibReq.Param.AppKey = config->config.otaa.appKey;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );

  	    	mibReq.Type = MIB_NWK_KEY;
  	    	mibReq.Param.NwkKey = config->config.otaa.nwkKey;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );
  	    } else if (config->JoinType == __LORAWAN_ABP) {

  	    	if (config->config.abp.devAddr == 0) {
				// Choose a random device address
				srand1( itsdk_getRandomSeed( ) );
				config->config.abp.devAddr = randr( 0, 0x01FFFFFF );
  	    	}

  	    	mibReq.Type = MIB_DEV_ADDR;
  	    	mibReq.Param.DevAddr = config->config.abp.devAddr;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );

  	    	mibReq.Type = MIB_NET_ID;
  	    	mibReq.Param.NetID = ITSDK_LORAWAN_NETWORKID;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );

  	    	mibReq.Type = MIB_F_NWK_S_INT_KEY;
  	    	mibReq.Param.FNwkSIntKey = config->config.abp.FNwkSIntKey;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );

  	    	mibReq.Type = MIB_S_NWK_S_INT_KEY;
  	    	mibReq.Param.SNwkSIntKey = config->config.abp.SNwkSIntKey;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );

  	    	mibReq.Type = MIB_NWK_S_ENC_KEY;
  	    	mibReq.Param.NwkSEncKey = config->config.abp.nwkSEncKey;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );

  	    	mibReq.Type = MIB_APP_S_KEY;
  	    	mibReq.Param.AppSKey = config->config.abp.appSKey;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );

  	    	mibReq.Type = MIB_NETWORK_ACTIVATION;
  	    	mibReq.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );

  	    }
  	    mibReq.Type = MIB_DEVICE_CLASS;
  	    mibReq.Param.Class= CLASS_A;
  	    LoRaMacMibSetRequestConfirm( &mibReq );

	    mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
		mibReq.Param.SystemMaxRxError = 20;
		LoRaMacMibSetRequestConfirm( &mibReq );


		// this part of the code is not yes clear...
		#if defined( HYBRID )
                uint16_t channelMask[] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0001, 0x0000};
                mibReq.Type = MIB_CHANNELS_MASK;
                mibReq.Param.ChannelsMask = channelMask;
                LoRaMacMibSetRequestConfirm( &mibReq );
                mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
                mibReq.Param.ChannelsDefaultMask = channelMask;
                LoRaMacMibSetRequestConfirm( &mibReq );
		#endif

         // Init the Mac layer
         LoRaMacStart();
         __loraWanState.stackState = LORAWAN_STATE_INITIALIZED;

}


// =======================================================================================
// Join
// =======================================================================================


lorawan_driver_join_status lorawan_driver_LORA_Join(
		lorawan_driver_config_t * config,
		lorawan_driver_run_t 	  runMode
){
    MlmeReq_t mlmeReq;

    mlmeReq.Type = MLME_JOIN;
    mlmeReq.Req.Join.DevEui = config->devEui;
    mlmeReq.Req.Join.JoinEui = config->config.otaa.appEui;
    mlmeReq.Req.Join.Datarate = __convertDR(config->txDatarate);

    //MlmeReqJoin_t JoinParameters = mlmeReq.Req.Join;

    switch (config->JoinType) {
    case __LORAWAN_OTAA:
    	{
			LoRaMacStatus_t r = LoRaMacMlmeRequest( &mlmeReq );
			if ( r != LORAMAC_STATUS_OK ) {
				log_warn("LoRaMacMlmeRequest return error(%d)\r\n",r);
				__loraWanState.stackState = LORAWAN_JOIN_FAILED;
			} else {
				__loraWanState.stackState = LORAWAN_STATE_JOINING;
			}
    	}
        break;
    case __LORAWAN_ABP:
    	{
			// Enable legacy mode to operate according to LoRaWAN Spec. 1.0.3
			Version_t abpLrWanVersion;
			MibRequestConfirm_t mibReq;

			abpLrWanVersion.Fields.Major    = 1;
			abpLrWanVersion.Fields.Minor    = 0;
			abpLrWanVersion.Fields.Revision = 3;
			abpLrWanVersion.Fields.Rfu      = 0;

			mibReq.Type = MIB_ABP_LORAWAN_VERSION;
			mibReq.Param.AbpLrWanVersion = abpLrWanVersion;
			LoRaMacMibSetRequestConfirm( &mibReq );
			__loraWanState.stackState = LORAWAN_STATE_JOIN_SUCCESS;
			__loraWanState.joinTime = (uint32_t)(itsdk_time_get_ms()/1000);


			lorawan_driver_onJoinSuccess();
    	}
        break;
    default:
    	log_error("Invalid Join method\r\n");
    	itsdk_error_handler(__FILE__,__LINE__);
    }

    if (runMode==LORAWAN_RUN_SYNC) {
        // Go for synchronous
    	while(__loraWanState.stackState == LORAWAN_STATE_JOINING) {
    		lorawan_driver_waitUntilEndOfExecution();
    	}
    	if ( __loraWanState.stackState == LORAWAN_STATE_JOIN_SUCCESS ) {
    		return LORAWAN_JOIN_SUCCESS;
    	} else {
    		return LORAWAN_JOIN_FAILED;
    	}
    } else {
    	return LORAWAN_JOIN_PENDING;
    }

}


// =============================================================================================
// MCPS ( TX & RX Operations ) LAYER
// =============================================================================================


static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{

    TVL2( PRINTNOW(); PRINTF("APP> McpsConfirm STATUS: %s\r\n", EventInfoStatusStrings[mcpsConfirm->Status] ); )

    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
            	if(mcpsConfirm->AckReceived){
            		log_info("### Ack confirmed\r\n");
            	} else {
            		log_info("### Ack not confirmed\r\n");
            	}
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }

    //implicitely desactivated when VERBOSE_LEVEL < 2
    TraceUpLinkFrame(mcpsConfirm);

}

static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    TVL2( PRINTNOW(); PRINTF("APP> McpsInd STATUS: %s\r\n", EventInfoStatusStrings[mcpsIndication->Status] );)

    //lora_AppData_t _AppData;
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
        	log_info("??? unconfirmed\r\n");
            break;
        }
        case MCPS_CONFIRMED:
        {
        	log_info("??? confirmed\r\n");
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
    	log_info("[LoRaWAN] Network Server is asking for an uplink transmission\r\n");

    }
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    if (certif_running() == true )
    {
      certif_DownLinkIncrement( );
    }

    if( mcpsIndication->RxData == true )
    {
      switch( mcpsIndication->Port )
      {
        case CERTIF_PORT:
          // revoir cette partie... pas top de garder des param comme ca en rab
  		  #warning "See how to setup this part back"
          //certif_rx( mcpsIndication, &JoinParameters );
          break;
        default:


//          _AppData.Port = mcpsIndication->Port;
//          _AppData.BuffSize = mcpsIndication->BufferSize;
//          _AppData.Buff = mcpsIndication->Buffer;

          log_info("### Data received\r\n");
          lorawan_driver_onDataReception(
        		  mcpsIndication->Port,
				  mcpsIndication->Buffer,
				  mcpsIndication->BufferSize
          );
          break;
      }
    }

    /*implicitely desactivated when VERBOSE_LEVEL < 2*/
    TraceDownLinkFrame(mcpsIndication);
}

// =============================================================================================
// MLME ( Management ) LAYER
// =============================================================================================

static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
#ifdef LORAMAC_CLASSB_ENABLED
    MibRequestConfirm_t mibReq;
#endif /* LORAMAC_CLASSB_ENABLED */

    TVL2( PRINTNOW(); PRINTF("APP> MlmeConfirm STATUS: %s\r\n", EventInfoStatusStrings[mlmeConfirm->Status] );)

    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
              // Status is OK, node has joined the network
              __loraWanState.stackState = LORAWAN_STATE_JOIN_SUCCESS;
              __loraWanState.joinTime = (uint32_t)(itsdk_time_get_ms()/1000);
              lorawan_driver_onJoinSuccess();
              log_info("### The device has joined\r\n");

#ifdef LORAMAC_CLASSB_ENABLED
#if defined( USE_DEVICE_TIMING )
              LORA_DeviceTimeReq();
#else
              LORA_BeaconTimeReq();
#endif /* USE_DEVICE_TIMING */
#endif /* LORAMAC_CLASSB_ENABLED */
            }
            else
            {
                // Join was not successful. Try to join again
            	log_info("### Join failed, try again. (we should remove this)\r\n");
            	__loraWanState.stackState = LORAWAN_STATE_JOIN_FAILED;
                // -> join retry to manage out of this ... LORA_Join();
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if (certif_running() == true )
                {
                     certif_linkCheck( mlmeConfirm);
                }
            }
            break;
        }
#ifdef LORAMAC_CLASSB_ENABLED
        case MLME_BEACON_ACQUISITION:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                /* Beacon has been acquired */
                /* REquest Server for Ping Slot */
                LORA_PingSlotReq( );
            }
            else
            {
                /* Beacon not acquired */
                /* Search again */
                /* we can check if the MAC has received a time reference for the beacon*/
                /* in this case do either a Device_Time_Req  or a Beacon_Timing_req*/
                LORA_BeaconReq( );
            }
            break;
        }
        case MLME_PING_SLOT_INFO:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
               /* class B is now ativated*/
                mibReq.Type = MIB_DEVICE_CLASS;
                mibReq.Param.Class = CLASS_B;
                LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_AU915 ) || defined( REGION_US915 )
                mibReq.Type = MIB_PING_SLOT_DATARATE;
                mibReq.Param.PingSlotDatarate = DR_8;
                LoRaMacMibSetRequestConfirm( &mibReq );
#endif
                TVL2( PRINTF("\r\n#= Switch to Class B done. =#r\n" );)

                /*notify upper layer*/
                LoRaMainCallbacks->LORA_ConfirmClass(CLASS_B);
            }
            else
            {
                LORA_PingSlotReq( );
            }
            break;
        }
#if defined( USE_DEVICE_TIMING )
        case MLME_DEVICE_TIME:
        {
            if( mlmeConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK )
            {
              LORA_DeviceTimeReq();
            }
        }
#endif /* USE_DEVICE_TIMING */
#endif /* LORAMAC_CLASSB_ENABLED */
        default:
            break;
    }
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] MlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *MlmeIndication )
{
#ifdef LORAMAC_CLASSB_ENABLED
    MibRequestConfirm_t mibReq;
#endif /* LORAMAC_CLASSB_ENABLED */

    TVL2( PRINTNOW(); PRINTF("APP> MLMEInd STATUS: %s\r\n", EventInfoStatusStrings[MlmeIndication->Status] );    )

    switch( MlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {
            // The MAC signals that we shall provide an uplink as soon as possible
        	log_info("### Tx Needed\r\n");
        	lorawan_driver_onTxNeeded();
            break;
        }
#ifdef LORAMAC_CLASSB_ENABLED
        case MLME_BEACON_LOST:
        {
            // Switch to class A again
            mibReq.Type = MIB_DEVICE_CLASS;
            mibReq.Param.Class = CLASS_A;
            LoRaMacMibSetRequestConfirm( &mibReq );

            TVL2( PRINTF("\r\n#= Switch to Class A done. =# BEACON LOST\r\n" ); )

            LORA_BeaconReq();
            break;
        }
        case MLME_BEACON:
        {
            if( MlmeIndication->Status == LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED )
            {
              TraceBeaconInfo(MlmeIndication);
            }
            else
            {
              TVL2( PRINTF( "BEACON NOT RECEIVED\n\r");)
            }
            break;

        }
#endif /* LORAMAC_CLASSB_ENABLED */
        default:
            break;
    }
}


#endif //ITSDK_WITH_LORAWAN_LIB
