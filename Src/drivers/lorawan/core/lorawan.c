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
#include <it_sdk/logger/error.h>
#include <it_sdk/time/timer.h>
#include <it_sdk/lorawan/lorawan.h>
#include <drivers/lorawan/core/lorawan.h>
#include <drivers/lorawan/mac/LoRaMac.h>
#include <drivers/lorawan/core/lora-test.h>
#include <drivers/lorawan/compiled_region.h>


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

	while (    __loraWanState.joinState != LORAWAN_STATE_NONE
			&& __loraWanState.joinState != LORAWAN_STATE_INITIALIZED
			&& __loraWanState.reqPending ) {
		__loraWanState.reqPending=false;
        LoRaMacProcess( );
	}

}

__weak void lorawan_driver_waitUntilEndOfExecution() {

	lorawan_driver_loop();
	#if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
	   wdg_refresh();
	#endif
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
	LOG_INFO_LORAWAN(("[LoRaWAN] data received %d bytes\r\n",size));
}


/**
 * Callback function on JOIN Success
 */
__weak void lorawan_driver_onJoinSuccess() {
	LOG_INFO_LORAWAN(("[LoRaWAN] Join success\r\n"));
}

/**
 * Callback function on JOIN Failed
 */
__weak void lorawan_driver_onJoinFailed() {
	LOG_INFO_LORAWAN(("[LoRaWAN] Join failed\r\n"));
}

/**
 * Callback function on Send Success
 */
__weak void lorawan_driver_onSendSuccess() {
	LOG_INFO_LORAWAN(("[LoRaWAN] Send success\r\n"));
}

/**
 * Callback function on Send+Ack Success
 */
__weak void lorawan_driver_onSendAckSuccess() {
	LOG_INFO_LORAWAN(("[LoRaWAN] Send+Ack success\r\n"));
}

/**
 * Callback function on Send+Ack Success & downlink is pending
 */
__weak void lorawan_driver_onPendingDownlink() {
	LOG_INFO_LORAWAN(("[LoRaWAN] Pending downlink\r\n"));
}


/**
 * Callback function on Send Success but Ack Failed
 */
__weak void lorawan_driver_onSendSuccessAckFailed() {
	LOG_INFO_LORAWAN(("[LoRaWAN] Send sucess Ack failed\r\n"));
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
  __loraWanState.reqPending=true;
}

/**
 * Called after attribute change for NVM storage
 */
void lorawan_driver_nvmContextChange(LoRaMacNvmCtxModule_t module) {
	LOG_INFO_LORAWAN(("[LoRaWAN] Nvm Change\r\n"));
}

/**
 * Callback function requesting transmission
 */
__weak void lorawan_driver_onTxNeeded() {
	LOG_INFO_LORAWAN(("[LoRaWAN] Network Server is asking for an uplink transmission\r\n"));
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
		ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_INVALID_DR,(uint16_t)itsdkDr);
	}
	return 0;	// never reached
}

// From itsdk -> semtech
static uint8_t __convertTxPower(uint8_t pwr) {
	switch (pwr) {
	case LORAWAN_TXPOWER_0 : return TX_POWER_0;
	case LORAWAN_TXPOWER_1 : return TX_POWER_1;
	case LORAWAN_TXPOWER_2 : return TX_POWER_2;
	case LORAWAN_TXPOWER_3 : return TX_POWER_3;
	case LORAWAN_TXPOWER_4 : return TX_POWER_4;
	case LORAWAN_TXPOWER_5 : return TX_POWER_5;
	case LORAWAN_TXPOWER_6 : return TX_POWER_6;
	case LORAWAN_TXPOWER_7 : return TX_POWER_7;
	case LORAWAN_TXPOWER_8 : return TX_POWER_8;
	case LORAWAN_TXPOWER_9 : return TX_POWER_9;
	case LORAWAN_TXPOWER_10 : return TX_POWER_10;
	case LORAWAN_TXPOWER_11 : return TX_POWER_11;
	case LORAWAN_TXPOWER_12 : return TX_POWER_12;
	case LORAWAN_TXPOWER_13 : return TX_POWER_13;
	case LORAWAN_TXPOWER_14 : return TX_POWER_14;
	case LORAWAN_TXPOWER_15 : return TX_POWER_15;
	default:
		ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_INVALID_TXPWR,(uint16_t)pwr);
	}
	return 0;	// never reached
}

// from semtech -> itsdk
static uint8_t __unconvertTxPower(uint8_t pwr) {
	switch (pwr) {
	case TX_POWER_0 : return LORAWAN_TXPOWER_0;
	case TX_POWER_1 : return LORAWAN_TXPOWER_1;
	case TX_POWER_2 : return LORAWAN_TXPOWER_2;
	case TX_POWER_3 : return LORAWAN_TXPOWER_3;
	case TX_POWER_4 : return LORAWAN_TXPOWER_4;
	case TX_POWER_5 : return LORAWAN_TXPOWER_5;
	case TX_POWER_6 : return LORAWAN_TXPOWER_6;
	case TX_POWER_7 : return LORAWAN_TXPOWER_7;
	case TX_POWER_8 : return LORAWAN_TXPOWER_8;
	case TX_POWER_9 : return LORAWAN_TXPOWER_9;
	case TX_POWER_10 : return LORAWAN_TXPOWER_10;
	case TX_POWER_11 : return LORAWAN_TXPOWER_11;
	case TX_POWER_12 : return LORAWAN_TXPOWER_12;
	case TX_POWER_13 : return LORAWAN_TXPOWER_13;
	case TX_POWER_14 : return LORAWAN_TXPOWER_14;
	case TX_POWER_15 : return LORAWAN_TXPOWER_15;
	default:
		ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_INVALID_TXPWR,(uint16_t)pwr);
	}
	return 0;	// never reached
}


// =======================================================================================
// Some string for trace
// =======================================================================================

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWLORAINF) > 0
// MAC event info status strings.
const char* EventInfoStatusStrings[] =
{
    "OK", "Error", "Tx timeout", "Rx 1 timeout",
    "Rx 2 timeout", "Rx1 error", "Rx2 error",
    "Join failed", "Downlink repeated", "Tx DR payload size error",
    "Downlink too many frames loss", "Address fail", "MIC fail",
    "Multicast faile", "Beacon locked", "Beacon lost", "Beacon not found"
};
#endif

static struct {
	uint8_t		port;
	uint8_t 	size;
	uint8_t 	data[ITSDK_LORAWAN_MAX_DWNLNKSZ];

} __lorawan_driver_lastDownlink;

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
  LOG_INFO_LORAWAN(("lorawan_driver_LORA_Init\r\n"));

  __loraWanState.joinState = LORAWAN_STATE_NONE;
  __loraWanState.upLinkCounter = 0;
  __loraWanState.downlinkCounter = 0;
  __loraWanState.lastRssi = LORAWAN_DRIVER_INVALID_RSSI;
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
        	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915 );
        	break;
		#endif
		#if defined( REGION_RU864 )
        case __LORAWAN_REGION_RU864:
        	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_RU864 );
        	break;
		#endif
        default:
        	LOG_ERROR_LORAWAN(("[LoRaWan] Invalid region selected\r\n"));
    		ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_INVALID_REGION,(uint16_t)config->region);
        	return;
        }

		#if ITSDK_LORAWAN_REGION_ALLOWED == __LORAWAN_REGION_NONE
			#error "Please define a region in the compiler options."
		#endif


  	    // Set the default configuration to the MIB
 	  	MibRequestConfirm_t mibReq;

		#if ITSDK_LORAWAN_MAX_RX_ERROR > 0
  	  	mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;	// Change the default RX window margin error (default is +/- 10ms)
  	  	mibReq.Param.SystemMaxRxError = ITSDK_LORAWAN_MAX_RX_ERROR;
  	  	LoRaMacMibSetRequestConfirm( &mibReq );
		#endif

  	    mibReq.Type = MIB_ADR;
  	    mibReq.Param.AdrEnable = config->adrEnable;
  	    LoRaMacMibSetRequestConfirm( &mibReq );

  	    mibReq.Type = MIB_PUBLIC_NETWORK;
  	    mibReq.Param.EnablePublicNetwork = config->enablePublicNetwork;
  	    LoRaMacMibSetRequestConfirm( &mibReq );

  	    __loraWanState.txDatarate = __convertDR(config->txDatarate);
  	    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
  	    mibReq.Param.ChannelsDefaultDatarate = __convertDR(config->txDatarate);
  	    LoRaMacMibSetRequestConfirm( &mibReq );

  	    __loraWanState.JoinType = config->JoinType;
  	    if ( config->JoinType == __LORAWAN_OTAA ) {
  	    	mibReq.Type = MIB_APP_KEY;
  	    	mibReq.Param.AppKey = config->config.otaa.appKey;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );

  	    	mibReq.Type = MIB_NWK_KEY;
  	    	mibReq.Param.NwkKey = config->config.otaa.nwkKey;
  	    	LoRaMacMibSetRequestConfirm( &mibReq );

  	    	// Store the config element not in MIB
  	        bcopy(config->devEui,__loraWanState.join.otaa.devEui,8);
  	        bcopy(config->config.otaa.appEui,__loraWanState.join.otaa.appEui,8);

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
		#if ITSDK_LORAWAN_MAX_RX_ERROR > 0
		  mibReq.Param.SystemMaxRxError = ITSDK_LORAWAN_MAX_RX_ERROR;
		#else
		  mibReq.Param.SystemMaxRxError = 10;		// default 10ms
		#endif
		LoRaMacMibSetRequestConfirm( &mibReq );




		// Sounds like this is remapping the channels
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
         __loraWanState.joinState = LORAWAN_STATE_INITIALIZED;

}


// =======================================================================================
// Join
// =======================================================================================

static MlmeReqJoin_t JoinParameters;
itsdk_lorawan_join_t lorawan_driver_LORA_Join(
		itsdk_lorawan_run_t 	  runMode
){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_Join (mode:%d)\r\n",runMode));

    switch (__loraWanState.JoinType) {
    case __LORAWAN_OTAA:
    	{
    	    MlmeReq_t mlmeReq;
    	    mlmeReq.Type = MLME_JOIN;
    	    mlmeReq.Req.Join.DevEui = __loraWanState.join.otaa.devEui;
    	    mlmeReq.Req.Join.JoinEui = __loraWanState.join.otaa.appEui;
    	    mlmeReq.Req.Join.Datarate = __loraWanState.txDatarate;
    	    JoinParameters = mlmeReq.Req.Join;

    	    LoRaMacStatus_t r = LoRaMacMlmeRequest( &mlmeReq );
			if ( r != LORAMAC_STATUS_OK ) {
				LOG_WARN_LORAWAN(("LoRaMacMlmeRequest return error(%d)\r\n",r));
				__loraWanState.joinState = LORAWAN_STATE_JOIN_FAILED;
				lorawan_driver_onJoinFailed();
			} else {
				__loraWanState.joinState = LORAWAN_STATE_JOINING;
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
			__loraWanState.joinState = LORAWAN_STATE_JOIN_SUCCESS;
			__loraWanState.joinTime = (uint32_t)(itsdk_time_get_ms()/1000);


			lorawan_driver_onJoinSuccess();
    	}
        break;
    default:
		ITSDK_ERROR_REPORT(ITSDK_ERROR_LORAWAN_INVALID_JOIN,(uint16_t)__loraWanState.JoinType);
    }

    if (runMode==LORAWAN_RUN_SYNC) {
        // Go for synchronous
    	while(__loraWanState.joinState == LORAWAN_STATE_JOINING) {
    		lorawan_driver_waitUntilEndOfExecution();
    	}
    	if ( __loraWanState.joinState == LORAWAN_STATE_JOIN_SUCCESS ) {
    		return LORAWAN_JOIN_SUCCESS;
    	} else {
    		return LORAWAN_JOIN_FAILED;
    	}
    } else {
    	return LORAWAN_JOIN_PENDING;
    }

}

// =======================================================================================
// Send
// =======================================================================================

/**
 * Send a LoRaWan frame
 */
itsdk_lorawan_send_t lorawan_driver_LORA_Send(
		uint8_t					* payload,
		uint8_t					  size,
		uint8_t					  port,
		uint8_t					  dataRate,
		itsdk_lorawan_sendconf_t  isTxConfirmed,
		uint8_t					  retry,
		itsdk_lorawan_run_t 	  runMode,
		uint8_t					* rPort,				// for sync mode only - on reception - Port
		uint8_t					* rSize,				// for sync mode only - on reception - DataSize - contains maxSize on input
		uint8_t					* rData					// for sync mode only - on reception - Data (bcopied)
){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_Send (mode:%d)\r\n",runMode));

    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if (__loraWanState.joinState != LORAWAN_STATE_JOIN_SUCCESS ) return LORAWAN_SEND_NOT_JOINED;
    if (__loraWanState.sendState == LORAWAN_SEND_STATE_RUNNING ) return LORAWAN_SEND_ALREADYRUNNING;

    /*if certification test are on going, application data is not sent*/
    if (certif_running() == true) {
      return false;
    }

    // Update the Datarate information this is important to correctly calculate the max size of frame
    // for the LoRaMacQueryTxPossible function
    MibRequestConfirm_t set;
    set.Type = MIB_CHANNELS_DATARATE;
    set.Param.ChannelsDatarate = __convertDR(dataRate);
    LoRaMacMibSetRequestConfirm(&set);

    // Verify if a command can be proceed by the MAC Layer
    if( LoRaMacQueryTxPossible( size, &txInfo ) != LORAMAC_STATUS_OK ) {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = __convertDR(dataRate);
        // @TODO here we do not send the expected payload so we may have a callback to notice this
		#warning "Manage the Flush MAC case"
    } else {
    	__loraWanState.lastRetries = 0;
    	// Ok To proceed
        if( isTxConfirmed == LORAWAN_SEND_UNCONFIRMED )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = port;
            mcpsReq.Req.Unconfirmed.fBufferSize = size;
            mcpsReq.Req.Unconfirmed.fBuffer = payload;
            mcpsReq.Req.Unconfirmed.Datarate = __convertDR(dataRate);
        }
        else
        {
        	mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = port;
            mcpsReq.Req.Confirmed.fBufferSize = size;
            mcpsReq.Req.Confirmed.fBuffer = payload;
            mcpsReq.Req.Confirmed.NbTrials = retry;
            mcpsReq.Req.Confirmed.Datarate = __convertDR(dataRate);
        }
    }
    __loraWanState.sendState = LORAWAN_SEND_STATE_RUNNING;
    LoRaMacStatus_t r = LoRaMacMcpsRequest( &mcpsReq );
    switch ( r ) {
    	case LORAMAC_STATUS_OK:
    		if ( runMode==LORAWAN_RUN_SYNC ) {
    	    	while(  __loraWanState.sendState == LORAWAN_SEND_STATE_RUNNING ) {
    	    		lorawan_driver_waitUntilEndOfExecution();
    	    	}
    	    	switch(__loraWanState.sendState) {
    	    	case LORAWAN_SEND_STATE_SENT:
    	    		return LORAWAN_SEND_SENT;
    	    	case LORAWAN_SEND_STATE_ACKED_WITH_DOWNLINK:
    	    	case LORAWAN_SEND_STATE_ACKED_DOWNLINK_PENDING:
    	    		if ( rData != NULL && rPort != NULL && rSize != NULL) {
						*rPort = __lorawan_driver_lastDownlink.port;
						if ( *rSize >= __lorawan_driver_lastDownlink.size) {
							*rSize = __lorawan_driver_lastDownlink.size;
							bcopy(
								__lorawan_driver_lastDownlink.data,
								rData,
								((ITSDK_LORAWAN_MAX_DWNLNKSZ<__lorawan_driver_lastDownlink.size)?ITSDK_LORAWAN_MAX_DWNLNKSZ:__lorawan_driver_lastDownlink.size)
							);
						} else {
							bcopy(
								__lorawan_driver_lastDownlink.data,
								rData,
								*rSize
							);
							*rSize = __lorawan_driver_lastDownlink.size;
						}
    	    		} else {
    	    			LOG_WARN_LORAWAN(("[LoRaWan] Receiving downlink but can't return it\r\n"));
    	    		}
    	    		return (__loraWanState.sendState ==LORAWAN_SEND_STATE_ACKED_WITH_DOWNLINK)?LORAWAN_SEND_ACKED_WITH_DOWNLINK:LORAWAN_SEND_ACKED_WITH_DOWNLINK_PENDING;
    	    	case LORAWAN_SEND_STATE_ACKED_NO_DOWNLINK:
    	    		return LORAWAN_SEND_ACKED;
    	    	case LORAWAN_SEND_STATE_NOTACKED:
    	    		return LORAWAN_SEND_SENT;
    	    	default:
    	    		LOG_INFO_LORAWAN(("Abnormal state : %d\r\n",__loraWanState.sendState));
    	    		return LORAWAN_SEND_FAILED;
    	    	}
   	    		return LORAWAN_SEND_FAILED;	// Never reached
    		} else {
    			return LORAWAN_SEND_RUNNING;
    		}
    	case LORAMAC_STATUS_DUTYCYCLE_RESTRICTED:
    		__loraWanState.sendState = LORAWAN_SEND_STATE_DUTYCYCLE;
    		return LORAWAN_SEND_DUTYCYCLE;
    	case LORAMAC_STATUS_NO_NETWORK_JOINED:
    		__loraWanState.sendState = LORAWAN_SEND_STATE_FAILED;
    		return LORAWAN_SEND_NOT_JOINED;
    	default:
    		__loraWanState.sendState = LORAWAN_SEND_STATE_FAILED;
    		LOG_WARN_LORAWAN(("[LoRaWan] can't send err(%d)\r\n",r));
    		return LORAWAN_SEND_FAILED;
    }

}


/**
 * Configure the channels.
 * Default channels can't be overiden
 * For each of the channel the following information are provided
 * - channelID - entry Id in the channel table
 * - frequency - central freq
 * - rx1frequency - expected rx frequency for this channel (usualy the same)
 * - min / max DataRate format __LORAWAN_DR_xx
 * - band - index in the band[] dfined in the RegionXXMMM.h file - in most case this parameter is not required and computed as part ot the init
 */
itsdk_lorawan_channel_t lorawan_driver_LORA_AddChannel(
		uint8_t		channelId,
		uint32_t 	frequency,
		uint32_t	rx1Frequency,
		uint8_t		minDataRate,
		uint8_t		maxDataRate,
		uint8_t		band
){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_AddChannel (%d)\r\n",channelId));

	ChannelParams_t params;
	params.Frequency=frequency;
	params.Rx1Frequency=rx1Frequency;
	params.Band=band;					// Sounds like this parameter is completed during initialization
										// Band definitions are in the different RegionXXMMM.h file
										// This entry is the index in the band definition
	params.DrRange.Fields.Min=__convertDR(minDataRate);
	params.DrRange.Fields.Max=__convertDR(maxDataRate);

	LoRaMacStatus_t r = LoRaMacChannelAdd(channelId, params);
	switch ( r ) {
		case LORAMAC_STATUS_OK:
			return LORAWAN_CHANNEL_SUCCESS;
		case LORAMAC_STATUS_PARAMETER_INVALID:
		case LORAMAC_STATUS_FREQ_AND_DR_INVALID:
		case LORAMAC_STATUS_DATARATE_INVALID:
		case LORAMAC_STATUS_FREQUENCY_INVALID:
			LOG_WARN_LORAWAN(("[LoRaWan] Invalid channel configuration (%d)\r\n",r));
			return LORAWAN_CHANNEL_INVALID_PARAMS;
		default:
			LOG_WARN_LORAWAN(("[LoRaWan] Channel configuration error (%d)\r\n",r));
			return LORAWAN_CHANNEL_FAILED;
	}
}

/**
 * Change channel mask to enable only the one we need
 * The channels parameter is a table containing x time 16b corresponding
 * to the possible channels
 * for US915 as an example we have 6 entries of 16b in the tab for the 72 possible channels
 */
itsdk_lorawan_channel_t lorawan_driver_LORA_SelectChannels(uint16_t region, uint16_t * channels ){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_SelectChannels (%d)\r\n",region));
	ChanMaskSetParams_t chanMaskSet;
	chanMaskSet.ChannelsMaskType = CHANNELS_REINIT_MASK;
	chanMaskSet.ChannelsMaskIn = channels;
	switch ( region ) {
	case __LORAWAN_REGION_US915:
		if ( RegionChanMaskSet(LORAMAC_REGION_US915,&chanMaskSet) ) {
			return LORAWAN_CHANNEL_SUCCESS;
		}
		break;
	default:
		break;
	}
	LOG_WARN_LORAWAN(("[LoRaWan] Channel configuration error\r\n"));
	return LORAWAN_CHANNEL_FAILED;
}


/**
 * Remove a previously defined channel
 * channelId - entry Id in the channel table
 */
itsdk_lorawan_channel_t lorawan_driver_LORA_RemoveChannel(uint8_t channelId){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_RemoveChannel (%d)\r\n",channelId));
	if ( LoRaMacChannelRemove(channelId) == LORAMAC_STATUS_OK ) {
		return LORAWAN_CHANNEL_SUCCESS;
	} else {
		LOG_WARN_LORAWAN(("[LoRaWan] Channel removal error\r\n"));
		return LORAWAN_CHANNEL_FAILED;
	}
}

/**
 * Returns the last Uplink frame counter
 */
uint16_t lorawan_driver_LORA_GetUplinkFrameCounter(){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_GetUplinkFrameCounter\r\n"));
	return __loraWanState.upLinkCounter;
}

/**
 * Returns the last downlink frame counter
 */
uint16_t lorawan_driver_LORA_GetDownlinkFrameCounter(){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_GetDownlinkFrameCounter\r\n"));
	return __loraWanState.downlinkCounter;
}


/**
 * Return the TX_POWER
 * 0 = TX_POWER_0 => Max EIRP
 * 1 = TX_POWER_1 => Max EIRP - 2
 * 2 = TX_POWER_2 => Max EIRP - 4
 * ...
 * see https://stackforce.github.io/LoRaMac-doc & Region.h
 */
itsdk_lorawan_txpower lorawan_driver_LORA_GetTxPower(){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_GetTxPower\r\n"));

	 MibRequestConfirm_t mib;

	 mib.Type = MIB_CHANNELS_TX_POWER;
	 if ( LoRaMacMibGetRequestConfirm(&mib) == LORAMAC_STATUS_OK) {
		 return __unconvertTxPower(mib.Param.ChannelsTxPower);
	 }
	 return 0xFF;
}

/**
 * Change the Transmission power
 * The transmission power is not in dB but in predefined values regarding Max ERP
 * according to documentation.
 */
bool lorawan_driver_LORA_SetTxPower(itsdk_lorawan_txpower txPwr ){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_SetTxPower\r\n"));
    MibRequestConfirm_t mib;

	mib.Type = MIB_CHANNELS_TX_POWER;
	mib.Param.ChannelsTxPower = __convertTxPower(txPwr);
	if ( LoRaMacMibSetRequestConfirm(&mib) == LORAMAC_STATUS_OK ){
		return true;
	}
	return false;
}


/**
 * Return the last RSSI/SNR from the last downlink or Ack message.
 */
itsdk_lorawan_rssisnr_t lorawan_driver_LORA_GetLastRssiSnr(int16_t *rssi, uint8_t *snr){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_GetLastRssiSnr\r\n"));
	if ( __loraWanState.lastRssi != LORAWAN_DRIVER_INVALID_RSSI ) {
		*rssi = __loraWanState.lastRssi;
		*snr = __loraWanState.lastSnr;
		return LORAWAN_RSSISNR_VALID;
	}
	return LORAWAN_RSSISNR_INVALID;
}

/**
 * Change default DR (use on Join request)
 */
void lorawan_driver_LORA_ChangeDefaultRate(uint8_t newRate){
	LOG_INFO_LORAWAN(("lorawan_driver_LORA_ChangeDefaultRate\r\n"));
	__loraWanState.txDatarate = __convertDR(newRate);
}


/**
 * Return the current JoinState - use to follow the async join procedure
 * if used in polling mode
 */
lorawan_driver_joinState lorawan_driver_LORA_getJoinState(){
	LOG_DEBUG_LORAWAN(("lorawan_driver_LORA_getJoinState\r\n"));
	return __loraWanState.joinState;
}

/**
 * Return the current/last SendState - use to follow the async send procedure
 * if used in polling mode
 */
lorawan_driver_sendState lorawan_driver_LORA_getSendState(){
	LOG_DEBUG_LORAWAN(("lorawan_driver_LORA_getSendState\r\n"));
	return __loraWanState.sendState;
}


// =============================================================================================
// MCPS ( TX & RX Operations ) LAYER
// =============================================================================================


static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{

    TVL2( PRINTNOW(); PRINTF("APP> McpsConfirm STATUS: %s\r\n", EventInfoStatusStrings[mcpsConfirm->Status] ); )

	switch (mcpsConfirm->Status){
	case LORAMAC_EVENT_INFO_STATUS_OK:
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
            	__loraWanState.sendState = LORAWAN_SEND_STATE_SENT;
            	lorawan_driver_onSendSuccess();
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
            	if(mcpsConfirm->AckReceived){
            		// There are two type of ACK : w & w/o downlink, we will set the status later in the MLME layer
                	//__loraWanState.sendState = LORAWAN_SEND_STATE_ACKED;
                	lorawan_driver_onSendSuccess();
                	lorawan_driver_onSendAckSuccess();
            	} else {
            		// Apprently when no ACK the status is no LORAMAC_EVENT_INFO_STATUS_OK so this branch
            		// is not executed
                	__loraWanState.sendState = LORAWAN_SEND_STATE_NOTACKED;
                	lorawan_driver_onSendSuccess();
                	lorawan_driver_onSendSuccessAckFailed();
            	}
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
            	__loraWanState.sendState = LORAWAN_SEND_STATE_NONE;
            	break;
            }
            default:
            	__loraWanState.sendState = LORAWAN_SEND_STATE_FAILED;
                break;
        }
        break;
    case LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT:
    	__loraWanState.sendState = LORAWAN_SEND_STATE_NOTACKED;
    	lorawan_driver_onSendSuccess();
    	lorawan_driver_onSendSuccessAckFailed();
    	break;
    default:
    	LOG_WARN_LORAWAN(("[LoRaWan] MCPSc returns(%d)\r\n",mcpsConfirm->Status));
    	__loraWanState.sendState = LORAWAN_SEND_STATE_FAILED;
	}

    __loraWanState.upLinkCounter = mcpsConfirm->UpLinkCounter;
    __loraWanState.lastRetries = mcpsConfirm->NbRetries;

    //implicitely desactivated when VERBOSE_LEVEL < 2
    //TraceUpLinkFrame(mcpsConfirm);

}

static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    TVL2( PRINTNOW(); PRINTF("APP> McpsInd STATUS: %s\r\n", EventInfoStatusStrings[mcpsIndication->Status] );)

    //lora_AppData_t _AppData;
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
    	LOG_WARN_LORAWAN(("[LoRaWan] MCPSi returns(%d)\r\n",mcpsIndication->Status));
    	__loraWanState.sendState = LORAWAN_SEND_STATE_FAILED;
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
        	//log_info("??? unconfirmed\r\n");
        	// on est ici visibelemtn apres receptin d'un doanwlink
        	// #= D/L FRAME 3 =# RxWin 2, Port 0, data size 0, rssi -49, snr 7
        	// Downlink reception ...

            break;
        }
        case MCPS_CONFIRMED:
        {
        	LOG_WARN_LORAWAN(("??? MCPS_CONFIRMED\r\n"));
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
          certif_rx( mcpsIndication, &JoinParameters );
          __loraWanState.sendState = LORAWAN_SEND_STATE_ACKED_WITH_DOWNLINK;
          break;
        default:

          LOG_INFO_LORAWAN(("### Data received\r\n"));
          __lorawan_driver_lastDownlink.size = mcpsIndication->BufferSize;
          __lorawan_driver_lastDownlink.port = mcpsIndication->Port;
		  bcopy(
				mcpsIndication->Buffer,
				__lorawan_driver_lastDownlink.data,
				((mcpsIndication->BufferSize<=ITSDK_LORAWAN_MAX_DWNLNKSZ)?mcpsIndication->BufferSize:ITSDK_LORAWAN_MAX_DWNLNKSZ)
          );
          lorawan_driver_onDataReception(
        		  mcpsIndication->Port,
				  mcpsIndication->Buffer,
				  mcpsIndication->BufferSize
          );
          __loraWanState.sendState = LORAWAN_SEND_STATE_ACKED_WITH_DOWNLINK;
          break;
      }
    } else {
    	__loraWanState.sendState = LORAWAN_SEND_STATE_ACKED_NO_DOWNLINK;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
    	LOG_INFO_LORAWAN(("[LoRaWAN] Network Server is asking for an uplink transmission\r\n"));
    	__loraWanState.sendState = LORAWAN_SEND_STATE_ACKED_DOWNLINK_PENDING;
    	lorawan_driver_onPendingDownlink();

    }

    __loraWanState.lastRssi = mcpsIndication->Rssi;
    __loraWanState.downlinkCounter = mcpsIndication->DownLinkCounter;
    __loraWanState.lastSnr = mcpsIndication->Snr;

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
              __loraWanState.joinState = LORAWAN_STATE_JOIN_SUCCESS;
              __loraWanState.joinTime = (uint32_t)(itsdk_time_get_ms()/1000);
              lorawan_driver_onJoinSuccess();

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
            	__loraWanState.joinState = LORAWAN_STATE_JOIN_FAILED;
            	lorawan_driver_onJoinFailed();
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
                if (certif_running() == true ){
                     certif_linkCheck(mlmeConfirm);
                }
                LOG_DEBUG_LORAWAN(("### link_Check\r\n"));
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
        	LOG_INFO_LORAWAN(("### Tx Needed\r\n"));
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
