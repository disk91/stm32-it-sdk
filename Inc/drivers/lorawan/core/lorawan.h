/* ==========================================================
 * lorawan.h - Driver for making semtech LoRaWan stack
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
 */

#ifndef IT_SDK_DRIVERS_LORAWAN_H_
#define IT_SDK_DRIVERS_LORAWAN_H_

typedef enum {
	LORAWAN_JOIN_PENDING = 0,
	LORAWAN_JOIN_SUCCESS,
	LORAWAN_JOIN_FAILED
} lorawan_driver_join_status;

typedef enum lorawan_driver_state_e {
	LORAWAN_STATE_NONE = 0,
	LORAWAN_STATE_INITIALIZED,
	LORAWAN_STATE_JOINING,
	LORAWAN_STATE_JOIN_SUCCESS,
	LORAWAN_STATE_JOIN_FAILED,

	LORAWAN_STATE_END
} lorawan_driver_state;


/**
 * This structure maintains the abstraction layer internal state
 */
typedef struct lorawan_driver_state_s {
	volatile lorawan_driver_state		stackState;			// current state for the stack
	uint32_t							joinTime;			// Time in S when the device has confirmed last joined
	bool								reqPending;			// Indicate a processing is pending

} lorawan_driver_state_t;


/**
 * This structure contains the LoRaStack configuration to be transfered
 * at the LoRaInit. Clean it after init as it contains sensitive informations
 */
typedef struct lorawan_driver_config_s {
    bool 		adrEnable;				// ADR -> Adaptative data rate On/Off
    int8_t 		txDatarate;				// Tx DataRate
    bool 		enablePublicNetwork;	// Public or private network
    uint16_t	region;					// Region to be set

    // __LORAWAN_OTAA or __LORAWAN_ABP
    uint8_t   JoinType;
    uint8_t * devEui;						// Dev EUI pointer (8B)
    union {
	   struct s_otaa {
		   uint8_t * appEui;				// OTAA App EUI pointer (8B)
		   uint8_t * appKey;		   		// OTAA App Key pointer (16B)
		   uint8_t * nwkKey;		   		// OTAA Nwk Key pointer (16B)
	   } otaa;
	   struct s_abp {
		   uint8_t * FNwkSIntKey;			// Nwk Internal Session Key
		   uint8_t * SNwkSIntKey;			//
		   uint8_t * nwkSEncKey;			// Nwk Session Key
		   uint8_t * appSKey;				// App Session Key
		   uint32_t  devAddr;				// Dev addr - When 0, the device address will be generated randomly
	   } abp;
    } config;
} lorawan_driver_config_t;


typedef enum {
	LORAWAN_RUN_SYNC = 0,
	LORAWAN_RUN_ASYNC
} lorawan_driver_run_t;

/**
 * API
 */

void lorawan_driver_LORA_Init(
		lorawan_driver_config_t * config
);

lorawan_driver_join_status lorawan_driver_LORA_Join(
		lorawan_driver_config_t * config,
		lorawan_driver_run_t 	  runMode
);

#endif // IT_SDK_DRIVERS_LORAWAN_H_
