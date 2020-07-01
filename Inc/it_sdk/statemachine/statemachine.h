/* ==========================================================
 * statemachine.h - State Machine
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 sept. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 Disk91
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
#include <it_sdk/config.h>

#ifndef IT_SDK_STATEMACHINE_STATEMACHINE_H_
#define IT_SDK_STATEMACHINE_STATEMACHINE_H_


#define STATE_UNKNOWN		255
#define STATE_LAST			254
#define STATE_MACHINE_SZ	ITSDK_STATEMACHINE_TASKS

#define STATE_IMMEDIATE_JUMP 0x8000				// when this flag is set the next state is immedialty executed
												//  we do not wait for next iteration

#define LOOP_INIT_VALUE 		1
#define TOTAL_LOOP_INIT_VALUE	0
#define LOOP_TIMEOUT_VALUE		(10*60)			// trop de boucle tue la boucle ...

#define STATE_NAME_SIZE		ITSDK_STATEMACHINE_NAMESZ

typedef struct st_state {

	uint8_t stuid;					// State uniq ID use to identify it and jump to it
	void (*reset)(void);			// Function to be called before first loop in this step to reset
	uint16_t (*process)(void *, uint8_t, uint16_t, uint32_t);
									// State function to execute:
									// receive a static void * as parameter
									// uint8_t - The current stateId
									// uint16_t - is current loop counter
									// uint32_t - is the total loop counter
									// return the next step to jump to

	void * param;					// Param to be used (need to be cast to void*)
	char   name[STATE_NAME_SIZE];	// Human readable name for traceing

} state_t;


typedef struct st_machine {
	uint8_t   currentState;					 // currentState is usually set to 0 in the static init
	uint16_t  loopCurrentStep;		    	 // count loop in the currentState
	uint8_t   lastState;					 // Initialized with STATE_UNKNOWN, it will be calculated on first run and fill with the last of the state entry
	uint32_t  totalLoop;					 // Total loop since start of machine (%32b)
	void      (*precall)(void);				 // Function called before every state - for example to update timing.
#if !defined ITSDK_STATEMACHINE_STATIC || ITSDK_STATEMACHINE_STATIC == __DISABLE
	state_t   stm[STATE_MACHINE_SZ];		 // describes each state
											 // The last STATE UID must be STATE_LAST to determine
											 // end of states and make some extra debug control
#else
	const state_t * stm;
#endif

} machine_t;


void statem(machine_t *);					// Run the state machine


#endif /* IT_SDK_STATEMACHINE_STATEMACHINE_H_ */
