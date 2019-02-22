/* ==========================================================
 * statemachine.c - State Machine stuff
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
#include <it_sdk/statemachine/statemachine.h>
#include <it_sdk/debug.h>
#include <it_sdk/wrappers.h>
/**
 * statem()
 *
 * Process the current step and prepare to the next one
 */
void statem(machine_t * machine) {

	int previousState;
	uint16_t stateRet;

#if ITSDK_DEBUG_STATEM >= 1
	// In debug mode we ensure never jump on a not existing state
	// in production mode we are suppose to have a working code not doing a such thing
	if ( machine->lastState == STATE_UNKNOWN ) {
		int i;
		for (i=0 ; i < STATE_MACHINE_SZ ; i++) {
			if ( machine->stm[i].stuid ==  STATE_LAST ) break;
		}
		machine->lastState = i;
	}
	if ( machine->currentState == STATE_UNKNOWN || machine->currentState >= machine->lastState ) {
		_LOG_STATEM(("[STM][E] Invalid state (%d)\r\n",machine->currentState));
		return;
	}
#else
	if ( machine->currentState == STATE_UNKNOWN || machine->currentState >= machine->lastState ) {
		itsdk_reset();
	}
#endif

	state_t * st = &machine->stm[(int)machine->currentState];
#if ITSDK_DEBUG_STATEM >= 3
	_LOG_STATEM(("[STM][I] St(%d)[%s] param (%d) lp (%d) tlp (%d)\r\n",
			machine->currentState,
			st->name,
			(int)st->param,
			machine->loopCurrentStep,
			machine->totalLoop
	));
#endif
    machine->totalLoop++;
	previousState = machine->currentState;
	if( machine->precall != NULL ) machine->precall();
	stateRet = st->process(st->param, machine->currentState, machine->loopCurrentStep, machine->totalLoop);
	machine->currentState = (uint8_t)(stateRet & 0xFF);

	// Do we have change state of are we on the same one ?
	if ( previousState == machine->currentState ) {
		machine->loopCurrentStep++;
	} else {
#if ITSDK_DEBUG_STATEM >= 2
		_LOG_STATEM(("[STM][I] St change for (%d)[%s] - tlp (%d)\r\n",
				machine->currentState,
				machine->stm[(int)machine->currentState].name,
				machine->totalLoop));
#endif
		machine->loopCurrentStep = LOOP_INIT_VALUE;
		if( machine->stm[(int)machine->currentState].reset != NULL ) {
#if ITSDK_DEBUG_STATEM >= 3
			_LOG_STATEM(("[STM][I] Call reset for next step (%d)\r\n",machine->currentState));
#endif
			machine->stm[(int)machine->currentState].reset();
		}
	}

#if ITSDK_DEBUG_STATEM >= 3
	_LOG_STATEM(("[STM][I] Next State will (%d)[%s]\r\n",
			machine->currentState,
			machine->stm[(int)machine->currentState].name));
#endif
#ifdef ITSDK_DEBUG_STATEM
	if ( machine->currentState == STATE_UNKNOWN || machine->currentState >= machine->lastState ) {
		_LOG_STATEM(("[STM][E] Invalid next state (%d)\r\n",machine->currentState));
	}
#endif
	// If the IMMEDIATE_JUMP is set, we reenter the next state execution
	if ( ((stateRet & STATE_IMMEDIATE_JUMP) > 0) && previousState != machine->currentState ) {
		statem(machine);
	}
}

