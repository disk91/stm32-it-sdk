/* ==========================================================
 * scheduler.h - Manage recurrent tasks
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 15 sept. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018  IngeniousThings and Disk91
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
 * Try to be tinny as possible ;)
 *
 * ==========================================================
 */

#ifndef IT_SDK_SCHEDULER_H_
#define IT_SDK_SCHEDULER_H_

#include <it_sdk/config.h>
#include <it_sdk/itsdk.h>

#define ITSDK_SCHED_CONF_DEFAULT	0x00
#define ITSDK_SCHED_CONF_SKIP		0x01		// SKIP mode will avoid the scheduler to run twice (or more)
												// if it starts to be late and iteration have been missed
												// with not SKIP the scheduler executer a task on every period but
												// it can be consecutive, not after period time.
#define ITSDK_SCHED_CONF_HALT		0x02		// The scheduler is halted (1) / Running (0)

#define ITSDK_SCHED_CONF_IMMEDIATE	0x100		// The task is immediately executed vs will be executed in next time period

#define ITSDK_SCHED_ERROR			0xFF

/**
 * schedule configuration
 */
typedef struct s_sched {
	uint64_t nextRun;			// Next run time in ms
	uint16_t period;			// Period between 2 execution
								// Mode as a bitfield for task configuration see ITSDK_SCHED_CONF_XXX
	uint8_t  skip:1;			//    skip missed execution
	uint8_t	 halt:1;			//    stop for next executions
	void (*func)(void);			// function to call
} sched_t;


/**
 * Functions
 */
void itdt_sched_execute();
uint8_t itdt_sched_registerSched(uint16_t periodMs,uint16_t mode, void (*f)(void));
void itdf_sched_haltSched(uint8_t schedId);
void itdf_sched_runSched(uint8_t schedId);


#endif 	// IT_SDK_SCHEDULER_H_
