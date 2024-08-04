/* ==========================================================
 * lbm_mcu.h - headers for LoRaWan LBM library, board api
 * ----------------------------------------------------------
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
 * ---------------------------------------------------------
 *
 *  Created on: 30 july 2024
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2024
 * ==========================================================
 */
#ifndef IT_SDK_DRIVER_LORA_LBM_MCU_H_
#define IT_SDK_DRIVER_LORA_LBM_MCU_H_
#include <it_sdk/config.h>

#if ITSDK_WITH_LORAWAN_LIB == __ENABLE && ITSDK_LORAWAN_STACK == __LORAWAN_LBM

#include <it_sdk/logger/logger.h>

#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWLORADBG) > 0
	#define _LOG_LBMMCU_DEBUG(x)	log_debug x
	#define _LOG_LBMMCU_INFO(x)		log_info x
	#define _LOG_LBMMCU_WARN(x)		log_warn x
	#define _LOG_LBMMCU_ERROR(x)	log_error x
#else
	#define _LOG_LBMMCU_DEBUG(x)
	#define _LOG_LBMMCU_INFO(x)
	#define _LOG_LBMMCU_WARN(x)
	#define _LOG_LBMMCU_ERROR(x)
#endif


#endif // ITSDK_WITH_LORAWAN_LIB == __ENABLE && ITSDK_LORAWAN_STACK == __LORAWAN_LBM
#endif // IT_SDK_DRIVER_LORA_LBM_MCU_H_
