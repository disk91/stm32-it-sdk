/* ==========================================================
 * compiled_region.h - Adapt the Region Define from ITSDK to
 *                     LoRa Stack
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 22 jan. 2019
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
 * ==========================================================
 */
#ifndef IT_SDK_LORAWAN_COMPILED_REGION_H_
#define IT_SDK_LORAWAN_COMPILED_REGION_H_

#include <it_sdk/config.h>
#if ITSDK_WITH_LORAWAN_LIB == __ENABLE

#if (ITSDK_LORAWAN_REGION_ALLOWED & __LORAWAN_REGION_AS923) != 0
	#define REGION_AS923
#endif

#if (ITSDK_LORAWAN_REGION_ALLOWED & __LORAWAN_REGION_AU915) != 0
	#define REGION_AU915
#endif

#if (ITSDK_LORAWAN_REGION_ALLOWED & __LORAWAN_REGION_CN470) != 0
	#define REGION_CN470
#endif

#if (ITSDK_LORAWAN_REGION_ALLOWED & __LORAWAN_REGION_CN779) != 0
	#define REGION_CN779
#endif

#if (ITSDK_LORAWAN_REGION_ALLOWED & __LORAWAN_REGION_EU433) != 0
	#define REGION_EU433
#endif

#if (ITSDK_LORAWAN_REGION_ALLOWED & __LORAWAN_REGION_EU868) != 0
	#define REGION_EU868
#endif

#if (ITSDK_LORAWAN_REGION_ALLOWED & __LORAWAN_REGION_KR920) != 0
	#define REGION_KR920
#endif

#if (ITSDK_LORAWAN_REGION_ALLOWED & __LORAWAN_REGION_IN865) != 0
	#define REGION_IN865
#endif

#if (ITSDK_LORAWAN_REGION_ALLOWED & __LORAWAN_REGION_US915) != 0
	#define REGION_US915
#endif

#if (ITSDK_LORAWAN_REGION_ALLOWED & __LORAWAN_REGION_RU864) != 0
	#define REGION_RU864
#endif



#endif	// ITSDK_WITH_LORAWAN_LIB
#endif  // IT_SDK_LORAWAN_COMPILED_REGION_H_
