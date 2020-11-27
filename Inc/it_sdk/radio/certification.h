/* ==========================================================
 * certification.h - For radio certification
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 22 nov. 2020
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

#ifndef IT_SDK_RADIO_CERTIFICATION_H_
#define IT_SDK_RADIO_CERTIFICATION_H_

itsdk_bool_e startContinousWaveTransmission(uint32_t frequency, int8_t power, uint32_t durationMs );
itsdk_bool_e stopContinousWaveTransmission( void );

#endif /* IT_SDK_RADIO_CERTIFICATION_H_ */
