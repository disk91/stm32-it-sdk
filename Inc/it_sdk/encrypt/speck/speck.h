/* ==========================================================
 * speck.h - SPECK32 encryption library
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 09 dec. 2018
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
 * Library for SPECK encryption
 * - from https://github.com/shreyasj2006/Speck32-64
 *
 * ==========================================================
 */
#ifndef ITSDK_ENCRYPT_SPECK_H
#define ITSDK_ENCRYPT_SPECK_H


void speck32_encrypt(uint8_t * key, uint8_t * data, uint8_t len);

#endif //ITSDK_ENCRYPT_SPECK_H
