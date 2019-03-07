/* ==========================================================
 * cayenne.h - Implement cayenne LPP protocol
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 20 févr. 2019
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
 * Cayenne LPP format : https://mydevices.com/cayenne/docs/lora/#lora-cayenne-low-power-payload
 * Basically, every data is composed of the following structure:
 * - 1 Byte : channel (UID for each of the sensors)
 * - 1 Byte : Type of Data
 * - N Byte : Data - N depends of the Type of Data
 * ==========================================================
 */

#ifndef IT_SDK_LORAWAN_CAYENNE_H_
#define IT_SDK_LORAWAN_CAYENNE_H_

#include <stdint.h>

typedef enum {
	CAYENNE_CONVERSION_SUCCESS = 0,
	CAYENNE_CONVERSION_OUTOFBOUNDS,
	CAYENNE_CONVERSION_FAILED,
	CAYENNE_CONVERSION_NOTSUPPORTED
} itsdk_cayenne_ret_e;

typedef union {
	uint8_t 	digital_input;
	uint8_t		digital_output;
	int16_t		analog_input;
	int16_t		analog_output;
	uint16_t	illuminance;
	uint8_t		presence;
	int16_t		temperature;
	uint8_t		humidity;
	struct {
		int16_t	x;
		int16_t y;
		int16_t z;
	} 			accelerometer;
	uint16_t	barometer;
	struct {
		int16_t	x;
		int16_t y;
		int16_t z;
	} 			gyrometer;
	struct {
		int32_t lat;
		int32_t lng;
		int32_t alt;
	}			location;
} itsdk_cayenne_data_u;


typedef enum {												// Data Size in Byte MSB => Most significant Byte comes first
		ITSDK_CAYENNE_TYPE_DIGITAL_INPUT	=	0x00,		// 1
		ITSDK_CAYENNE_TYPE_DIGITAL_OUTPUT	=	0x01,		// 1
		ITSDK_CAYENNE_TYPE_ANALOG_INPUT		=	0x02,		// 2 bytes MSB signed, resolution 0.01
		ITSDK_CAYENNE_TYPE_ANALOG_OUTPUT	=	0x03,		// 2 bytes MSB signed, resolution 0.01
		ITSDK_CAYENNE_TYPE_ILLUMINANCE		=	0x65,		// 2 bytes MSB unsigned in Lux
		ITSDK_CAYENNE_TYPE_PRESENCE			=	0x66,		// 1 byte
		ITSDK_CAYENNE_TYPE_TEMPERATURE		=	0x67,		// 2 byte MSB signed, resolution 0.1°C
		ITSDK_CAYENNE_TYPE_HUMIDITY			=	0x68,		// 1 byte unsigned, resolution 0.5%
		ITSDK_CAYENNE_TYPE_ACCELEROMETER	=	0x71,		// 3x2 byte MSB signed, 0.001G per axis
		ITSDK_CAYENNE_TYPE_BAROMETER		=	0x73,		// 2 byte MSB unsigned, resolution 0.1hPa
		ITSDK_CAYENNE_TYPE_GYROMETER		=	0x86,		// 3x2 byte MSB signed resolution 0.01°/s per axis
		ITSDK_CAYENNE_TYPE_GPSLOCATION		=	0x88		// 3x3 bytes MSB
															//  Latitude : 0.0001° signed MSB
															//  Longitude : 0.0001° signed MSB
															//  Altitude : 0.01 m signed MSB

} itsdk_cayenne_type_e;




itsdk_cayenne_ret_e itsdk_cayenne_encodePayload(
		uint8_t channel,				// Data channel / Sensor Id (uniq value per sensor, no rules)
		itsdk_cayenne_type_e  dataType, // type of data - see ITSDK_CAYENNE_TYPE_XXXXX
		itsdk_cayenne_data_u *data, 	// Data to use in the unit defined in cayenne.h
		uint8_t * buffer,  				// Buffer where to encode the data
		int		* index,				// Index where to start encode the data into the Buffer, Index is updated
		int		  bufferSz				// Max buffer size.
);



#endif /* IT_SDK_LORAWAN_CAYENNE_H_ */
