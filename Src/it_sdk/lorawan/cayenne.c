/* ==========================================================
 * cayenne.c - Implement cayenne LPP protocol
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
 * 
 *
 * ==========================================================
 */
#include <it_sdk/lorawan/cayenne.h>

/**
 * Encode the given data in the LPP format.
 * Verify the buffer size, fill the buffer from the index entry.
 * The Index is updated with the next entry.
 */
itsdk_cayenne_ret_e itsdk_cayenne_encodePayload(
		uint8_t channel,				// Data channel / Sensor Id (uniq value per sensor, no rules)
		itsdk_cayenne_type_e  dataType, // type of data - see ITSDK_CAYENNE_TYPE_XXXXX
		itsdk_cayenne_data_u *data, 	// Data to use in the unit defined in cayenne.h
		uint8_t * buffer,  				// Buffer where to encode the data
		int		* index,				// Index where to start encode the data into the Buffer, Index is updated
		int		  bufferSz				// Max buffer size.
) {
	// check size
	int dsz;
	switch (dataType) {
	case ITSDK_CAYENNE_TYPE_ANALOG_INPUT:
	case ITSDK_CAYENNE_TYPE_ANALOG_OUTPUT:
	case ITSDK_CAYENNE_TYPE_ILLUMINANCE:
	case ITSDK_CAYENNE_TYPE_TEMPERATURE:
	case ITSDK_CAYENNE_TYPE_BAROMETER:
		dsz = 4;
		break;
	case ITSDK_CAYENNE_TYPE_ACCELEROMETER:
	case ITSDK_CAYENNE_TYPE_GYROMETER:
		dsz = 8;
		break;
	case ITSDK_CAYENNE_TYPE_GPSLOCATION:
		dsz = 11;
		break;
	default:
		dsz = 3;
		break;
	}
	if ( *index+dsz > bufferSz ) return CAYENNE_CONVERSION_OUTOFBOUNDS;

	// Encode
	buffer[*index] = channel;
	buffer[(*index)+1] = dataType;
	switch (dataType) {

	case ITSDK_CAYENNE_TYPE_ANALOG_INPUT:
		buffer[(*index)+2] = (data->analog_input >> 8 ) & 0xFF;
		buffer[(*index)+3] = (data->analog_input & 0xFF );
		*index = (*index)+4;
		break;

	case ITSDK_CAYENNE_TYPE_ANALOG_OUTPUT:
		buffer[(*index)+2] = (data->analog_output >> 8 ) & 0xFF;
		buffer[(*index)+3] = (data->analog_output & 0xFF );
		*index = (*index)+4;
		break;

	case ITSDK_CAYENNE_TYPE_ILLUMINANCE:
		buffer[(*index)+2] = (data->illuminance >> 8 ) & 0xFF;
		buffer[(*index)+3] = (data->illuminance & 0xFF );
		*index = (*index)+4;
		break;

	case ITSDK_CAYENNE_TYPE_TEMPERATURE:
		buffer[(*index)+2] = (data->temperature >> 8 ) & 0xFF;
		buffer[(*index)+3] = (data->temperature & 0xFF );
		*index = (*index)+4;
		break;

	case ITSDK_CAYENNE_TYPE_BAROMETER:
		buffer[(*index)+2] = (data->barometer >> 8 ) & 0xFF;
		buffer[(*index)+3] = (data->barometer & 0xFF );
		*index = (*index)+4;
		break;

	case ITSDK_CAYENNE_TYPE_DIGITAL_INPUT:
		buffer[(*index)+2] = data->digital_input;
		*index = (*index)+3;
		break;

	case ITSDK_CAYENNE_TYPE_DIGITAL_OUTPUT:
		buffer[(*index)+2] = data->digital_output;
		*index = (*index)+3;
		break;

	case ITSDK_CAYENNE_TYPE_PRESENCE:
		buffer[(*index)+2] = data->presence;
		*index = (*index)+3;
		break;

	case ITSDK_CAYENNE_TYPE_HUMIDITY:
		buffer[(*index)+2] = data->humidity;
		*index = (*index)+3;
		break;

	case ITSDK_CAYENNE_TYPE_ACCELEROMETER:
		buffer[(*index)+2] = (data->accelerometer.x >> 8 ) & 0xFF;
		buffer[(*index)+3] = (data->accelerometer.x & 0xFF );
		buffer[(*index)+4] = (data->accelerometer.y >> 8 ) & 0xFF;
		buffer[(*index)+5] = (data->accelerometer.y & 0xFF );
		buffer[(*index)+6] = (data->accelerometer.z >> 8 ) & 0xFF;
		buffer[(*index)+7] = (data->accelerometer.z & 0xFF );
		*index = (*index)+8;
		break;

	case ITSDK_CAYENNE_TYPE_GYROMETER:
		buffer[(*index)+2] = (data->gyrometer.x >> 8 ) & 0xFF;
		buffer[(*index)+3] = (data->gyrometer.x & 0xFF );
		buffer[(*index)+4] = (data->gyrometer.y >> 8 ) & 0xFF;
		buffer[(*index)+5] = (data->gyrometer.y & 0xFF );
		buffer[(*index)+6] = (data->gyrometer.z >> 8 ) & 0xFF;
		buffer[(*index)+7] = (data->gyrometer.z & 0xFF );
		*index = (*index)+8;
		break;

	case ITSDK_CAYENNE_TYPE_GPSLOCATION:
		buffer[(*index)+2] = (data->location.lat >> 16 ) & 0xFF;
		buffer[(*index)+3] = (data->location.lat >> 8 ) & 0xFF;
		buffer[(*index)+4] = (data->location.lat & 0xFF );
		buffer[(*index)+5] = (data->location.lng >> 16 ) & 0xFF;
		buffer[(*index)+6] = (data->location.lng >> 8 ) & 0xFF;
		buffer[(*index)+7] = (data->location.lng & 0xFF );
		buffer[(*index)+8] = (data->location.alt >> 16 ) & 0xFF;
		buffer[(*index)+9] = (data->location.alt >> 8 ) & 0xFF;
		buffer[(*index)+10] = (data->location.alt & 0xFF );
		*index = (*index)+11;
		break;
	default:
		return CAYENNE_CONVERSION_NOTSUPPORTED;
		break;
	}
	return CAYENNE_CONVERSION_SUCCESS;
}
