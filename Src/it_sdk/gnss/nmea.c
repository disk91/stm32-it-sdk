/* ==========================================================
 * nmea.c - standard NMEA driver (not implementing vendor specific)
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
 *  Created on: 13 avr. 2020
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2020
 * ==========================================================
 */

#include <it_sdk/itsdk.h>
#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_WITH_GNSS_DRIVER == __ENABLE

#include <it_sdk/gnss/gnss.h>
#include <it_sdk/gnss/nmea.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/time/time.h>

#include <time.h>

/**
 * Process a NMEA line
 */
gnss_ret_e nmea_processNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz) {

	// check for start delimiter
	gnss_ret_e ret = GNSS_SUCCESS;
	if ( (ret = nmea_verifyChecksum(line,sz)) == GNSS_SUCCESS ) {
		if ( line[1] == 'G' && (line[2] == 'P' || line[2] == 'N') ) {
			// talker Global Positioning System or Global Navigation System
			if ( sz > 6 && line[3]=='R' && line[4] == 'M' && line[5] == 'C' ) {
				uint8_t * pt = &line[5];
				// --RMC
				// Format $--RMC,UTC Time (hhmmss.ssss),Data Valid, Latitude (ddmm.mmmm),N/S, Longitude (dddmm.mmmm), E/W, speed knot, Course over ground in degree, date (ddmmyy), magnetic variation, E/W, Position (N -Nofix / A Autonomous / D differential)
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t * time = pt;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t isValid = *pt;
				if ( isValid == 'V' ) {
					// data are not valid - no need to parse more ?
					data->fixInfo.fixType = GNSS_FIX_NONE;
					return GNSS_SUCCESS;
				}

				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t * lat = pt;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t ns = *pt;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t * lon = pt;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t ew = *pt;

				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint16_t speed;
				if ( nmea_getRationalField(pt,&speed) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;

				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint16_t cov;
				if ( nmea_getRationalField(pt,&cov) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;

				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t * date = pt;

				// Magnetic variation
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				// E/W
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				// Position system Mode
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				switch ( *pt ) {
					case 'A': data->fixInfo.positionMode = GNSS_POSMODE_AUTONOMOUS; break;
					case 'D': data->fixInfo.positionMode = GNSS_POSMODE_DIFFERENTIAL; break;
					case 'E': data->fixInfo.positionMode = GNSS_POSMODE_ESTIMATED; break;
					case 'M': data->fixInfo.positionMode = GNSS_POSMODE_MANUAL; break;
					case 'S': data->fixInfo.positionMode = GNSS_POSMODE_SIMULATOR; break;
					case 'N': data->fixInfo.positionMode = GNSS_POSMODE_INVALID; break;
					default: data->fixInfo.positionMode = GNSS_POSMODE_UNKNOWN; break;
				}
				// process dateTime
				nmea_getUTCTimeDateField(&data->gpsTime, time, date);
				itsdk_time_sync_UTC_s(data->gpsTime.hours*3600+data->gpsTime.minutes*60+data->gpsTime.seconds);
				#if ITSDK_DRIVERS_GNSS_WITH_UTCDATE_FULL == __ENABLE
				  itsdk_time_sync_EPOC_s(data->gpsTime.epoc);
				#endif

				// process position



			} else if ( sz > 6 && line[3]=='G' && line[4]=='S' && line[5]=='A' ) {
				// --GSA - DOP and Active Satellites
				//   Mode A/M Automatic switch 2D/3D or Manual, TypeOf Fix 1:N/A 2=2D 3=3D
				//	 List of sat Ids x12
				// PDOP X.XX, HDOP X.XX, VDOP X.XX
				uint8_t * pt = &line[5];
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				switch (*pt) {
					case '1':
						data->fixInfo.fixType = GNSS_FIX_NONE;
						data->fixInfo.fixTime = 0;
						data->fixInfo.hdop = 0;
						data->fixInfo.vdop = 0;
						data->fixInfo.pdop = 0;
						break;
					case '2':
						data->fixInfo.fixType = GNSS_FIX_2D;
						itsdk_time_is_UTC_s(&data->fixInfo.fixTime);
						itsdk_time_is_UTC_s(&data->lastRefreshS);
						break;
					case '3':
						data->fixInfo.fixType = GNSS_FIX_3D;
						itsdk_time_is_UTC_s(&data->fixInfo.fixTime);
						itsdk_time_is_UTC_s(&data->lastRefreshS);
						break;
					default:
						return GNSS_INVALIDFORMAT;
				}
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t usedSat = 0;
				uint16_t satNum;
				for (int i=0 ; i < 12 ; i++ ) {
					if ( nmea_getDecimalField(pt,&satNum) == GNSS_SUCCESS ) {
						usedSat++;
					}
					nmea_goNextField(&pt);
				}
				data->fixInfo.nbSatUsed = usedSat;
				uint16_t dop;
				if ( nmea_getRationalField(pt,&dop) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				data->fixInfo.pdop = dop;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_getRationalField(pt,&dop) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				data->fixInfo.hdop = dop;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_getRationalField(pt,&dop) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				data->fixInfo.vdop = dop;

			} else if ( sz > 6 && line[3]=='G' && line[4]=='S' && line[5]=='V' ) {
				// --GSV - Satellites in View
				// Format $--GSV,Total Sentences 1-9, Sentence number 1-9, Total Sat in View XX, (when 0, nothing more than an extra 0 field)
				  // ... Sat ID Number (GPS 1 à 32) / 33-64 ( WAAS) / 65-88 Glonass,
				  // ... Elevation in degree 0 - 90 , Azimuth degree 000 - 359, SNR 00 -99dB (null if not tracking)
				  // Repetition of these 2 last line 4 times for different satellites.
				// ex :  $GPGSV,1,1,00,0*65
				// ex :  $GPGSV,2,1,07,24,60,119,33,37,34,155,,19,22,043,35,02,16,108,32,0*6E
				uint8_t * pt = &line[5];
				uint16_t totSentences;
				uint16_t curSentence;
				uint16_t totSatInView;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_getDecimalField(pt,&totSentences) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_getDecimalField(pt,&curSentence) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_getDecimalField(pt,&totSatInView) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				if ( totSatInView > 0 ) {
					data->satInView = totSatInView;
					itsdk_time_is_UTC_s(&data->lastRefreshS);
					// get details on sats we have <= 4 sat data per line
					uint8_t pendingSat = (totSatInView - ((curSentence-1) * 4)) & 3;
					for ( int i = 0 ; i < pendingSat ; i++ ) {
						uint16_t satId;
						uint16_t elevation;
						uint16_t azimuth;
						uint16_t snr;
						if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
						if ( nmea_getDecimalField(pt,&satId) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
						if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
						if ( nmea_getDecimalField(pt,&elevation) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
						if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
						if ( nmea_getDecimalField(pt,&azimuth) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
						if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
						if ( nmea_getDecimalField(pt,&snr) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
						if (snr == 0) snr=0xFF;	// not tracked

						#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
						if ( satId >= 0 && satId <= 32 ) {
							// GPS Sat
							itsdk_time_is_UTC_s(&data->sat_gps[satId-1].updateTime);
							data->sat_gps[satId-1].elevation = elevation;
							data->sat_gps[satId-1].azimuth = azimuth;
							data->sat_gps[satId-1].signal = snr;
							if ( snr < data->sat_gps[satId-1].maxSignal ) data->sat_gps[satId-1].maxSignal = snr;
						}
						#endif
						#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
						if ( satId >= 65 && satId <= 88 ) {
							// GLONASS Sat
							itsdk_time_is_UTC_s(&data->sat_glonas[satId-65].updateTime);
							data->sat_glonas[satId-65].elevation = elevation;
							data->sat_glonas[satId-65].azimuth = azimuth;
							data->sat_glonas[satId-65].signal = snr;
							if ( snr < data->sat_glonas[satId-65].maxSignal ) data->sat_glonas[satId-65].maxSignal = snr;
						}
						#endif
					}
				} else {
					// in this case we have no more information

				}


			}




		} else if ( line[1] == 'P') return GNSS_PROPRIETARY;
	   //log_info("=> %s\r\n",line);


	} else {
		log_error(">> [%d] %s\r\n",ret, line);
		return ret;
	}
	return ret;

}

/**
 * Validate the NMEA line format and the ending checksum value
 * The checksum is a bit exclusive OR of all the characters betwwen
 * the '$' (line start char) and the '*' (checksum start char)
 */
gnss_ret_e nmea_verifyChecksum(uint8_t * line, uint16_t sz) {
	if ( sz < 4 ) return GNSS_INVALIDFORMAT;
	if ( line[0] != '$' ) return GNSS_INVALIDFORMAT;
	// search for "*"xx for checksum
	int chksumpos = 1;
	uint8_t chksum = 0;
	while ( chksumpos < sz && line[chksumpos] != '*' ) {
		chksum ^= line[chksumpos];
		chksumpos++;
	}
	if ( chksumpos >= sz-2 ) {
log_info("## %d %d\r\n",chksumpos,sz);
		return GNSS_INVALIDFORMAT;
	}
	if ( itdt_convertHexChar2Int((char*)&line[chksumpos+1]) != chksum ) {
log_error("Chk error %02X %02X\r\n",(int)itdt_convertHexChar2Int((char*)&line[chksumpos+1]),chksum);
		return GNSS_CHECKSUMERROR;
	}
	return GNSS_SUCCESS;
}

/**
 * Find the next field -> fields are seperated by comma
 * The pointer is updated with the new value in case of
 * success only. It point to the first digit after comma
 */
gnss_ret_e nmea_goNextField(uint8_t ** line) {
	uint8_t * pt = *line;
	do {
		uint8_t c = *pt;
		switch (c) {
		 case ',' :
			 *line = pt+1;
			 return GNSS_SUCCESS;
		 case '\0':
		 case '\r':
		 case '\n':
		 case '*' :
			 return GNSS_NOMOREFIELD;
		 default:
			 break;
		}
		pt++;
	} while(1);
}

/**
 * Get a filed composed of base 10 digit up to the next comma of end of line
 * return an error if non digit number are part of the content
 */
gnss_ret_e nmea_getDecimalField(uint8_t * line, uint16_t * number) {
	uint8_t sz = 0;
	uint16_t num = 0;
	while ( *line >= '0' && *line <= '9' ) {
		num = 10*num;
		num += *line - '0';
		line++;
		sz++;
	}
	if ( *line == ',' || *line == '*' ) {
		*number = num;
		if ( sz == 0 ) return GNSS_EMPTYFIELD;
		return GNSS_SUCCESS;
	}
	return GNSS_INVALIDFORMAT;
}

/**
 * Get a filed composed of base 10 digit with format X.XX up to the next comma of end of line
 * We return the value x100 as an uint16_t (basically . is bypassed
 * return an error if non digit number are part of the content
 *
 */
gnss_ret_e nmea_getRationalField(uint8_t * line, uint16_t * number) {
	uint8_t sz = 0;
	uint16_t num = 0;
	while ( (*line >= '0' && *line <= '9') || *line == '.' ) {
		if (*line != '.') {
			num = 10*num;
			num += *line - '0';
			sz++;
		}
		line++;
	}
	if ( *line == ',' || *line == '*' ) {
		*number = num;
		if ( sz == 0 ) return GNSS_EMPTYFIELD;
		return GNSS_SUCCESS;
	}
	return GNSS_INVALIDFORMAT;
}

/**
 * Fill a Time & Date structure from  Time and Date strings
 * Time format : hhmmss.ssss
 * Date format : ddmmyy
 *
 * hhmmss.ssss
 */
#define __NMEA_TESTCONVERT_CHAR(x) ((x >= '0' && x <= '9')?x-'0':0)
#define __NMEA_CONVERT_CHAR(x) (x-'0')
gnss_ret_e nmea_getUTCTimeDateField(gnss_date_t *pTime, uint8_t * timePt, uint8_t * datePt) {

	pTime->hours = 10*__NMEA_CONVERT_CHAR(timePt[0]) + __NMEA_CONVERT_CHAR(timePt[1]);
	pTime->minutes = 10*__NMEA_CONVERT_CHAR(timePt[2]) + __NMEA_CONVERT_CHAR(timePt[3]);
	pTime->seconds = 10*__NMEA_CONVERT_CHAR(timePt[4]) + __NMEA_CONVERT_CHAR(timePt[5]);

	pTime->day = 10*__NMEA_CONVERT_CHAR(datePt[0]) + __NMEA_CONVERT_CHAR(datePt[1]);
	pTime->month = 10*__NMEA_CONVERT_CHAR(datePt[2]) + __NMEA_CONVERT_CHAR(datePt[3]);
	pTime->year = 10*__NMEA_CONVERT_CHAR(datePt[4]) + __NMEA_CONVERT_CHAR(datePt[5]);

	#if ITSDK_DRIVERS_GNSS_WITH_UTCDATE_FULL == __ENABLE
		// This cost 6KB of code size
		struct tm t;
		t.tm_year = pTime->year + 100;
		t.tm_mon = pTime->month -1;
		t.tm_mday = pTime->day;
		t.tm_hour = pTime->hours;
		t.tm_min = pTime->minutes;
		t.tm_sec = pTime->seconds;
		t.tm_isdst = 0;
		pTime->epoc = mktime(&t);
	#else
		pTime->epoc = 0;
	#endif

	pTime->isSet = BOOL_TRUE;
	return GNSS_SUCCESS;
}


#endif // ITSDK_DRIVERS_WITH_GNSS_DRIVER
#endif // ITSDK_WITH_DRIVERS
