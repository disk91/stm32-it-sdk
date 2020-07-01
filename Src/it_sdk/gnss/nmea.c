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
 * Supports Messages
 *            Position  Altitude  Speed  Hdop Pdop Vdop Time Date  SatStatus  COG
 *  - --RMC     x                   x                     x   x                x
 *  - --GGA     x          x               x              x
 *  - --GSA                                x   x    x
 *  - --GSV                                                            x
 *  - --GLL     x                                         x
 *
 * Not Yet supported
 *  - --VTG                         x                                          x
 *  - --ZDA												  x   x
 *  - MTKCHN														   x
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
 * Setup the right number of NMEA messaging according to the request made by the user in term of
 * information. Some of the information only exist in one type of message, some are redundant and
 * we need to reduce the number of redundancy but keeping all the needed information.
 * Currently only one type of GNSS is supported so it is not so complex... this can start to be a
 * mess later .. subject to evolution.
 * @TODO - match with supported message
 */
gnss_ret_e nmea_selectNMEAMessages(gnss_config_t * config, nmea_supported_e supported) {

	gnss_ret_e ret = GNSS_SUCCESS;

	config->driver.nmea.expectedRMC = 0;
	config->driver.nmea.expectedGSA = 0;
	config->driver.nmea.expectedGSV = 0;
	config->driver.nmea.expectedGLL = 0;
	config->driver.nmea.expectedVTG = 0;		// not yet supported
	config->driver.nmea.expectedZDA = 0;		// not yet supported
	config->driver.nmea.expectedCHN = 0;		// not yet supported


	if ( (supported & NMEA_GGA) > 0 ) {
		config->driver.nmea.expectedGGA = 1;
	} else {
		// @TODO ... here the chose starts to be complex a bit, let see it later
		config->driver.nmea.expectedRMC = 1;
		if ((ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_2DPOS) > 0) {
			ret = GNSS_NOTSUPPORTED;
		}
	}

	// Indicate to the driver the type of messages expected
	if ( (ITSDK_DRIVERS_GNSS_POSINFO & ( __GNSS_WITH_SPEED ) ) > 0 ) {
		if ( (supported & NMEA_RMC) == 0 ) {
			ret = GNSS_NOTSUPPORTED;
		} else {
			config->driver.nmea.expectedRMC = 1;
		}
	}
	if ( (ITSDK_DRIVERS_GNSS_POSINFO & ( __GNSS_WITH_COG ) ) > 0 ) {
		if ( (supported & NMEA_RMC) == 0 ) {
			ret = GNSS_NOTSUPPORTED;
		} else {
			config->driver.nmea.expectedRMC = 1;
		}
	}
	if ( (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_DATE ) > 0 ) {
		if ( (supported & NMEA_RMC) == 0 ) {
			ret = GNSS_NOTSUPPORTED;
		} else {
			config->driver.nmea.expectedRMC = 1;
		}
	}
	if ( (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_PDOP_VDOP ) > 0 ) {
		if ( (supported & NMEA_GSA) == 0 ) {
			ret = GNSS_NOTSUPPORTED;
		} else {
			config->driver.nmea.expectedGSA = 1;
		}
	}
	if ( (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS ) > 0 ) {
		if ( (supported & NMEA_GSV) == 0 ) {
			ret = GNSS_NOTSUPPORTED;
		} else {
			config->driver.nmea.expectedGSV = 1;
		}
	}
	if ( config->driver.nmea.expectedRMC == 1 && (ITSDK_DRIVERS_GNSS_POSINFO & ( __GNSS_WITH_3DPOS | __GNSS_WITH_HDOP )) == 0 ) {
		config->driver.nmea.expectedGGA = 0;
	}
	if ( (ITSDK_DRIVERS_GNSS_POSINFO & ~( __GNSS_WITH_2DPOS | __GNSS_WITH_TIME )) == 0 ) {
		if ( (supported & NMEA_GLL) > 0 ) {
			config->driver.nmea.expectedGGA = 0;
			config->driver.nmea.expectedRMC = 0;
			config->driver.nmea.expectedGLL = 1;
		}
	}

	return ret;
}

/**
 * Process a NMEA line
 */
gnss_ret_e nmea_processNMEA(gnss_data_t * data, uint8_t * line, uint16_t sz, gnss_nmea_driver_t * driver) {

	// check for start delimiter
	gnss_ret_e ret = GNSS_SUCCESS;
	if ( (ret = nmea_verifyChecksum(line,sz)) == GNSS_SUCCESS ) {
		driver->currentMessage = NMEA_NONE;
		if ( line[1] == 'G' && (line[2] == 'P' || line[2] == 'N' || line[2] == 'L' || line[2] == 'A' ) ) {
			switch(line[2]) {
				case 'P': driver->currentMessage |= NMEA_GP; break;	// GPS
				case 'N': driver->currentMessage |= NMEA_GN; break; // Multiple GPS + GLONASS
				case 'L': driver->currentMessage |= NMEA_GL; break; // Glonas
				case 'A': driver->currentMessage |= NMEA_GA; break; // Galileo
				default: break;
			}
			// talker Global Positioning System or Global Navigation System
			if ( sz > 6 && line[3]=='R' && line[4] == 'M' && line[5] == 'C' ) {
				uint8_t * pt = &line[5];
				// --RMC
				// Format $--RMC,UTC Time (hhmmss.ssss),Data Valid, Latitude (ddmm.mmmm),N/S, Longitude (dddmm.mmmm), E/W, speed knot, Course over ground in degree, date (ddmmyy), magnetic variation, E/W, Position (N -Nofix / A Autonomous / D differential)
				driver->currentMessage = NMEA_GN | NMEA_RMC;
					// I force GN for RMC as this message can switch from GP to GN as soon as a fix is performed
					// and we do not have multiple message of that type on the current MediaTek chip used...
					// see later if this evolving
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
				uint16_t cog;
				if ( nmea_getRationalField(pt,&cog) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;

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
				if ( nmea_getUTCTimeDateField(&data->gpsTime, time, date) == GNSS_SUCCESS ) {
					itsdk_time_sync_UTC_s(data->gpsTime.hours*3600+data->gpsTime.minutes*60+data->gpsTime.seconds);
					itsdk_time_is_UTC_s(&data->lastRefreshS);
				}
				#if ITSDK_DRIVERS_GNSS_WITH_UTCDATE_FULL == __ENABLE
				  itsdk_time_sync_EPOC_s(data->gpsTime.epoc);
				#endif
				if ( data->fixInfo.fixType == GNSS_FIX_NONE ) {
				   data->fixInfo.fixType = GNSS_FIX_TIME;
				}

				// process speed
				data->fixInfo.speed_knot = speed/100;
				uint32_t kmh = speed;
				kmh = (kmh * 1852) / 100000;
				data->fixInfo.speed_kmh = (uint16_t)kmh;

				// process position
				int32_t ilat=0;
				if ( nmea_getLatLngField(lat, &ilat, ns) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				int32_t ilon=0;
				if ( nmea_getLatLngField(lon, &ilon, ew) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				data->fixInfo.latitude = ilat;
				data->fixInfo.longitude = ilon;
				if ( data->fixInfo.fixType < GNSS_FIX_2D ) {
				   data->fixInfo.fixType = GNSS_FIX_2D;
				}

				// process direction
				data->fixInfo.direction = cog;

			} else if ( sz > 6 && line[3]=='G' && line[4]=='G' && line[5]=='A' ) {
				// -- GGA - Global positionning fix data with 3D information
				//     UTC Time hhmmss.sss, the time is reported also when not valid
				//	   Latitude ddmm.mmmm, N/S, Longitude dddmm.mmmm, E/W
				//	   Fix Status 0 - invalid, 1 - Gnss fix, 2 - DGPS fix, 6 - estimated , 3 -  PPS, 4 Real Time Kinematic, 5 float RTK, 7 manual input, 8 simulator
				//     Number of stats used
				//	   Hdop x.xx
				//	   Altitude in meter x.x
				//	   M => means Meter
				//     Height of GeoId above see level ? x.x
				//     M => means Meter
				//     DGPS age, Dgps station ID
				// ex - before fix - $GPGGA,161423.000,,,,,0,0,,,M,,M,,*4B
				//      when fix -- $GPGGA,161438.000,4533.4708,N,00215.7051,E,1,5,2.63,382.7,M,48.7,M,,*5E
				driver->currentMessage |= NMEA_GGA;
				uint8_t * pt = &line[5];
				// Time
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t * time = pt;

				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( *pt != ',' ) {
					// in this case it means we got a fix
					uint8_t * lat = pt;
					if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					uint8_t ns = *pt;
					if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					uint8_t * lon = pt;
					if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					uint8_t ew = *pt;

					if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					//uint8_t fixstat = *pt;

					if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					uint16_t nSatUsed;
					if ( nmea_getDecimalField(pt,&nSatUsed) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
					if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					uint16_t hdop;
					if ( nmea_getRationalField(pt,&hdop) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;

					if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					uint16_t altitude; // in meter, we get only decimal part to support up-to 65km
					itsdk_bool_e fix3d = BOOL_FALSE;
					if ( *pt != ',' ) {
						if ( nmea_getDecimalField(pt,&altitude) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
						fix3d = BOOL_TRUE;
					}

					if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					uint16_t hgeoid; // in decimeter
					if ( nmea_getRationalField(pt,&hgeoid) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;

					// skip the end
					if ( nSatUsed > 0 ) {
						// Update time
						if ( nmea_getUTCTimeDateField(&data->gpsTime, time, NULL) == GNSS_SUCCESS ) {
						    data->fixInfo.fixType = GNSS_FIX_TIME;
							itsdk_time_sync_UTC_s(data->gpsTime.hours*3600+data->gpsTime.minutes*60+data->gpsTime.seconds);
						}
						itsdk_time_is_UTC_s(&data->lastRefreshS);

						int32_t ilat=0;
						if ( nmea_getLatLngField(lat, &ilat, ns) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
						int32_t ilon=0;
						if ( nmea_getLatLngField(lon, &ilon, ew) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
						data->fixInfo.latitude = ilat;
						data->fixInfo.longitude = ilon;

						if ( fix3d == BOOL_TRUE ) {
						   data->fixInfo.fixType = GNSS_FIX_3D;
						   data->fixInfo.altitude = altitude;
						   //@TODO - let see what to do with the hgeoid
						} else {
						   data->fixInfo.fixType = GNSS_FIX_2D;
						}
						#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
							if ( line[2] == 'P' || line[2] == 'N' ) {
								data->fixInfo.gps.nbSatUsed = nSatUsed;
								data->fixInfo.gps.hdop = hdop;
							}
						#endif
						#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
							if ( line[2] == 'L' || line[2] == 'N' ) {
								data->fixInfo.glonass.nbSatUsed = nSatUsed;
								data->fixInfo.glonass.hdop = hdop;
							}
						#endif
						#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
							if ( line[2] == 'A' || line[2] == 'N' ) {
								data->fixInfo.galileo.nbSatUsed = nSatUsed;
								data->fixInfo.galileo.hdop = hdop;
							}
						#endif

						itsdk_time_is_UTC_s(&data->fixInfo.fixTime);
					}
				}

			} else if ( sz > 6 && line[3]=='G' && line[4]=='L' && line[5]=='L' ) {
				// --GLL - Geographic Lat & Lon ...
				//	   Latitude ddmm.mmmm, N/S, Longitude dddmm.mmmm, E/W
				//     UTC Time hhmmss.sss
				//     Data Valid 'V' when invalid, A when valid
				//	   Position Mode like for RMC
				driver->currentMessage |= NMEA_GLL;
				uint8_t * pt = &line[5];


				// Position
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t * lat = pt;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t ns = *pt;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t * lon = pt;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t ew = *pt;

				// Time
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t * time = pt;

				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t isValid = *pt;
				if ( isValid == 'V' ) {
					// data are not valid - no need to parse more ?
					data->fixInfo.fixType = GNSS_FIX_NONE;
					return GNSS_SUCCESS;
				}
				// Position system Mode
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t pmode = *pt;

				if ( *time != ',' ) {
					// Update time
					if ( nmea_getUTCTimeDateField(&data->gpsTime, time, NULL) == GNSS_SUCCESS ) {
						itsdk_time_sync_UTC_s(data->gpsTime.hours*3600+data->gpsTime.minutes*60+data->gpsTime.seconds);
						if ( data->fixInfo.fixType < GNSS_FIX_TIME ) data->fixInfo.fixType = GNSS_FIX_TIME;
					}
					itsdk_time_is_UTC_s(&data->lastRefreshS);

					int32_t ilat=0;
					if ( nmea_getLatLngField(lat, &ilat, ns) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					int32_t ilon=0;
					if ( nmea_getLatLngField(lon, &ilon, ew) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
					data->fixInfo.latitude = ilat;
					data->fixInfo.longitude = ilon;
					if ( data->fixInfo.fixType < GNSS_FIX_2D ) data->fixInfo.fixType = GNSS_FIX_2D;

					switch ( pmode ) {
						case 'A': data->fixInfo.positionMode = GNSS_POSMODE_AUTONOMOUS; break;
						case 'D': data->fixInfo.positionMode = GNSS_POSMODE_DIFFERENTIAL; break;
						case 'E': data->fixInfo.positionMode = GNSS_POSMODE_ESTIMATED; break;
						case 'M': data->fixInfo.positionMode = GNSS_POSMODE_MANUAL; break;
						case 'S': data->fixInfo.positionMode = GNSS_POSMODE_SIMULATOR; break;
						case 'N': data->fixInfo.positionMode = GNSS_POSMODE_INVALID; break;
						default: data->fixInfo.positionMode = GNSS_POSMODE_UNKNOWN; break;
					}

				}

			} else if ( sz > 6 && line[3]=='G' && line[4]=='S' && line[5]=='A' ) {
				// --GSA - DOP and Active Satellites
				//   Mode A/M Automatic switch 2D/3D or Manual, TypeOf Fix 1:N/A 2=2D 3=3D
				//	 List of sat Ids x12
				// PDOP X.XX, HDOP X.XX, VDOP X.XX
				// => new field indicating the GNSS System
				//      1 - GPS / 2 - GLONASS / 3- Galileo / 4 - Beidou
				driver->currentMessage |= NMEA_GSA;
				uint8_t * pt = &line[5];
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t * fixstatus = pt;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				uint8_t usedSat = 0;
				uint16_t satNum;
				for (int i=0 ; i < 12 ; i++ ) {
					if ( nmea_getDecimalField(pt,&satNum) == GNSS_SUCCESS ) {
						usedSat++;
					}
					nmea_goNextField(&pt);
				}
				uint16_t pdop;
				if ( nmea_getRationalField(pt,&pdop) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				uint16_t hdop;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_getRationalField(pt,&hdop) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				uint16_t vdop;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_getRationalField(pt,&vdop) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;

				#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
				   gnss_fix_qality_t * q = &data->fixInfo.gps;
				#elif ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
				   gnss_fix_qality_t * q = &data->fixInfo.glonass;
				#elif ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
				   gnss_fix_qality_t * q = &data->fixInfo.galileo;
				#else
					#error "At least one of the gnss shall be enabled"
				#endif
				if ( nmea_goNextField(&pt) == GNSS_SUCCESS ) {

					switch (*pt) {
					#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE
						case '2': // GLONASS
							q = &data->fixInfo.glonass;
							break;
					#endif
					#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE
						case '3':
							q = &data->fixInfo.galileo;
							break;
					#endif
					#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE
						case '1': // GPS
							q = &data->fixInfo.gps;
							break;
					#endif
						default:
							break;
					}

				}
				switch (*fixstatus) {
					case '1':
						q->fixType = GNSS_FIX_NONE;
						q->hdop = 0;
						q->vdop = 0;
						q->pdop = 0;
						break;
					case '2':
						q->fixType = GNSS_FIX_2D;
						itsdk_time_is_UTC_s(&data->fixInfo.fixTime);
						itsdk_time_is_UTC_s(&data->lastRefreshS);
						break;
					case '3':
						q->fixType = GNSS_FIX_3D;
						itsdk_time_is_UTC_s(&data->fixInfo.fixTime);
						itsdk_time_is_UTC_s(&data->lastRefreshS);
						break;
					default:
						return GNSS_INVALIDFORMAT;
				}
				if ( q->fixType > data->fixInfo.fixType ) {
					data->fixInfo.fixType = q->fixType;
				}
				q->nbSatUsed = usedSat;
				q->pdop = pdop;
				q->hdop = hdop;
				q->vdop = vdop;

			} else if ( sz > 6 && line[3]=='G' && line[4]=='S' && line[5]=='V' ) {
				// @TODO - The first 2 bytes determine the GNSS constellation
				//   -- need to check this
				// --GSV - Satellites in View
				// Format $--GSV,Total Sentences 1-9, Sentence number 1-9, Total Sat in View XX, (when 0, nothing more than an extra 0 field)
				  // ... Sat ID Number (GPS 1 à 32) / 33-64 ( WAAS) / 65-88 Glonass,
				  // ... Elevation in degree 0 - 90 , Azimuth degree 000 - 359, SNR 00 -99dB (null if not tracking)
				  // Repetition of these 2 last line 4 times for different satellites.
				// ex :  $GPGSV,1,1,00,0*65
				// ex :  $GPGSV,2,1,07,24,60,119,33,37,34,155,,19,22,043,35,02,16,108,32,0*6E
				// Rq extra field at the end, SignalID 0=all channe, 1 G1 C/A ?
				uint8_t * pt = &line[5];
				uint16_t totSentences;
				uint16_t curSentence;
				uint16_t totSatInView;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_getDecimalField(pt,&totSentences) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_getDecimalField(pt,&curSentence) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				if ( curSentence == totSentences ) {
					driver->currentMessage |= NMEA_GSVL;
				} else {
					driver->currentMessage |= NMEA_GSV;
				}

				if ( nmea_goNextField(&pt) != GNSS_SUCCESS ) return GNSS_INVALIDFORMAT;
				if ( nmea_getDecimalField(pt,&totSatInView) == GNSS_INVALIDFORMAT ) return GNSS_INVALIDFORMAT;
				if ( totSatInView > 0 ) {
					data->satDetailsUpdated = 1;
					data->satInView = totSatInView;
					itsdk_time_is_UTC_s(&data->lastRefreshS);
					// get details on sats we have <= 4 sat data per line
					uint8_t pendingSat = (totSatInView - ((curSentence-1) * 4)) & 3;
					uint64_t now = itsdk_time_get_ms()/1000;
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

						#if ITSDK_DRIVERS_GNSS_WITHGPSSAT == __ENABLE && (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS) > 0
						if ( satId >= 1 && satId <= 32 ) {
							// GPS Sat
							data->sat_gps[satId-1].updateTime = (uint32_t)now;
							data->sat_gps[satId-1].elevation = elevation;
							data->sat_gps[satId-1].azimuth = azimuth;
							data->sat_gps[satId-1].signal = snr;
							if ( snr > data->sat_gps[satId-1].maxSignal ) data->sat_gps[satId-1].maxSignal = snr;
						}
						#endif
						#if ITSDK_DRIVERS_GNSS_WITHGLOSAT == __ENABLE && (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS) > 0
						if ( satId >= 65 && satId <= 88 ) {
							// GLONASS Sat
							data->sat_glonas[satId-65].updateTime= (uint32_t)now;
							data->sat_glonas[satId-65].elevation = elevation;
							data->sat_glonas[satId-65].azimuth = azimuth;
							data->sat_glonas[satId-65].signal = snr;
							if ( snr > data->sat_glonas[satId-65].maxSignal ) data->sat_glonas[satId-65].maxSignal = snr;
						}
						#endif
						#if ITSDK_DRIVERS_GNSS_WITHGALSAT == __ENABLE && (ITSDK_DRIVERS_GNSS_POSINFO & __GNSS_WITH_SAT_DETAILS) > 0
							#warning "GALILEO Support not yet implemented"
						#endif
					}
				}
			}  else if ( sz > 6 && line[3]=='V' && line[4]=='T' && line[5]=='G' ) {
				// @TODO to be decoded
				driver->currentMessage |= NMEA_VTG;
			} else {
				// Unsupported NMEA message
				//log_info("[UKN] %s \r\n",line);
			}
		} else if ( line[1] == 'P') {
			return GNSS_PROPRIETARY;
		} else {
			//log_info("[UNKNONWN] %s \r\n",line);
			#if (ITSDK_LOGGER_MODULE & __LOG_MOD_GNSS) > 0
				log_info("U");
			#endif

		}

	} else {
		//log_error(">> [%d] %s\r\n",ret, line);
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
		return GNSS_INVALIDFORMAT;
	}
	if ( itdt_convertHexChar2Int((char*)&line[chksumpos+1]) != chksum ) {
		//log_error("Chk error %02X %02X\r\n",(int)itdt_convertHexChar2Int((char*)&line[chksumpos+1]),chksum);
		return GNSS_CHECKSUMERROR;
	}
	return GNSS_SUCCESS;
}


/**
 * Add the checksum value to the end of the line
 * The size is the max size of the buffer
 */
gnss_ret_e nmea_addChecksum(uint8_t * line, uint16_t sz) {
	if ( line[0] != '$' ) return GNSS_INVALIDFORMAT;

	// search for "*"xx for checksum
	int chksumpos = 1;
	uint8_t chksum = 0;
	while ( chksumpos < sz && line[chksumpos] != '*' ) {
		chksum ^= line[chksumpos];
		chksumpos++;
	}
	if ( chksumpos >= sz-5 ) {
		return GNSS_TOOSMALL;
	}
	itdt_convertInt2HexChar(chksum,(char*)&line[chksumpos+1],BOOL_TRUE);
	line[chksumpos+3]='\r';
	line[chksumpos+4]='\n';
	line[chksumpos+5]='\0';
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
 * Get a filed composed of base 10 digit up to the next comma or end of line or '.' when we want a decimal part of
 * a rational number
 * return an error if non digit number are part of the content
 */
gnss_ret_e nmea_getDecimalField(uint8_t * line, uint16_t * number) {
	uint8_t sz = 0;
	uint16_t num;
	uint32_t _num = 0;
	while ( *line >= '0' && *line <= '9' ) {
		_num = 10*_num;
		_num += *line - '0';
		line++;
		sz++;
	}
	if ( *line == ',' || *line == '*' || *line == '.' ) {
		if ( _num > 0xFFFF ) {
			_num = 0xFFFF;
			num = (uint16_t)_num;
			*number = num;
			return GNSS_OVERFLOW;
		} else {
			num = (uint16_t)_num;
			*number = num;
			if ( sz == 0 ) return GNSS_EMPTYFIELD;
			return GNSS_SUCCESS;
		}
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
	uint16_t num;
	uint32_t _num = 0;
	while ( (*line >= '0' && *line <= '9') || *line == '.' ) {
		if (*line != '.') {
			_num = 10*_num;
			_num += *line - '0';
			sz++;
		}
		line++;
	}
	if ( *line == ',' || *line == '*' ) {
		if ( _num > 0xFFFF ) {
			_num = 0xFFFF;
			num = (uint16_t)_num;
			*number = num;
			return GNSS_OVERFLOW;
		} else {
			num = (uint16_t)_num;
			*number = num;
			if ( sz == 0 ) return GNSS_EMPTYFIELD;
			return GNSS_SUCCESS;
		}

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

	if ( datePt == NULL && (pTime->status & GNSS_TIME_DATE) > 0 ) {
		// It means we have a better source of date in the other messages
		// it's better to not update
		return GNSS_NOTUPDATED;
	}

	pTime->hours = 10*__NMEA_CONVERT_CHAR(timePt[0]) + __NMEA_CONVERT_CHAR(timePt[1]);
	pTime->minutes = 10*__NMEA_CONVERT_CHAR(timePt[2]) + __NMEA_CONVERT_CHAR(timePt[3]);
	pTime->seconds = 10*__NMEA_CONVERT_CHAR(timePt[4]) + __NMEA_CONVERT_CHAR(timePt[5]);
	pTime->status |= GNSS_TIME_TIME;

	if ( datePt != NULL ){
		pTime->day = 10*__NMEA_CONVERT_CHAR(datePt[0]) + __NMEA_CONVERT_CHAR(datePt[1]);
		pTime->month = 10*__NMEA_CONVERT_CHAR(datePt[2]) + __NMEA_CONVERT_CHAR(datePt[3]);
		pTime->year = 10*__NMEA_CONVERT_CHAR(datePt[4]) + __NMEA_CONVERT_CHAR(datePt[5]);
		pTime->status |= GNSS_TIME_DATE;

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
			pTime->status |= GNSS_TIME_EPOC;
		#else
			pTime->epoc = 0;
			pTime->status &= ~GNSS_TIME_EPOC;
		#endif

	}
	return GNSS_SUCCESS;
}


/**
 * Convert a Lat or Lng field format [d]ddmm.mmmmm into degree * 10_000_000
 * Value is negative when the direction is South or West
 */
gnss_ret_e nmea_getLatLngField(uint8_t * l, int32_t * degrees, uint8_t orientation) {

	int32_t degree = 0;
	int32_t minute = 0;
	uint8_t shift;
	if ( l[4] == '.' ) {
		shift = 0;
	} else if ( l[5] == '.' ) {
		shift = 1;
		degree+=1000000000*__NMEA_CONVERT_CHAR(l[0]);
	} else return GNSS_INVALIDFORMAT;
	degree+=100000000*__NMEA_CONVERT_CHAR(l[0+shift])
	       + 10000000*__NMEA_CONVERT_CHAR(l[1+shift]);
	minute =100000000*__NMEA_CONVERT_CHAR(l[2+shift])
		   + 10000000*__NMEA_CONVERT_CHAR(l[3+shift])
		   +  1000000*__NMEA_CONVERT_CHAR(l[5+shift])
	       +   100000*__NMEA_CONVERT_CHAR(l[6+shift])
	       +    10000*__NMEA_CONVERT_CHAR(l[7+shift])
		   +     1000*__NMEA_CONVERT_CHAR(l[8+shift]);
	degree+= minute/60;

	if ( orientation == 'S' || orientation == 'W' ) {
		*degrees = -degree;
	} else {
		*degrees = degree;
	}
	return GNSS_SUCCESS;

}



#endif // ITSDK_DRIVERS_WITH_GNSS_DRIVER
#endif // ITSDK_WITH_DRIVERS
