/* ==========================================================
 * bme280.c - Bosh BME280 Temp / Hygro / Pressure I2C-SPI sensor
 *            Bosh BMP280 Temp / Pressure I2C-SPI sensor
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 fevr. 2019
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
 * Supports the I2C implementation of the Bosh sensor
 *
 * ==========================================================
 */
#ifndef INC_IT_SDK_DRIVER_BME280_H_
#define INC_IT_SDK_DRIVER_BME280_H_




// ====================================================================
// API
// ====================================================================

typedef enum {
	BME280_SUCCESS = 0,
	BMP280_SUCCESS,
	BME280_NOTFOUND,

	BME280_FAILED

} drivers_bme280_ret_e;


typedef enum {
	BME280_MODE_WEATHER_MONITORING = 0,		// See Datasheet
	BME280_MODE_HUMIDITY_SENSING,
	BME280_MODE_INDOOR_NAVIGATION,
											// Custom mode

} drivers_bme280_mode_e;

typedef enum {
	BME280 = 0,								// Driver type
	BMP280,
} drivers_bme280_type_e;

typedef struct {
	drivers_bme280_ret_e 		mode;		// Setup mode
	drivers_bme280_type_e		type;
// TODO - make a memory optimized version where the calibration values are not stored into memory
#warning TODO Optimisation memoire
	uint16_t					t1;			// calibration
	int16_t						t2;
	int16_t						t3;
	uint16_t					p1;
	int16_t						p2;
	int16_t						p3;
	int16_t						p4;
	int16_t						p5;
	int16_t						p6;
	int16_t						p7;
	int16_t						p8;
	int16_t						p9;
	uint8_t						h1;
	int16_t						h2;
	uint8_t						h3;
	int16_t						h4;
	int16_t						h5;
	int8_t						h6;
	int32_t 					t_fine;		// Intermediate Temp computation used by other sensor computation
} drivers_bme280_conf_t;


/**
 * Setup the sensor is a given mode after verifying the presence
 * Actually the implemented mode are corresponding to the documentation
 * Custom mode can ba added.
 */
drivers_bme280_ret_e drivers_bme280_setup(drivers_bme280_mode_e mode);

/**
 * Depends on mode, get the last sensor values or request a new value and get it
 * Get the sensors value.
 * Temperature is in m°C
 * Humidity is in m%RH
 * Pressure is in Pa
 */
drivers_bme280_ret_e drivers_bme280_getSensors(
		int32_t  * temperature,			// Temp in m�C
		uint32_t * pressure,			// Pressure un Pa
		uint32_t * humidity				// Humidity in m%RH
);

// ====================================================================
// REGISTERS
// ====================================================================

#define DRIVER_BME280_DEVICE_ADR				0x76			// Device default address, this can be
																// affected by the SDO Configuration
																// Same address for BMP280

#define DRIVER_BME280_REG_ID_ADR				0xD0			// RD - Device identification
#define DRIVER_BME280_REG_ID_VALUE				0x60			//   This value confirms the device is BME280
#define DRIVER_BMP280_REG_ID_VALUE				0x58			//   This value confirms the device is BMP280
#define DRIVER_BME280_REG_RESET_ADR				0xE0			// WR - Reset register
#define DRIVER_BME280_REG_RESET_VALUE			0xB6			//   Writting the value reset the device

#define DRIVER_BME280_REG_CTRLHUM_ADR			0xF2			// RD/WR - Humidity control register
#define DRIVER_BME280_REG_CTRLHUM_OSRD_MASK		0x07			//  Oversampling humidity data
#define DRIVER_BME280_REG_CTRLHUM_OSRD_SKIP		0x00			//    skipped
#define DRIVER_BME280_REG_CTRLHUM_OSRD_X1		0x01			//	  x1
#define DRIVER_BME280_REG_CTRLHUM_OSRD_X2		0x02			//	  x2
#define DRIVER_BME280_REG_CTRLHUM_OSRD_X4		0x03			//	  x4
#define DRIVER_BME280_REG_CTRLHUM_OSRD_X8		0x04			//	  x8
#define DRIVER_BME280_REG_CTRLHUM_OSRD_X16		0x05			//	  x16

#define DRIVER_BME280_REG_STATUS_ADR			0xF3			// RD - Status register
#define DRIVER_BME280_REG_STATUS_MEASURING_MASK 0x08			//  set when the device is running a conversion
#define DRIVER_BME280_REG_STATUS_UPDATING_MASK  0x01			//  set during NVM data transfer is in progress

#define DRIVER_BME280_REG_CTRLMEAS_ADR			0xF4			// RD/WR - Pressure & Temperature control register
#define DRIVER_BME280_REG_CTRLMEAS_OSRST_MASK	0xE0			//  Oversampling temperature data
#define DRIVER_BME280_REG_CTRLMEAS_OSRST_SKIP   0x00			//   skipped
#define DRIVER_BME280_REG_CTRLMEAS_OSRST_X1		0x20			//   x1
#define DRIVER_BME280_REG_CTRLMEAS_OSRST_X2		0x40			//   x2
#define DRIVER_BME280_REG_CTRLMEAS_OSRST_X4		0x50			//   x4
#define DRIVER_BME280_REG_CTRLMEAS_OSRST_X8		0x80			//   x8
#define DRIVER_BME280_REG_CTRLMEAS_OSRST_X16	0xA0			//   x16
#define DRIVER_BME280_REG_CTRLMEAS_OSRSP_MASK	0x1C			//  Oversampling pressure data
#define DRIVER_BME280_REG_CTRLMEAS_OSRSP_SKIP   0x00			//   skipped
#define DRIVER_BME280_REG_CTRLMEAS_OSRSP_X1		0x04			//   x1
#define DRIVER_BME280_REG_CTRLMEAS_OSRSP_X2		0x08			//   x2
#define DRIVER_BME280_REG_CTRLMEAS_OSRSP_X4		0x0C			//   x4
#define DRIVER_BME280_REG_CTRLMEAS_OSRSP_X8		0x10			//   x8
#define DRIVER_BME280_REG_CTRLMEAS_OSRSP_X16	0x14			//   x16
#define DRIVER_BME280_REG_CTRLMEAS_MODE_MASK	0x03			//  Sensor mode
#define DRIVER_BME280_REG_CTRLMEAS_MODE_SLEEP	0x00			//   Sleep Mode - do nothing
#define DRIVER_BME280_REG_CTRLMEAS_MODE_FORCED	0x01			//   Force Mode - fire 1 measure
#define DRIVER_BME280_REG_CTRLMEAS_MODE_NORMAL	0x03			//   Normal Mode - repeat measure every xx ms


#define DRIVER_BME280_REG_CONFIG_ADR			0xF5			// RW/WR - configuration register
#define DRIVER_BME280_REG_CONFIG_TSB_MASK		0xE0			//  Set Tstandby in normal mode (time between measure)
#define DRIVER_BME280_REG_CONFIG_TSB_0_5MS		0x00			//   0.5ms
#define DRIVER_BME280_REG_CONFIG_TSB_62_5MS		0x20			//   62.5ms
#define DRIVER_BME280_REG_CONFIG_TSB_125MS		0x40			//   125ms
#define DRIVER_BME280_REG_CONFIG_TSB_250MS		0x60			//   250ms
#define DRIVER_BME280_REG_CONFIG_TSB_500MS		0x80			//   500ms
#define DRIVER_BME280_REG_CONFIG_TSB_1000MS		0xA0			//   1s
#define DRIVER_BME280_REG_CONFIG_TSB_10MS		0xC0			//   10ms
#define DRIVER_BME280_REG_CONFIG_TSB_20MS		0xE0			//   20ms
#define DRIVER_BME280_REG_CONFIG_FILTER_MASK	0x1C			//  Time constant for IIR filter
#define DRIVER_BME280_REG_CONFIG_FILTER_OFF		0x00			//	 Off
#define DRIVER_BME280_REG_CONFIG_FILTER_2		0x04			//	 2
#define DRIVER_BME280_REG_CONFIG_FILTER_4		0x08			//	 4
#define DRIVER_BME280_REG_CONFIG_FILTER_8		0x0C			//	 8
#define DRIVER_BME280_REG_CONFIG_FILTER_16		0x10			//	 16

#define DRIVER_BME280_REG_PRESS_MSB_ADR			0xF7			// Pressure value MSB
#define DRIVER_BME280_REG_PRESS_LSB_ADR			0xF8			// Pressure value LSB
#define DRIVER_BME280_REG_PRESS_XLSB_ADR		0xF9			// Pressure XLSB Part. Depends on resolution
#define DRIVER_BME280_REG_PRESS_XLSB_MSK		0xF0

#define DRIVER_BME280_REG_TEMP_MSB_ADR			0xFA			// Temperature value MSB
#define DRIVER_BME280_REG_TEMP_LSB_ADR			0xFB			// Temperature value LSB
#define DRIVER_BME280_REG_TEMP_XLSB_ADR			0xFC			// Temperature value XLSB
#define DRIVER_BME280_REG_TEMP_XLSB_MSK			0xF0			// Temperature value XLSB

#define DRIVER_BME280_REG_HUM_MSB_ADR			0xFD			// Humidity value MSB
#define DRIVER_BME280_REG_HUM_LSB_ADR			0xFE			// Humidity value LSB

#define DRIVER_BME280_DIG_T1_MSB				0x89			// Compensation parameter
#define DRIVER_BME280_DIG_T1_LSB				0x88
#define DRIVER_BME280_DIG_T2_MSB				0x8B
#define DRIVER_BME280_DIG_T2_LSB				0x8A
#define DRIVER_BME280_DIG_T3_MSB				0x8D
#define DRIVER_BME280_DIG_T3_LSB				0x8C
#define DRIVER_BME280_DIG_P1_MSB				0x8F
#define DRIVER_BME280_DIG_P1_LSB				0x8E
#define DRIVER_BME280_DIG_P2_MSB				0x91
#define DRIVER_BME280_DIG_P2_LSB				0x90
#define DRIVER_BME280_DIG_P3_MSB				0x93
#define DRIVER_BME280_DIG_P3_LSB				0x92
#define DRIVER_BME280_DIG_P4_MSB				0x95
#define DRIVER_BME280_DIG_P4_LSB				0x94
#define DRIVER_BME280_DIG_P5_MSB				0x97
#define DRIVER_BME280_DIG_P5_LSB				0x96
#define DRIVER_BME280_DIG_P6_MSB				0x99
#define DRIVER_BME280_DIG_P6_LSB				0x98
#define DRIVER_BME280_DIG_P7_MSB				0x9B
#define DRIVER_BME280_DIG_P7_LSB				0x9A
#define DRIVER_BME280_DIG_P8_MSB				0x9D
#define DRIVER_BME280_DIG_P8_LSB				0x9C
#define DRIVER_BME280_DIG_P9_MSB				0x9F
#define DRIVER_BME280_DIG_P9_LSB				0x9E
#define DRIVER_BME280_DIG_H1					0xA1
#define DRIVER_BME280_DIG_H2_MSB				0xE2
#define DRIVER_BME280_DIG_H2_LSB				0xE1
#define DRIVER_BME280_DIG_H3					0xE3
#define DRIVER_BME280_DIG_H4_MSB				0xE4
#define DRIVER_BME280_DIG_H4_LSB				0xE5
#define DRIVER_BME280_DIG_H4_LSB_MASK			0x0F
#define DRIVER_BME280_DIG_H5_MSB				0xE6
#define DRIVER_BME280_DIG_H5_LSB				0xE5
#define DRIVER_BME280_DIG_H5_LSB_MASK			0xF0
#define DRIVER_BME280_DIG_H6					0xE7


#endif



