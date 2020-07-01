/* ==========================================================
 * o_2smpb.c - Omron 2SMPB-02B Temp / Pressure I2C driver
 *           - Support - 2SMPB-02B
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 31 May 2020
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2020 Disk91
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
#ifndef INC_DRIVERS_TEMPHYGROPRESSURE_OMRON_2SMPB_02B_O_2SMPB_H_
#define INC_DRIVERS_TEMPHYGROPRESSURE_OMRON_2SMPB_02B_O_2SMPB_H_


// ====================================================================
// API
// ====================================================================

typedef enum {
	O2SMPB_SUCCESS =0,
	O2SMPB_NOTFOUND,

	O2SMPB_FAILED

} drivers_o2smpb_ret_e;

typedef enum {
	O2SMPB_MODE_UNKNONW = 0,			// Mode unknonwn
	O2SMPB_MODE_ONESHOT_AVG4 = 1		// 1 shot measure with 4 averaging

										// Custom mode

} drivers_o2smpb_mode_e;


drivers_o2smpb_ret_e drivers_o2smpb_getSensors(
		int32_t  * temperature,			// Temp in m oC
		uint32_t * pressure  			// Pressure un Pa
);

drivers_o2smpb_ret_e drivers_o2smpb_setup(drivers_o2smpb_mode_e mode);

// ====================================================================
// REGISTERS
// ====================================================================

#define DRIVER_O2SMPB_TEMP_TXD0_I2CADR				0xFC			// temp Data bit [8:1]
#define DRIVER_O2SMPB_TEMP_TXD1_I2CADR				0xFB			// temp Data bit [16:9]
#define DRIVER_O2SMPB_TEMP_TXD2_I2CADR				0xFA			// temp Data bit [14:17]

#define DRIVER_O2SMPB_PRES_TXD0_I2CADR				0xF9			// pressure Data bit [8:1]
#define DRIVER_O2SMPB_PRES_TXD1_I2CADR				0xF8			// pressure Data bit [16:9]
#define DRIVER_O2SMPB_PRES_TXD2_I2CADR				0xF7			// pressure Data bit [14:17]

#define DRIVER_O2SMPB_IOSETUP_I2CADR				0xF5			// Io Setup
#define DRIVER_O2SMPB_IOSETUP_TSTANDBY_MSK			0xE0			//  T_standby
#define DRIVER_O2SMPB_IOSETUP_TSTANDBY_1MS			0x00			//		1ms standby time   (*) default
#define DRIVER_O2SMPB_IOSETUP_TSTANDBY_5MS			0x20			//		5ms standby time
#define DRIVER_O2SMPB_IOSETUP_TSTANDBY_50MS			0x40			//	   50ms standby time
#define DRIVER_O2SMPB_IOSETUP_TSTANDBY_250MS		0x60			//	  250ms standby time
#define DRIVER_O2SMPB_IOSETUP_TSTANDBY_500MS		0x80			//	  500ms standby time
#define DRIVER_O2SMPB_IOSETUP_TSTANDBY_1S			0xA0			//		1s  standby time
#define DRIVER_O2SMPB_IOSETUP_TSTANDBY_2S			0xC0			//		2s  standby time
#define DRIVER_O2SMPB_IOSETUP_TSTANDBY_4S			0xE0			//		4s  standby time
#define DRIVER_O2SMPB_IOSETUP_SPIOTYPE_MSK			0x04			//  SPI output type for SDI
#define DRIVER_O2SMPB_IOSETUP_SPIOTYPE_LOHIGHZ		0x00			//    SPI output type Low / Highz (*)
#define DRIVER_O2SMPB_IOSETUP_SPIOTYPE_LOHIGH		0x04			//    SPI output type Low / High
#define DRIVER_O2SMPB_IOSETUP_SPI3W_MSK				0x01			//  SPI mode 3 or 4 wires
#define DRIVER_O2SMPB_IOSETUP_SPI3W_3WIRES			0x01			//    SPI mode 3 wires
#define DRIVER_O2SMPB_IOSETUP_SPI3W_4WIRES			0x00			//    SPI mode 4 wires (*)


#define DRIVER_O2SMPB_CTRLMEAS_I2CADR				0xF4			//
#define DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_MSK			0xE0			//  Temperature averaging times
#define DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_SKIP			0x00			//  	None (*)
#define DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_1			0x20			//  	 1
#define DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_2			0x40			//  	 2
#define DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_4			0x60			//  	 4
#define DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_8			0x80			//  	 8
#define DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_16			0xA0			//  	16
#define DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_32			0xC0			//  	32
#define DRIVER_O2SMPB_CTRLMEAS_TEMPAVG_64			0xE0			//  	64
#define DRIVER_O2SMPB_CTRLMEAS_PRESAVG_MSK			0x1C			//  Pressure averaging times
#define DRIVER_O2SMPB_CTRLMEAS_PRESAVG_SKIP			0x00			//  	None (*)
#define DRIVER_O2SMPB_CTRLMEAS_PRESAVG_1			0x04			//  	 1
#define DRIVER_O2SMPB_CTRLMEAS_PRESAVG_2			0x08			//  	 2
#define DRIVER_O2SMPB_CTRLMEAS_PRESAVG_4			0x0C			//  	 4
#define DRIVER_O2SMPB_CTRLMEAS_PRESAVG_8			0x10			//  	 8
#define DRIVER_O2SMPB_CTRLMEAS_PRESAVG_16			0x14			//  	16
#define DRIVER_O2SMPB_CTRLMEAS_PRESAVG_32			0x18			//  	32
#define DRIVER_O2SMPB_CTRLMEAS_PRESAVG_64			0x1C			//  	64
#define DRIVER_O2SMPB_CTRLMEAS_POWERMD_MSK			0x03			//  Power mode
#define DRIVER_O2SMPB_CTRLMEAS_POWERMD_SLEEP		0x00			//  	Sleep Mode (*)
#define DRIVER_O2SMPB_CTRLMEAS_POWERMD_FORCED		0x01			//  	Forced Mode - just make 1 measure (can be a averaged measure)
#define DRIVER_O2SMPB_CTRLMEAS_POWERMD_NORMAL		0x03			//  	Normal Mode - measure on every t_standby


#define DRIVER_O2SMPB_DEVSTAT_I2CADR				0xF3			// Device Stat
#define DRIVER_O2SMPB_DEVSTAT_MEASURE_MSK			0x08			//   Status of measurement
#define DRIVER_O2SMPB_DEVSTAT_MEASURE_FINISHED		0x00			//     The measurement is completed
#define DRIVER_O2SMPB_DEVSTAT_MEASURE_RUNNING		0x08			//     The measurement is in progress
#define DRIVER_O2SMPB_DEVSTAT_OTPSTAT_MSK			0x01			//   OTP Status data access
#define DRIVER_O2SMPB_DEVSTAT_OTPSTAT_NOACCESS		0x00			//     No running access on OTP Data
#define DRIVER_O2SMPB_DEVSTAT_OTPSTAT_ACCESSING		0x01			//     OTP Data is under access



#define DRIVER_O2SMPB_I2CSET_I2CADR					0xF2			// I2C Set
#define DRIVER_O2SMPB_I2CSET_MSTRCODE_MSK			0x07			//   Master code at I2C High Speed mode
#define DRIVER_O2SMPB_I2CSET_MSTRCODE_08			0x00			//   	Master code 0x08
#define DRIVER_O2SMPB_I2CSET_MSTRCODE_09			0x01			//   	Master code 0x09 (*)
#define DRIVER_O2SMPB_I2CSET_MSTRCODE_0A			0x02			//   	Master code 0x0A
#define DRIVER_O2SMPB_I2CSET_MSTRCODE_0B			0x03			//   	Master code 0x0B
#define DRIVER_O2SMPB_I2CSET_MSTRCODE_0C			0x04			//   	Master code 0x0C
#define DRIVER_O2SMPB_I2CSET_MSTRCODE_0D			0x05			//   	Master code 0x0D
#define DRIVER_O2SMPB_I2CSET_MSTRCODE_0E			0x06			//   	Master code 0x0E
#define DRIVER_O2SMPB_I2CSET_MSTRCODE_0F			0x07			//   	Master code 0x0F


#define DRIVER_O2SMPB_IIRFILTER_I2CADR				0xF1			// IIR Filter coef
#define DRIVER_O2SMPB_IIRFILTER_COEFE_MSK			0x07			//   IIR filter coefficients
#define DRIVER_O2SMPB_IIRFILTER_COEFE_OFF			0x00			//   	IIR filter off (*)
#define DRIVER_O2SMPB_IIRFILTER_COEFE_N2			0x01			//   	IIR filter N = 2
#define DRIVER_O2SMPB_IIRFILTER_COEFE_N4			0x02			//   	IIR filter N = 4
#define DRIVER_O2SMPB_IIRFILTER_COEFE_N8			0x03			//   	IIR filter N = 8
#define DRIVER_O2SMPB_IIRFILTER_COEFE_N16			0x04			//   	IIR filter N = 16
#define DRIVER_O2SMPB_IIRFILTER_COEFE_N32			0x05			//   	IIR filter N = 32


#define DRIVER_O2SMPB_RESET_I2CADR					0xE0			// Reset command => write 0xE6 to reset the device
#define DRIVER_O2SMPB_RESET_CMD						0xE6

#define DRIVER_O2SMPB_CHIPID_I2CADR					0xD1			// Return 0x5C for device identification
#define DRIVER_O2SMPB_CHIPID_VALUE					0x5C

#define DRIVER_O2SMPB_COEPTAT32_I2CADR				0xB4			// 2nd order correction coef PTAT aa[7:0]
#define DRIVER_O2SMPB_COEPTAT31_I2CADR				0xB3			// 2nd order correction coef PTAT aa[15:8]

#define DRIVER_O2SMPB_COEPTAT22_I2CADR				0xB2			// correction coef PTAT ba[7:0]
#define DRIVER_O2SMPB_COEPTAT21_I2CADR				0xB1			// correction coef PTAT ba[15:8]

#define DRIVER_O2SMPB_COEPTAT13_I2CADR				0xAF			// Offset Ptat PTAT ca[7:0]
#define DRIVER_O2SMPB_COEPTAT12_I2CADR				0xAE			// Offset PTAT ca[15:8]
#define DRIVER_O2SMPB_COEPTAT11_I2CADR				0xAD			// Offset PTAT ca[23:16]

#define DRIVER_O2SMPB_COETEMP32_I2CADR				0xAC			// 2nd order correction coef TEMP at[7:0]
#define DRIVER_O2SMPB_COETEMP31_I2CADR				0xAB			// 2nd order correction coef TEMP at[15:8]

#define DRIVER_O2SMPB_COETEMP22_I2CADR				0xAA			// 1st order correction coef TEMP bt[7:0]
#define DRIVER_O2SMPB_COETEMP21_I2CADR				0xA9			// 1st order correction coef TEMP bt[15:8]

#define DRIVER_O2SMPB_COETEMP12_I2CADR				0xA8			// Corr coef TEMP ct[7:0]
#define DRIVER_O2SMPB_COETEMP11_I2CADR				0xA7			// Corr coef TEMP ct[15:8]


#define DRIVER_O2SMPB_COEPR32_I2CARD				0xA6			// 2nd order correction coef PRESSURE ap[7:0]
#define DRIVER_O2SMPB_COEPR31_I2CARD				0xA5			// 2nd order correction coef PRESSURE ap[15:8]

#define DRIVER_O2SMPB_COEPR22_I2CARD				0xA4			// 1st order correction coef PRESSURE bp[7:0]
#define DRIVER_O2SMPB_COEPR21_I2CARD				0xA3			// 1st order correction coef PRESSURE bp[15:8]

#define DRIVER_O2SMPB_COEPR13_I2CARD				0xA2			// Offset PRESSURE cp[7:0]
#define DRIVER_O2SMPB_COEPR12_I2CARD				0xA1			// Offset PRESSURE cp[15:8]
#define DRIVER_O2SMPB_COEPR11_I2CARD				0xA0			// Offset PRESSURE cp[23:16]



typedef struct {
	drivers_o2smpb_mode_e 		mode;		// Setup mode
	int16_t						aa;			// calibration PTAT
	int16_t						ba;
	int32_t						ca;

	int16_t						at;			// calibration TEMP
	int16_t						bt;
	int16_t						ct;

	int16_t						ap;			// calibration PRESSURE
	int16_t						bp;
	int32_t						cp;
} drivers_o2smpb_conf_t;


#endif /* INC_DRIVERS_TEMPHYGROPRESSURE_OMRON_2SMPB_02B_O_2SMPB_H_ */
