/* ==========================================================
 * dps422.h - Driver for Infineon Temp & Pressure I2C driver
 *           - Support - DPS422
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 06 jul 2020
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
 * Supports the I2C implementation of the Omron sensor
 * Omron sensor is pin to pin compatible with BME280
 *
 * ==========================================================
 */
#ifndef INC_DRIVERS_TEMPHYGROPRESSURE_INFINEON_DPS422_H_
#define INC_DRIVERS_TEMPHYGROPRESSURE_INFINEON_DPS422_H_


// ====================================================================
// API
// ====================================================================

typedef enum {
	DPS422_SUCCESS =0,
	DPS422_NOTFOUND,

	DPS422_FAILED

} drivers_dps422_ret_e;

typedef enum {
	DPS422_MODE_UNKNONW = 0,			// Mode unknonwn
	DPS422_MODE_ONESHOT_AVG4 = 1		// 1 shot measure with 4 averaging

										// Custom mode

} drivers_dps422_mode_e;


drivers_dps422_ret_e drivers_dps422_getSensors(
		int32_t  * temperature,			// Temp in m oC
		uint32_t * pressure  			// Pressure un Pa
);

drivers_dps422_ret_e drivers_dps422_setup(drivers_dps422_mode_e mode);

// ====================================================================
// REGISTERS
// ====================================================================

#define DRIVER_DPS422_PRESB2_I2CADR					0x00
#define DRIVER_DPS422_PRESB1_I2CADR					0x01
#define DRIVER_DPS422_PRESB0_I2CADR					0x02
#define DRIVER_DPS422_TEMPB2_I2CADR					0x03
#define DRIVER_DPS422_TEMPB1_I2CADR					0x04
#define DRIVER_DPS422_TEMPB0_I2CADR					0x05

#define DRIVER_DPS422_PSRCFG_I2CADR					0x06
#define DRIVER_DPS422_PSRCFG_RATE_MSK				0x70	// Pressure measurment rate
#define DRIVER_DPS422_PSRCFG_RATE_1HZ				0x00
#define DRIVER_DPS422_PSRCFG_RATE_2HZ				0x10
#define DRIVER_DPS422_PSRCFG_RATE_4HZ				0x20
#define DRIVER_DPS422_PSRCFG_RATE_8HZ				0x30
#define DRIVER_DPS422_PSRCFG_RATE_16HZ				0x40
#define DRIVER_DPS422_PSRCFG_RATE_32HZ				0x50
#define DRIVER_DPS422_PSRCFG_RATE_64HZ				0x60
#define DRIVER_DPS422_PSRCFG_RATE_128HZ				0x70
#define DRIVER_DPS422_PSRCFG_RES_MSK				0x07	// Pressure measurement resolution (samples & decimation
#define DRIVER_DPS422_PSRCFG_RES_256SMP				0x00
#define DRIVER_DPS422_PSRCFG_RES_512SMP				0x01
#define DRIVER_DPS422_PSRCFG_RES_1024SMP			0x02
#define DRIVER_DPS422_PSRCFG_RES_2048SMP			0x03
#define DRIVER_DPS422_PSRCFG_RES_4096SMP			0x04
#define DRIVER_DPS422_PSRCFG_RES_8192SMP			0x05
#define DRIVER_DPS422_PSRCFG_RES_16384SMP			0x06
#define DRIVER_DPS422_PSRCFG_RES_32768SMP			0x07

#define DRIVER_DPS422_TMPCFG_I2CADR					0x07
#define DRIVER_DPS422_TMPCFG_FORCE1BIT				0x80
#define DRIVER_DPS422_TMPCFG_RATE_MSK				0x70
#define DRIVER_DPS422_TMPCFG_RATE_1HZ				0x00
#define DRIVER_DPS422_TMPCFG_RATE_2HZ				0x10
#define DRIVER_DPS422_TMPCFG_RATE_4HZ				0x20
#define DRIVER_DPS422_TMPCFG_RATE_8HZ				0x30
#define DRIVER_DPS422_TMPCFG_RATE_16HZ				0x40
#define DRIVER_DPS422_TMPCFG_RATE_32HZ				0x50
#define DRIVER_DPS422_TMPCFG_RATE_64HZ				0x60
#define DRIVER_DPS422_TMPCFG_RES_MSK				0x07
#define DRIVER_DPS422_TMPCFG_RES_256SMP				0x00
#define DRIVER_DPS422_TMPCFG_RES_512SMP				0x01
#define DRIVER_DPS422_TMPCFG_RES_1024SMP			0x02
#define DRIVER_DPS422_TMPCFG_RES_2048SMP			0x03
#define DRIVER_DPS422_TMPCFG_RES_4096SMP			0x04
#define DRIVER_DPS422_TMPCFG_RES_8192SMP			0x05
#define DRIVER_DPS422_TMPCFG_RES_16384SMP			0x06
#define DRIVER_DPS422_TMPCFG_RES_32768SMP			0x07

#define DRIVER_DPS422_MEASCFG_I2CARD				0x08
#define DRIVER_DPS422_MEASCFG_INITCOMPLETE_MSK		0x80
#define DRIVER_DPS422_MEASCFG_INITCOMPLETE			0x80
#define DRIVER_DPS422_MEASCFG_CONTMODE_MSK			0x40
#define DRIVER_DPS422_MEASCFG_CONTMODE_ON			0x40
#define DRIVER_DPS422_MEASCFG_TMPDATRDY_MSK			0x20
#define DRIVER_DPS422_MEASCFG_TMPDATRDY				0x20
#define DRIVER_DPS422_MEASCFG_PRSDATRDY_MSK			0x20
#define DRIVER_DPS422_MEASCFG_PRSDATRDY				0x20
#define DRIVER_DPS422_MEASCFG_MEASCTRL_MSK			0x07
#define DRIVER_DPS422_MEASCFG_MEASCTRL_IDLE_1		0x00		// 1 shot
#define DRIVER_DPS422_MEASCFG_MEASCTRL_PRES_1		0x01
#define DRIVER_DPS422_MEASCFG_MEASCTRL_TEMP_1		0x02
#define DRIVER_DPS422_MEASCFG_MEASCTRL_PnT_1		0x03
#define DRIVER_DPS422_MEASCFG_MEASCTRL_IDLE_C		0x04		// continuous
#define DRIVER_DPS422_MEASCFG_MEASCTRL_PRES_C		0x05
#define DRIVER_DPS422_MEASCFG_MEASCTRL_TEMP_C		0x06
#define DRIVER_DPS422_MEASCFG_MEASCTRL_PnT_C		0x07

#define DRIVER_DPS422_CFGREG_I2CADR					0x09
#define DRIVER_DPS422_CFGREG_INTSEL_MSK				0xF0		// Interrupt on SDO pin
#define DRIVER_DPS422_CFGREG_INTSEL_NOINT			0x00
#define DRIVER_DPS422_CFGREG_INTSEL_PRESS_ON		0x10
#define DRIVER_DPS422_CFGREG_INTSEL_TEMP_ON			0x20
#define DRIVER_DPS422_CFGREG_INTSEL_PnT_ON			0x30
#define DRIVER_DPS422_CFGREG_INTSEL_FIFOWTM_ON		0x40
#define DRIVER_DPS422_CFGREG_INTSEL_FIFOFUL_ON		0x80
#define DRIVER_DPS422_CFGREG_INTPOL_MSK				0x08
#define DRIVER_DPS422_CFGREG_INTPOL_LOW				0x00
#define DRIVER_DPS422_CFGREG_INTPOL_HIGH			0x08
#define DRIVER_DPS422_CFGREG_FIFOMODE_MSK			0x04
#define DRIVER_DPS422_CFGREG_FIFOMODE_STREAM		0x00
#define DRIVER_DPS422_CFGREG_FIFOMODE_STOPONFULL	0x04
#define DRIVER_DPS422_CFGREG_FIFOSTATE_MSK			0x02
#define DRIVER_DPS422_CFGREG_FIFOSTATE_ENABLE		0x02
#define DRIVER_DPS422_CFGREG_FIFOSTATE_DISABLE		0x00
#define DRIVER_DPS422_CFGREG_SPIMODE_MSK			0x01
#define DRIVER_DPS422_CFGREG_SPIMODE_4WIRES			0x00
#define DRIVER_DPS422_CFGREG_SPIMODE_3WIRES			0x01

#define DRIVER_DPS422_INTSTS_I2CADR					0x0A		// flag are activ high
#define DRIVER_DPS422_INTSTS_FIFOINT_MSK			0x04
#define DRIVER_DPS422_INTSTS_TEMPINT_MSK			0x02
#define DRIVER_DPS422_INTSTS_PRESINT_MSK			0x01

#define DRIVER_DPS422_WMCFG_I2CADR					0x0B
#define DRIVER_DPS422_WMCFG_MSK						0x1F		// interrupt WTM will be generated when the FIFO reach this level of unread values

#define DRIVER_DPS422_FIFOSTS_I2CADR				0x0C
#define DRIVER_DPS422_FIFOSTS_LEVEL_MSK				0xFC
#define DRIVER_DPS422_FIFOSTS_LEVEL_SHIFT			2
#define DRIVER_DPS422_FIFOSTS_FULL_WTM_MSK			0x02		// flag set to 1 when the fifo is full of watermark
#define DRIVER_DPS422_FIFOSTS_EMPTY_MSK				0x01		// flag set to 1 when the fifo is empty

#define DRIVER_DPS422_RESET_I2CADR					0x0D
#define DRIVER_DPS422_RESET_FLUSHFIFO_MSK			0x80
#define DRIVER_DPS422_RESET_SOFTRESET_MSK			0x0F
#define DRIVER_DPS422_RESET_SOFTRESET_SIMPLE		0x08		// Reset config w/o eFuse refresh : 0,7ms
#define DRIVER_DPS422_RESET_SOFTRESET_FULL			0x09		// Reset config with eFuse refresh : 3ms

#define DRIVER_DPS422_PRODID_I2CADR					0x1D
#define DRIVER_DPS422_PRODID_REVISION_MSK			0xF0
#define DRIVER_DPS422_PRODID_PRODUCTID_MSK			0x0F
#define DRIVER_DPS422_PRODID_PRODUCTID_VALUE		0x0A


#define DRIVER_DPS422COEFF_TGAIN_I2CADR				0x20
#define DRIVER_DPS422COEFF_DVBE_I2CADR				0x21
#define DRIVER_DPS422COEFF_VBE_I2CADR				0x22
#define DRIVER_DPS422COEFF_REG1_I2ADR				0x26		// C00 19:12
#define DRIVER_DPS422COEFF_REG2_I2ADR				0x27		// C00 11:4
#define DRIVER_DPS422COEFF_REG3_I2ADR				0x28		// C00 3:0  | C10 19:16
#define DRIVER_DPS422COEFF_REG4_I2ADR				0x29		// C10 15:8
#define DRIVER_DPS422COEFF_REG5_I2ADR				0x2A		// C10 7:0
#define DRIVER_DPS422COEFF_REG6_I2ADR				0x2B		// C01 19:12
#define DRIVER_DPS422COEFF_REG7_I2ADR				0x2C		// C01 11:4
#define DRIVER_DPS422COEFF_REG8_I2ADR				0x2D		// C01 3:0 | C02 19:16
#define DRIVER_DPS422COEFF_REG9_I2ADR				0x2E		// C02 15:8
#define DRIVER_DPS422COEFF_REG10_I2ADR				0x2F		// C02 7:0
#define DRIVER_DPS422COEFF_REG11_I2ADR				0x30		// 0 | C20 14:8
#define DRIVER_DPS422COEFF_REG12_I2ADR				0x31		// C20 7:0
#define DRIVER_DPS422COEFF_REG13_I2ADR				0x32		// 0000 | C30 11:8
#define DRIVER_DPS422COEFF_REG14_I2ADR				0x33		// C30 7:0
#define DRIVER_DPS422COEFF_REG15_I2ADR				0x34		// C11 16:9
#define DRIVER_DPS422COEFF_REG16_I2ADR				0x35		// C11 8:1
#define DRIVER_DPS422COEFF_REG17_I2ADR				0x36		// C11 0 | C12 16:10
#define DRIVER_DPS422COEFF_REG18_I2ADR				0x37		// C12 9:2
#define DRIVER_DPS422COEFF_REG19_I2ADR				0x38		// C12 1:0 | C21 13:8
#define DRIVER_DPS422COEFF_REG20_I2ADR				0x39		// C21 7:0



typedef struct {
	drivers_dps422_mode_e 		mode;		// Setup mode
	#if ITSDK_DRIVERS_DPS422_SAVERAM == __ENABLE
	int8_t 	tgain;
	int8_t 	dVbe;
	int16_t Vbe;
	int32_t c00;
	int32_t c01;
	int32_t c02;
	int32_t c10;
	int32_t c11;
	int32_t c12;
	int16_t c20;
	int16_t c21;
	int16_t c30;
	#endif
} drivers_dps422_conf_t;


#endif /* INC_DRIVERS_TEMPHYGROPRESSURE_INFINEON_DPS422_H_ */
