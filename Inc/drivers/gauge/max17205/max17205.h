/* ==========================================================
 * max17205.h -  Maxim 17205 - Gauge 3 cells
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 24 f�vr. 2019
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

#ifndef DRIVERS_GAUGE_MAX17205_MAX17205_H_
#define DRIVERS_GAUGE_MAX17205_MAX17205_H_

// ====================================================================
// API
// ====================================================================

typedef enum {
	MAX17205_MODE_DEFAULT = 0,
	MAX17205_MODE_3CELLS_INT_TEMP = 1,


} drivers_max17205_mode_e;


typedef enum __attribute__ ((__packed__)) {
	MAX17205_TYPE_SINGLE_CELL = 1,
	MAX17205_TYPE_MULTI_CELL = 5,
} drivers_max17205_type_e;

typedef enum {
	MAX17205_CELL1 = 0,
	MAX17205_CELL2,
	MAX17205_CELL3,
	MAX17205_CELL4,
	MAX17205_CELLX,
	MAX17205_VBAT,
} drivers_max17205_cell_select_e;

typedef struct {
	drivers_max17205_mode_e 		mode;				// Setup mode
	drivers_max17205_type_e			devType;			// 172X1 or 172X5
	uint8_t							initialized:3;
	uint16_t						lastCapa;			// last read capacity mAh
	uint32_t						totalCapa;			// since the beginning what is the capa mAh
} drivers_max17205_conf_t;

typedef enum {
	MAX17205_SUCCESS=0,
	MAX17205_UNDERVOLT=1,
	MAX17205_NOTFOUND=2,

	MAX17205_FAILED=7

} drivers_max17205_ret_e;

drivers_max17205_ret_e drivers_max17205_setup(drivers_max17205_mode_e mode);
drivers_max17205_ret_e drivers_max17205_getTemperature(int32_t * mTemp);
drivers_max17205_ret_e drivers_max17205_getVoltage(drivers_max17205_cell_select_e cell, uint16_t * mVolt);
drivers_max17205_ret_e drivers_max17205_getCurrent(int32_t * uAmp);
drivers_max17205_ret_e drivers_max17205_getCapacity(uint16_t * mah);
drivers_max17205_ret_e drivers_max17205_getCoulomb(uint32_t * coulomb);
drivers_max17205_ret_e drivers_max17205_isReady();

drivers_max17205_ret_e drivers_max17205_getRemainingNVMUpdates(uint16_t * upd);

// ====================================================================
// Registers
// ====================================================================

#define ITSDK_DRIVERS_MAX17205_ADDRESS_000_0FF	0x36			// Non shifted address for memory 0x000 -> 0x0FF
#define ITSDK_DRIVERS_MAX17205_ADDRESS_100_1FF	0x0B			// Non shifted address for memory 0x100	-> 0x17F

#define ITSDK_DRIVERS_MAX17205_UNDERVOLTAGE		5000			// Limit in mV for MAX17205 to work properly


#define ITSDK_DRIVERS_MAX17205_REG_DEVNAME_ADR			0x21	// Device type identification
#define ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MSK			0x000F  // Device type part
#define ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MAX172X1		0x01	//  17201 / 17211 chip (single Cell)
#define ITSDK_DRIVERS_MAX17205_REG_DEVNAME_MAX172X5		0x05	//  17205 / 17215 chip (multi Cell)

#define ITSDK_DRIVERS_MAX17205_REG_TEMP_ADR				0x08	// Temperature

#define ITSDK_DRIVERS_MAX17205_REG_CELL1_VOLT_ADR		0xD8	// CELL1 Voltage
#define ITSDK_DRIVERS_MAX17205_REG_CELL2_VOLT_ADR		0xD7	// CELL2 Voltage
#define ITSDK_DRIVERS_MAX17205_REG_CELL3_VOLT_ADR		0xD6	// CELL3 Voltage
#define ITSDK_DRIVERS_MAX17205_REG_CELL4_VOLT_ADR		0xD5	// CELL4 Voltage
#define ITSDK_DRIVERS_MAX17205_REG_CELLX_VOLT_ADR		0xD9	// CELLX Voltage
#define ITSDK_DRIVERS_MAX17205_REG_VBAT_VOLT_ADR		0xDA	// VBAT Voltage
#define ITSDK_DRIVERS_MAX17205_REG_CURRENT_ADR			0x0A	// Current
#define ITSDK_DRIVERS_MAX17205_REG_QH_ADR				0x4D	// Coulomb

#define ITSDK_DRIVERS_MAX17205_REG_COMMAND_ADR			0x60	// Command register
#define ITSDK_DRIVERS_MAX17205_CMD_RECALL				0xE2FA	// Command to recall NV memory
#define ITSDK_DRIVERS_MAX17205_CMD_BLOCKCPY				0xE904	// Command to copy block into NV Memory

#define ITSDK_DRIVERS_MAX17205_REG_COMMSTAT_ADR			0x61	// CommStat register
#define ITSDK_DRIVERS_MAX17205_REG_COMMSTAT_NVERR_MSK	0x0004	// Indicating an error and potentialy something processing
#define ITSDK_DRIVERS_MAX17205_REG_COMMSTAT_NVBUSY_MSK	0x0002	// Indicating the NV Memory is busy


#define ITSDK_DRIVERS_MAX17205_REG_CONFIG2_ADR			0xBB	// Config 2 register

#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_ADR			0x1B5	// Config 2 register
#define ITSDK_DRIVERS_MAX17205_REG_BNPACKCFG_ADR		0x0BD	// Config 2 Persistant
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_NCELL_MSK	0x000F	// Number of cells
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_NCELL_SHIFT	0

#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_BALCFG_MSK	 	0x00E0	// Cells balancing
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_BALCFG_SHIFT 	5		//  Threshold = 1.25 * 2 ^ BALCFG
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_BALCFG_DISABLE	0

#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CELLX_MSK	 	0x0100	// Enable CELLs Channel
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CELLX_SHIFT 	8
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CELLX_DISABLE	0x0000
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CELLX_ENABLE	0x0100
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_VBAT_MSK	 	0x0200	// Enable VBAT Channel
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_VBAT_SHIFT 		9
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_VBAT_DISABLE	0x0000
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_VBAT_ENABLE		0x0200
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CHEN_MSK	 	0x0400	// Enable CELL1/CELL2/VBAT Channel
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CHEN_SHIFT 		10		//  override the previous bits
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CHEN_DISABLE	0x0000
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_CHEN_ENABLE		0x0400

#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_TDEN_MSK	 	0x0800	// Enable Die Temperature
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_TDEN_SHIFT 		11
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_TDEN_DISABLE	0x0000
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_TDEN_ENABLE		0x0800
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_A1EN_MSK	 	0x1000	// Enable A1 Temperature
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_A1EN_SHIFT 		12
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_A1EN_DISABLE	0x0000
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_A1EN_ENABLE		0x1000
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_A2EN_MSK	 	0x2000	// Enable A2 Temperature
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_A2EN_SHIFT 		13
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_A2EN_DISABLE	0x0000
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_A2EN_ENABLE		0x2000
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_FGT_MSK		 	0x8000	// Select the Temperature source
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_FGT_SHIFT 		15
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_FGT_DISABLE		0x0000
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_FGT_ENABLE		0x8000

#define ITSDK_DRIVERS_MAX17205_REG_NHIBCFG					0x01B4
#define ITSDK_DRIVERS_MAX17205_REG_NHIBCFG_ENHIB_MSK		0x8000	// Enable Hibernate mode switch (1) or disable it (0)

#define ITSDK_DRIVERS_MAX17205_REG_NVCFG					0x01B8	// Manages NVM backup and register RAM locations
#define ITSDK_DRIVERS_MAX17205_REG_NVCFG_DCAP_MSK			0x0010	// Enable DesignCap restore

#define ITSDK_DRIVERS_MAX17205_REG_NDCAP					0x01B3	// expected capacity (non volatile)
#define ITSDK_DRIVERS_MAX17205_REG_DCAP						0x0018	// expected capacity (restored after reset)

typedef enum {												//  Temp Source    RegisterToRead
	MAX17205_REG_NPACKCFG_TEMP_INTERNAL_DIETEMP = 0x0800,	//   Internal        DieTemp (135h)
	MAX17205_REG_NPACKCFG_TEMP_AIN1_TEMP1 = 0x9000,			//   AIN1			 Temp1 (134h)
	MAX17205_REG_NPACKCFG_TEMP_AIN2_TEMP2 = 0x2000,			// 	 AIN2			 Temp2 (13Bh)
	MAX17205_REG_NPACKCFG_TEMP_TEMP1_TEMP2 = 0x3000  		//   Temp1 & Temp2
															// CF P60 of datasheet, different configuration can
															// make the same result
}drivers_max17205_temp_conf_e;
#define ITSDK_DRIVERS_MAX17205_REG_NPACKCFG_TEMP_MSK	 	0xB800	// Mask to clean the Temperature source selection


#define ITSDK_DRIVERS_MAX17205_REG_NRSENSE					0x1CF	//  setup the Rsense value

#define ITSDK_DRIVERS_MAX17205_REG_REMAINUPD_ADR			0x1ED   // Remaing nv memory updates


#define ITSDK_DRIVERS_MAX17205_TIME_FOR_NVRECALL			 8	// Time to recall the NV memory content (5ms according to doc + margin)
#define ITSDK_DRIVERS_MAX17205_TIME_FOR_NVSAVE_100MS_LOOP   75	// Time to write the NV memory content (up to 7360ms according to doc)
#define ITSDK_DRIVERS_MAX17205_NVSAVE_MAX_TRY				 1	// Max try to save the NV MEMORY (1 == no retry)

#define ITSDK_DRIVERS_MAX17205_GAUGE_LSB_FACT				 5	// 10x Constant for value in uVh for capacity to coulomb conversion

#endif /* DRIVERS_GAUGE_MAX17205_MAX17205_H_ */
