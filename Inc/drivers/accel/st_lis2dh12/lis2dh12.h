/* ==========================================================
 * lis2dh12.h - ST  LIS2DH12 Hedaers
 * ----------------------------------------------------------
 * 
 *  Created on: 4 avr. 2020
 *      Author: Paul Pinault
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
 * ==========================================================
 */
#include <it_sdk/configDrivers.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/accel/accel.h>


#ifndef SRC_DRIVERS_ACCEL_ST_LIS2DH12_LIS2DH12_H_
#define SRC_DRIVERS_ACCEL_ST_LIS2DH12_LIS2DH12_H_


// Error have to be a bit field.
typedef enum {
	LIS2DH_SUCCESS=0,
	LIS2DH_NOTSUPPORTED=1,

	LIS2DH_FAILED=0x80
} drivers_lis2dh12_ret_e;


// ====== HIGH Pass filter mode
// see http://www.st.com/content/ccc/resource/technical/document/application_note/60/52/bd/69/28/f4/48/2b/DM00165265.pdf/files/DM00165265.pdf/jcr:content/translations/en.DM00165265.pdf
// HPCF[2:1]\ft    @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
//  * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
//  * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
//  * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
//  * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
typedef enum {
	LIS2DH_HPF_MODE_DISABLE = 		0,
	LIS2DH_HPF_MODE_LIGHT = 		1,
	LIS2DH_HPF_MODE_MEDIUM = 		2,
	LIS2DH_HPF_MODE_STRONG = 		3,
	LIS2DH_HPF_MODE_AGGRESSIVE = 	4
} drivers_lis2dh12_hpcfmode_e;

#define DRIVER_LIS2DH_DEFAULT_ADDRESS		0x18		// default I2C address
#define DRIVER_LIS2DH12_SA0_HIGH			0x1
#define DRIVER_LIS2DH12_SA0_LOW				0x0


#define DRIVER_LIS2DH_MIN_WATERMARK			 4			//  because we do not want frequency to be too high
#define DRIVER_LIS2DH_MAX_WATERMARK			28			//  -4 margin

// ********************************************************************************
// Registers
// ********************************************************************************
#define LIS2DH_STATUS_REG_AUX   0x07
#define LIS2DH_OUT_TEMP_L     	0x0C
#define LIS2DH_OUT_TEMP_H     	0x0D
#define LIS2DH_INT_COUNTER_REG  0x0E
#define LIS2DH_WHO_AM_I     	0x0F
#define LIS2DH_CTRL_REG0      	0x1E        // Not documented register : bit 1 => connect / disconnect SDO,SA0 internal pullup
#define LIS2DH_TEMP_CFG_REG   	0x1F
#define LIS2DH_CTRL_REG1    	0x20
#define LIS2DH_CTRL_REG2    	0x21
#define LIS2DH_CTRL_REG3    	0x22
#define LIS2DH_CTRL_REG4    	0x23
#define LIS2DH_CTRL_REG5    	0x24
#define LIS2DH_CTRL_REG6    	0x25
#define LIS2DH_REFERENCE    	0x26
#define LIS2DH_STATUS_REG2    	0x27
#define LIS2DH_OUT_X_L      	0x28
#define LIS2DH_OUT_X_H      	0x29
#define LIS2DH_OUT_Y_L      	0x2A
#define LIS2DH_OUT_Y_H      	0x2B
#define LIS2DH_OUT_Z_L      	0x2C
#define LIS2DH_OUT_Z_H      	0x2D
#define LIS2DH_FIFO_CTRL_REG  	0x2E
#define LIS2DH_FIFO_SRC_REG   	0x2F
#define LIS2DH_INT1_CFG     	0x30
#define LIS2DH_INT1_SOURCE    	0x31
#define LIS2DH_INT1_THS     	0x32
#define LIS2DH_INT1_DURATION  	0x33
#define LIS2DH_INT2_CFG      	0x34
#define LIS2DH_INT2_SOURCE    	0x35
#define LIS2DH_INT2_THS     	0x36
#define LIS2DH_INT2_DURATION  	0x37
#define LIS2DH_CLICK_CFG    	0x38
#define LIS2DH_CLICK_SRC    	0x39
#define LIS2DH_CLICK_THS    	0x3A
#define LIS2DH_TIME_LIMIT     	0x3B
#define LIS2DH_TIME_LATENCY   	0x3C
#define LIS2DH_TIME_WINDOW    	0x3D
#define LIS2DH_ACT_THS      	0x3E
#define LIS2DH_ACT_DUR      	0x3F

//Register Masks

//STATUS_AUX_REG masks
#define LIS2DH_TOR_MASK     	0x40
#define LIS2DH_TDA_MASK     	0x04

//INT_COUNTER masks
//what goes here?

//WHO_AM_I masks
#define LIS2DH_I_AM_VALUE   	0x33

// TEMP_CFG_REG masks
#define LIS2DH_TEMP_EN_MASK   	0xC0

// CTRL_REG0 masks
#define LIS2DH_REG0_SA0PULLUP_MASK 		0x90
#define LIS2DH_REG0_SA0PULLUP_ENABLE 	0x10
#define LIS2DH_REG0_SA0PULLUP_DISABLE 	0x90

// CTRL_REG1 masks
#define LIS2DH_ODR_MASK     	0xF0
#define LIS2DH_LPEN_MASK    	0x08
#define LIS2DH_Z_EN_MASK    	0x04
#define LIS2DH_Y_EN_MASK    	0x02
#define LIS2DH_X_EN_MASK    	0x01
#define LIS2DH_XYZ_EN_MASK  	0x07

#define LIS2DH_ODR_SHIFT      	4
#define LIS2DH_ODR_POWER_DOWN 	0x00
#define LIS2DH_ODR_1HZ        	0x01		// HR & Normal & Low Power
#define LIS2DH_ODR_10HZ       	0x02		// HR & Normal & Low Power
#define LIS2DH_ODR_25HZ       	0x03		// HR & Normal & Low Power
#define LIS2DH_ODR_50HZ       	0x04		// HR & Normal & Low Power
#define LIS2DH_ODR_100HZ      	0x05		// HR & Normal & Low Power
#define LIS2DH_ODR_200HZ      	0x06		// HR & Normal & Low Power
#define LIS2DH_ODR_400HZ      	0x07		// HR & Normal & Low Power
#define LIS2DH_ODR_1620HZ     	0x08		// Low Power
#define LIS2DH_ODR_1344HZ     	0x09		// HR & Normal
#define LIS2DH_ODR_5376HZ     	0x09		// Low Power
#define LIS2DH_ODR_MAXVALUE   	0x09

typedef enum {
	LIS2DH_FREQUENCY_POWERDOWN = LIS2DH_ODR_POWER_DOWN,
	LIS2DH_FREQUENCY_1HZ = LIS2DH_ODR_1HZ,
	LIS2DH_FREQUENCY_10HZ = LIS2DH_ODR_10HZ,
	LIS2DH_FREQUENCY_25HZ = LIS2DH_ODR_25HZ,
	LIS2DH_FREQUENCY_50HZ = LIS2DH_ODR_50HZ,
	LIS2DH_FREQUENCY_100HZ = LIS2DH_ODR_100HZ,
	LIS2DH_FREQUENCY_200HZ = LIS2DH_ODR_200HZ,
	LIS2DH_FREQUENCY_400HZ = LIS2DH_ODR_400HZ,
	LIS2DH_FREQUENCY_1620HZ = LIS2DH_ODR_1620HZ,
	LIS2DH_FREQUENCY_1344HZ = LIS2DH_ODR_1344HZ,
	LIS2DH_FREQUENCY_5376HZ = LIS2DH_ODR_5376HZ
} drivers_lis2dh12_frequency_e;


// CTRL_REG2 masks
#define LIS2DH_HPM_MASK         0xC0
#define LIS2DH_HPCF_MASK        0x30
#define LIS2DH_FDS_MASK         0x08
#define LIS2DH_HPCLICK_MASK     0x04
#define LIS2DH_HPIA2_MASK       0x02        // Apply filtering on interrupt 2
#define LIS2DH_HPIA1_MASK       0x01        // Apply filtering on interrupt 1

#define LIS2DH_HPM_SHIFT          6
#define LIS2DH_HPM_NORMAL_RESET   0x00      // In this mode - when reading on of the XL/XH_REFERENCE register the current acceleration is reset on the corresponding axe (manuel reset)
#define LIS2DH_HPM_REFSIG         0x01      // In this mode acceleration is the difference with the XL/XH_REFERENCE content for each axis
#define LIS2DH_HPM_NORMAL2        0x02      // In this mode I assume we have no filtering
#define LIS2DH_HPM_AUTORESET      0x03      // In this mode the interrupt event will reset the filter
#define LIS2DH_HPM_MAXVALUE       0x03

#define LIS2DH_HPCF_SHIFT         4
#define LIS2DH_HPCF_ODR_50        0x00      // F cut = ODR Freq / 50
#define LIS2DH_HPCF_ODR_100       0x01
#define LIS2DH_HPCF_ODR_9         0x02
#define LIS2DH_HPCF_ODR_400       0x03
#define LIS2DH_HPCF_MAXVALUE      0x03

typedef enum {
	LIS2DH_HPCF_ODRon9 = LIS2DH_HPCF_ODR_9,
	LIS2DH_HPCF_ODRon50 = LIS2DH_HPCF_ODR_50,
	LIS2DH_HPCF_ODRon100 = LIS2DH_HPCF_ODR_100,
	LIS2DH_HPCF_ODRon400 = LIS2DH_HPCF_ODR_400
} drivers_lis2dh12_hpcf_e;


// CTRL_REG3 masks
#define LIS2DH_I1_CLICK           0x80      // Interrupt on click on INT1
#define LIS2DH_I1_IA1             0x40      // Interrupt from IA1 on INT1
#define LIS2DH_I1_IA2             0x20      // Interrupt from IA2 on INT1
#define LIS3DH_I1_ZYXDA           0x10      // Any Data axis on INT1
#define LIS2DH_I1_WTM             0x04      // FiFo Watermark on INT1
#define LIS2DH_I1_OVERRUN         0x02      // FiFo Overrun on INT1
#define LIS2DH_I1_INTERRUPT_NONE  0x00

// CTRL_REG6 masks
#define LIS2DH_I2_MASK            0xF8       // Mask for interrupt
#define LIS2DH_I2_CLICK           0x80       // Click interrupt
#define LIS2DH_I2_IA1             0x40       // Interrupt 1 function
#define LIS2DH_I2_IA2             0x20       // Interrupt 2 function
#define LIS2DH_I2_BOOT            0x10       // Boot interrupt
#define LIS2DH_I2_ACTIVITY        0x08       // Activity interrupt
#define LIS2DH_I2_INTERRUPT_NONE  0x00
#define LIS2DH_INT_POLARITY       0x02        // Interupt polarity => 0 active high / 1 active low



// CTRL_REG4 masks
#define LIS2DH_BDU_MASK     	0x80
#define LIS2DH_BLE_MASK     	0x40
#define LIS2DH_FS_MASK      	0x30
#define LIS2DH_HR_MASK      	0x08
#define LIS2DH_ST_MASK      	0x06
#define LIS2DH_SIM_MASK     	0x01

#define LIS2DH_FS_SHIFT     	4
#define LIS2DH_FS_SCALE_2G  	0x00
#define LIS2DH_FS_SCALE_4G  	0x01
#define LIS2DH_FS_SCALE_8G  	0x02
#define LIS2DH_FS_SCALE_16G 	0x03
#define LIS2DH_FS_MAXVALUE  	0x03

typedef enum {
	LIS2DH_SCALE_FACTOR_2G = LIS2DH_FS_SCALE_2G,
	LIS2DH_SCALE_FACTOR_4G = LIS2DH_FS_SCALE_4G,
	LIS2DH_SCALE_FACTOR_8G = LIS2DH_FS_SCALE_8G,
	LIS2DH_SCALE_FACTOR_16G = LIS2DH_FS_SCALE_16G
} drivers_lis2dh12_scale_e;


#define LIS2DH_RESOLUTION_8B        0x01      // Different resolution mode => Low Power
#define LIS2DH_RESOLUTION_10B       0x02      // Normal mode (10b)
#define LIS2DH_RESOLUTION_12B       0x03      // High Resolution mode (12b)
#define LIS2DH_RESOLUTION_MAXVALUE  0x03

typedef enum {
	LIS2DH_RESOLUTION_MODE_8B = LIS2DH_RESOLUTION_8B,
	LIS2DH_RESOLUTION_MODE_10B = LIS2DH_RESOLUTION_10B,
	LIS2DH_RESOLUTION_MODE_12B = LIS2DH_RESOLUTION_12B
} drivers_lis2dh12_resolution_e;


// CTRL_REG5 masks
#define LIS2DH_BOOT_MASK      	0x80
#define LIS2DH_FIFO_EN_MASK   	0x40
#define LIS2DH_LIR_INT1_MASK  	0x08
#define LIS2DH_D4D_INT1_MASK  	0x04
#define LIS2DH_LIR_INT2_MASK  	0x02
#define LIS2DH_D4D_INT2_MASK  	0x01


// REF masks
// none

// STATUS_REG masks
#define LIS2DH_STATUS_ZYXOR     0x80      // X, Y, Z data overrun => a new set of data has overwritten the previous set
#define LIS2DH_STATUS_ZOR       0x40      // Z overrun
#define LIS2DH_STATUS_YOR       0x20      // Y overrun
#define LIS2DH_STATUS_XOR       0x10      // X overrun
#define LIS2DH_STATUS_ZYXDA     0x08      // X, Y, Z data available => a new set of data is availbale
#define LIS2DH_STATUS_ZDA       0x04      // Z data available
#define LIS2DH_STATUS_YDA       0x02      // Y data available
#define LIS2DH_STATUS_XDA       0x01      // X data available

// FIFO_CTRL_REG masks
#define LIS2DH_FM_MASK          0xC0
#define LIS2DH_TR_MASK          0x20
#define LIS2DH_FTH_MASK         0x1F

#define LIS2DH_FM_SHIFT         6
#define LIS2DH_FM_BYPASS        0x00      // No FIFO at all, the acceleration are not stored in FIFO
#define LIS2DH_FM_FIFO          0x01      // FIFO is used until being full after that no more data are added until clearing it by switching to bypass
#define LIS2DH_FM_STREAM        0x02      // FIFO is used and when full the older data are replaced by the new one.
#define LIS2DH_FM_STREAMFIFO    0x03      // In this mode the Interrupt Generator will automatically swicth the mode from STREAM to FiFo
#define LIS2DH_FM_MAXVALUE      0x03

typedef enum {
	LIS2DH_FIFO_MODE_BYPASS 	= LIS2DH_FM_BYPASS,
	LIS2DH_FIFO_MODE_FIFO 		= LIS2DH_FM_FIFO,
	LIS2DH_FIFO_MODE_STREAM 	= LIS2DH_FM_STREAM,
	LIS2DH_FIFO_MODE_STREAMFIFO = LIS2DH_FM_STREAMFIFO
} drivers_lis2dh12_fifomode_e;


#define LIS2DH_TR_SHIFT         5
#define LIS2DH_TR_INT1          0x00
#define LIS2DH_TR_INT2          0x01
#define LIS2DH_TR_MAXVALUE      0x01

typedef enum {
	LIS2DH_ONTRIGGER_INT1  = LIS2DH_TR_INT1,
	LIS2DH_ONTRIGGER_INT2  = LIS2DH_TR_INT2
} drivers_lis2dh12_triggers_e;

#define LIS2DH_FTH_SHIFT        0
#define LIS2DH_FTH_MAXVALUE     32

// FIFO_SRC_REG masks
#define LIS2DH_WTM_MASK          	0x80
#define LIS2DH_OVRN_FIFO_MASK    	0x40
#define LIS2DH_EMPTY_MASK        	0x20
#define LIS2DH_FSS_MASK          	0x1F

// INT1/2_CFG masks
#define LIS2DH_AOI_MASK         	0x80	// INT 1 specific
#define LIS2DH_6D_MASK          	0x40
#define LIS2DH_INT_MODE_MASK    	0xC0	// INT 2
#define LIS2DH_ZHIE_MASK        	0x20
#define LIS2DH_ZLIE_MASK        	0x10
#define LIS2DH_YHIE_MASK        	0x08
#define LIS2DH_YLIE_MASK        	0x04
#define LIS2DH_XHIE_MASK        	0x02
#define LIS2DH_XLIE_MASK        	0x01
#define LIS2DH_INTEVENT_MASK    	0x3F
#define LIS2DH_ALL_MASK    			0xFF

#define LIS2DH_INT_MODE_SHIFT   	6
#define LIS2DH_INT_MODE_OR      	0x00      // If one of the event triggers, the interrupt is fired
#define LIS2DH_INT_MODE_AND     	0x02      // When all the event have triggered, the inteerupt is fired
#define LIS2DH_INT_MODE_MOV     	0x01      // Movement recognition => when the orientation move from and unknown zone to a known zone, duration is ODR
#define LIS2DH_INT_MODE_POS     	0x03      // Interrupt fired when the orientation is in a known zone and stay until we stay in this zone => Position
#define LIS2DH_INT_MODE_MAXVALUE  	0x03

#define LIS2DH_INT_MODE_6D_STABLE	0x03	 // Active as soon as a stable position is detected, remain until the position is unknown
#define LIS2DH_INT_MODE_6D_CHANGE   0x01	 // Active when the position is becoming stable (state change detection)

typedef enum {
	LIS2DH_INTERRUPT_MODE_OR = LIS2DH_INT_MODE_OR,
	LIS2DH_INTERRUPT_MODE_AND = LIS2DH_INT_MODE_AND,
	LIS2DH_INTERRUPT_MODE_MOV = LIS2DH_INT_MODE_MOV,
	LIS2DH_INTERRUPT_MODE_POS = LIS2DH_INT_MODE_POS,
	LIS2DH_INTERRUPT_MODE_6D_STABLE = LIS2DH_INT_MODE_6D_STABLE,
	LIS2DH_INTERRUPT_MODE_6D_CHANGE = LIS2DH_INT_MODE_6D_CHANGE

} drivers_lis2dh12_intmode_e;

#define LIS2DH_INTEVENT_SHIFT   	0
#define LIS2DH_INTEVENT_Z_HIGH  	0x20      // Fire interrupt on Z high event of direction recognotion
#define LIS2DH_INTEVENT_Z_LOW   	0x10      // Fire interrupt on Z low event of direction recognotion
#define LIS2DH_INTEVENT_Y_HIGH  	0x08      // Fire interrupt on Y high event of direction recognotion
#define LIS2DH_INTEVENT_Y_LOW   	0x04      // Fire interrupt on Y low event of direction recognotion
#define LIS2DH_INTEVENT_X_HIGH  	0x02      // Fire interrupt on X high event of direction recognotion
#define LIS2DH_INTEVENT_X_LOW   	0x01      // Fire interrupt on X low event of direction recognotion
#define LIS2DH_INTEVENT_NONE    	0x00      // No interrupt
#define LIS2DH_INTEVENT_ALL     	0x3F      // No interrupt
#define LIS2DH_INTEVENT_MAXVALUE 	0x3F

typedef enum {
	LIS2DH_INTERRUPT_EVENT_ZHIGH = LIS2DH_INTEVENT_Z_HIGH,
	LIS2DH_INTERRUPT_EVENT_ZLOW  = LIS2DH_INTEVENT_Z_LOW,
	LIS2DH_INTERRUPT_EVENT_YHIGH = LIS2DH_INTEVENT_Y_HIGH,
	LIS2DH_INTERRUPT_EVENT_YLOW = LIS2DH_INTEVENT_Y_LOW,
	LIS2DH_INTERRUPT_EVENT_XHIGH = LIS2DH_INTEVENT_X_HIGH,
	LIS2DH_INTERRUPT_EVENT_XLOW = LIS2DH_INTEVENT_X_LOW,
	LIS2DH_INTERRUPT_EVENT_NONE = LIS2DH_INTEVENT_NONE,
	LIS2DH_INTERRUPT_EVENT_ALL  = LIS2DH_INTEVENT_ALL
} drivers_lis2dh12_intevent_e;

// INT1/2_SRC masks
#define LIS2DH_INT_IA_MASK    		0x40
#define LIS2DH_ZH_MASK        		0x20
#define LIS2DH_ZL_MASK        		0x10
#define LIS2DH_YH_MASK        		0x08
#define LIS2DH_YL_MASK        		0x04
#define LIS2DH_XH_MASK        		0x02
#define LIS2DH_XL_MASK        		0x01
#define LIS2DH_INT_ALL_MASK   		0x7F
#define LIS2DH_INT_POS_MASK   		0x3F

// INT1/2_THS masks
#define LIS2DH_THS_MASK       		0x7F
#define LIS2DH_THS_SHIFT      		0
#define LIS2DH_THS_MAXVALUE   		0x7F

// INT1/2_DURATION masks
#define LIS2DH_D_MASK         		0x7F
#define LIS2DH_DUR_SHIFT      		0
#define LIS2DH_DUR_MAXVALUE   		0x7F

// CLICK_CFG masks
#define LIS2DH_ZD_MASK      		0x20
#define LIS2DH_ZS_MASK      		0x10
#define LIS2DH_YD_MASK      		0x08
#define LIS2DH_YS_MASK      		0x04
#define LIS2DH_XD_MASK      		0x02
#define LIS2DH_XS_MASK      		0x01
#define LIS2DH_CLICEVENT_MASK       0x3F      // Interrupt mask

#define LIS2DH_CLICEVENT_SHIFT        0
#define LIS2DH_CLICEVENT_DBLE_Z       0x20      // Fire interrupt when detect double clic on Z - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_SINGLE_Z     0x10      // Fire interrupt when detect single clic on Z - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_DBLE_Y       0x08      // Fire interrupt when detect double clic on Y - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_SINGLE_Y     0x04      // Fire interrupt when detect single clic on Y - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_DBLE_X       0x02      // Fire interrupt when detect double clic on X - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_SINGLE_X     0x01      // Fire interrupt when detect single clic on X - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_MAXVALUE     0x3F



// CLICK_SRC masks
                                        // Basically we have 6 axis (X,Y,Z) in both positive and negative direction
                                        // we also have two type of clics => Single and Double
                                        // This set of bit allow to identify all these different kind of clics

#define LIS2DH_CLK_IA_MASK    		0x40      // An interrupt is active on one of the following bits
#define LIS2DH_DCLICK_MASK    		0x20      // Double clic detected
#define LIS2DH_SCLICK_MASK    		0x10      // Single clic detected
#define LIS2DH_SIGN_MASK      		0x08      // 0 - Positive detection / 1 - Negative detection (direction of the clic)
#define LIS2DH_Z_CLICK_MASK   		0x04      // Clic on Z axis
#define LIS2DH_Y_CLICK_MASK   		0x02      // Clic on Y axis
#define LIS2DH_X_CLICK_MASK   		0x01      // Clic on X axis
#define LIS2DH_CLICK_SRC_MASK 		0x7F

#define LIS2DH_CLIC_NONE        	0x0000  // For each of the possible clic combination. ( they can be cumulated )
#define LIS2DH_CLIC_DBL_X_POS   	0x0001
#define LIS2DH_CLIC_DBL_X_NEG   	0x0002
#define LIS2DH_CLIC_SIN_X_POS   	0x0004
#define LIS2DH_CLIC_SIN_X_NEG   	0x0008
#define LIS2DH_CLIC_DBL_Y_POS   	0x0010
#define LIS2DH_CLIC_DBL_Y_NEG   	0x0020
#define LIS2DH_CLIC_SIN_Y_POS   	0x0040
#define LIS2DH_CLIC_SIN_Y_NEG   	0x0080
#define LIS2DH_CLIC_DBL_Z_POS   	0x0100
#define LIS2DH_CLIC_DBL_Z_NEG   	0x0200
#define LIS2DH_CLIC_SIN_Z_POS   	0x0400
#define LIS2DH_CLIC_SIN_Z_NEG   	0x0800



// CLICK_THS masks
#define LIS2DH_CLK_THS_MASK       	0x7F
#define LIS2DH_CLK_THS_SHIFT      	0
#define LIS2DH_CLK_THS_MAXVALUE   	0x7F

#define LIS2DH_CLK_INTDUR_MASK        0x80
#define LIS2DH_CLK_INTDUR_SHIFT       7
#define LIS2DH_CLK_INTDUR_UNTILREAD   0x01    // Click interrupt is pending until the register CLICK_SRC has been read
#define LIS2DH_CLK_INTDUR_LATWINDOW   0x00    // Click interrupt is automatically cancel after the latency window duration ( TIME_LATENCY register )
#define LIS2DH_CLK_INTDUR_MAXVALUE    0x01

// TIME_LIMIT masks
#define LIS2DH_TLI_MASK             0x7F
#define LIS2DH_TLI_SHIFT            0
#define LIS2DH_TLI_MAXVALUE         0x7F

// TIME_LATENCY masks
#define LIS2DH_TIME_LATENCY_MASK         0xFF
#define LIS2DH_TIME_LATENCY_SHIFT        0
#define LIS2DH_TIME_LATENCY_MAXVALUE     0xFF

// TIME_WINDOW masks
#define LIS2DH_TIME_WINDOW_MASK          0xFF
#define LIS2DH_TIME_WINDOW_SHIFT         0
#define LIS2DH_TIME_WINDOW_MAXVALUE      0xFF

// ACT_THS masks
#define LIS2DH_ACT_THS_MASK       	0x7F
#define LIS2DH_ACT_THS_SHIFT      	0
#define LIS2DH_ACT_THS_MAXVALUE   	0x7F

// ACT_DUR masks
#define LIS2DH_ACT_DUR_MASK       	0xFF
#define LIS2DH_ACT_DUR_SHIFT      	0
#define LIS2DH_ACT_DUR_MAXVALUE   	0xFF



#define LIS2DH_INTERRUPT1     1
#define LIS2DH_INTERRUPT2     2

// 6D position of the Object. Assuming the TOP is component top side
// We assume the position entry is the direction the TOP side is looking
#define LIS2DH_POSITION_TOP_ON_NONE     0x00
#define LIS2DH_POSITION_TOP_ON_TOP      0x20
#define LIS2DH_POSITION_TOP_ON_BOTTOM   0x10
#define LIS2DH_POSITION_TOP_ON_RIGHT    0x02
#define LIS2DH_POSITION_TOP_ON_LEFT     0x01
#define LIS2DH_POSITION_TOP_ON_FRONT    0x04
#define LIS2DH_POSITION_TOP_ON_BACK     0x08
#define LIS2DH_POSITION_TOP_ON_ANY      0x3F

#define LIS2DH_TEMPERATURE_INVALID      0x1FFF


// ===================================================================================
// APIs
// ===================================================================================



typedef enum {
	LIS2DH_INTERRUPT_POLARITY_LOW = 0,
	LIS2DH_INTERRUPT_POLARITY_HIGH = 1
} drivers_lis2dh12_interruptPol_e;


typedef struct {
	uint8_t _address;									// I2C address
	drivers_lis2dh12_resolution_e 	_resolution;       	// store the current resolution (bits)
	drivers_lis2dh12_frequency_e 	_frequency;    		// store the current frequency (Hz) - ODR
    drivers_lis2dh12_scale_e 		_scale;         	// store the current scale (xG)
    drivers_lis2dh12_fifomode_e 	_fifoMode;          // store the current fifo mode
    drivers_lis2dh12_hpcf_e 		_hpcf;              // store the current cut-off frequency on ODR mode
    itsdk_bool_e					_tiltModeEnable;	// true when the tilt mode is running in background
    itsdk_bool_e					_clicModeReadClear;	// true when the clic_src register is cleared on reading because reading is differnt
    itsdk_accel_trigger_e			_tiltTriggerMsk;	// Desired Triggers for tilt detection
    itsdk_accel_trigger_e			_tiltTriggerLast;	// Last trigger state for position change detection
    void(* _tiltCB)(itsdk_accel_trigger_e reason);		// callback action on tilt event

    itsdk_bool_e					_captureModeEnable;	// true when the capture mode is running in background
    void (* _captureCB)(itsdk_accel_data_t * data, itsdk_accel_dataFormat_e format, uint8_t count, itsdk_bool_e overrun);

} drivers_lis2dh12_conf_t;

// Main function to be use
drivers_lis2dh12_ret_e lis2dh_simple_setup(void);                                                                    // Default Init 8b, 10Hz, 2G
drivers_lis2dh12_ret_e lis2dh_setup(
		drivers_lis2dh12_resolution_e resolution,
		drivers_lis2dh12_frequency_e frequency,
		drivers_lis2dh12_scale_e scale);
drivers_lis2dh12_ret_e lis2dh_initPosition6D(drivers_lis2dh12_scale_e scale, drivers_lis2dh12_frequency_e frequency); // Configure the 6D position function on INT1
drivers_lis2dh12_ret_e lis2dh_reinit();                                                                      // Do not change the device config but restore the lis2dh12 structure
void lis2dh12_process(void);
void lis2dh_dumpConfig(void);

uint8_t lis2dh_getPendingMotions( itsdk_accel_data_t * _buffer, uint8_t size);                        // Get all the FiFo pending measure for X,Y,Z
uint8_t lis2dh_getPendingAcceleration( itsdk_accel_data_t * _buffer, uint8_t size);                   // Get all the Fifo pending measure in Mg ( x,y,z )
uint8_t lis2dh_getPendingForces( uint16_t * _buffer, uint8_t size);                         // Get all the Fifo pending measure into a force in Mg array |A|

drivers_lis2dh12_ret_e lis2dh_setupBackgroundTiltDetection(								   // Configure a background movement detection with Interrupt or activ scan
		uint16_t forceMg, 							// force level
		drivers_lis2dh12_scale_e scale,				// scale 2G/4G...
		drivers_lis2dh12_frequency_e frequency, 	// capture frequency 1/10/25/50Hz..
		drivers_lis2dh12_resolution_e resolution,	// data precision 8B/10B/12B...
		drivers_lis2dh12_hpcfmode_e hpf,			// High Frequency Filter strategy
		itsdk_accel_trigger_e triggers,				// Select the triggers
		void(* cb)(itsdk_accel_trigger_e reason) 	// callback function
);
drivers_lis2dh12_ret_e lis2dh_cancelBackgroundTiltDetection(void);
itsdk_bool_e lis2dh_hasTiltDetected();                                                     // Return true when a tilt has been detected and clear the interrupt
itsdk_accel_trigger_e __lis2dh_determineTilt();


drivers_lis2dh12_ret_e lis2dh_setupDataAquisition(
		drivers_lis2dh12_scale_e 		scale,				// scale 2G/4G...
		drivers_lis2dh12_frequency_e 	frequency, 			// capture frequency 1/10/25/50Hz..
		drivers_lis2dh12_resolution_e 	resolution,			// data precision 8B/10B/12B...
		itsdk_accel_trigger_e 			axis,				// Select the axis
		drivers_lis2dh12_hpcfmode_e 	hpf,				// High Frequency Filter strategy
		uint8_t							dataBlock,			// Expected data to be returned by callback
		void (* callback)(itsdk_accel_data_t * data, itsdk_accel_dataFormat_e format, uint8_t count, itsdk_bool_e overrun)
															// callback function the interrupt will call
															// data format is always raw
);
drivers_lis2dh12_ret_e lis2dh_cancelDataAquisition(void);

uint8_t lis2dh_getPosition6D();                                                            // Return one of the 6D positions

// Low level functions
int16_t lis2dh_getAxisX(void);                                                             // Get the last measured X acceleration
int16_t lis2dh_getAxisY(void);                                                             // Get the last measured Y acceleration
int16_t lis2dh_getAxisZ(void);                                                             // Get the last measured Z acceleration
void lis2dh_getMotion(int16_t* ax, int16_t* ay, int16_t* az);                              // Get the last measured X,Y,Z acceleration
int8_t lis2dh_getAxisX_LR(void);                                                           // Get the last measured X acceleration in LowpoweR mode (8 bits)
int8_t lis2dh_getAxisY_LR(void);                                                           // Get the last measured Y acceleration in LowpoweR mode (8 bits)
int8_t lis2dh_getAxisZ_LR(void);                                                           // Get the last measured Z acceleration in LowpoweR mode (8 bits)
void lis2dh_getMotion_LR(int8_t* ax, int8_t* ay, int8_t* az);                              // Get the last measured X,Y,Z acceleration in LowpowerR mode (8bits)


itsdk_bool_e lis2dh_tempHasOverrun(void);
itsdk_bool_e lis2dh_tempDataAvailable(void);
int16_t lis2dh_getTemperature(void);
drivers_lis2dh12_ret_e lis2dh_whoAmI(void);
drivers_lis2dh12_ret_e lis2dh_getTempEnabled(void);
drivers_lis2dh12_ret_e lis2dh_setTempEnabled(itsdk_bool_e enable);
drivers_lis2dh12_frequency_e lis2dh_getDataRate(void);
drivers_lis2dh12_ret_e lis2dh_setDataRate(drivers_lis2dh12_frequency_e rate);
drivers_lis2dh12_ret_e lis2dh_enableLowPower(void);
drivers_lis2dh12_ret_e lis2dh_disableLowPower(void);
itsdk_bool_e lis2dh_isLowPowerEnabled(void);
drivers_lis2dh12_ret_e lis2dh_setPowerDown(void);
drivers_lis2dh12_ret_e lis2dh_setSleepNWakeUpThreshold(uint8_t raw);
drivers_lis2dh12_ret_e lis2dh_setSleepNWakeUpThresholdMg( uint8_t mg, const uint8_t scale);
drivers_lis2dh12_ret_e lis2dh_setSleepNWakeUpDuration(uint8_t raw);
drivers_lis2dh12_ret_e lis2dh_setSleepNWakeUpDurationMs(uint8_t _int, uint32_t ms, const uint8_t odr);

drivers_lis2dh12_ret_e lis2dh_enableAxisX(void);
drivers_lis2dh12_ret_e lis2dh_disableAxisX(void);
itsdk_bool_e lis2dh_isXAxisEnabled(void);
drivers_lis2dh12_ret_e lis2dh_enableAxisY(void);
drivers_lis2dh12_ret_e lis2dh_disableAxisY(void);
itsdk_bool_e lis2dh_isYAxisEnabled(void);
drivers_lis2dh12_ret_e lis2dh_enableAxisZ(void);
drivers_lis2dh12_ret_e lis2dh_disableAxisZ(void);
itsdk_bool_e lis2dh_isZAxisEnabled(void);
drivers_lis2dh12_hpcf_e __lis2dh_getCutOffODRValueFromHPFMode(
		drivers_lis2dh12_hpcfmode_e mode
);
uint8_t lis2dh_getHPFilterMode();
drivers_lis2dh12_ret_e lis2dh_setHPFilterMode(uint8_t mode);
drivers_lis2dh12_hpcf_e lis2dh_getHPFilterCutOff();
drivers_lis2dh12_ret_e lis2dh_setHPFilterCutOff(drivers_lis2dh12_hpcf_e mode);
drivers_lis2dh12_ret_e lis2dh_enableHPClick(void);
drivers_lis2dh12_ret_e lis2dh_disableHPClick(void);
itsdk_bool_e lis2dh_isHPClickEnabled(void);
drivers_lis2dh12_ret_e lis2dh_enableHPIA1(void);
drivers_lis2dh12_ret_e lis2dh_disableHPIA1(void);
itsdk_bool_e lis2dh_isHPIA1Enabled(void);
drivers_lis2dh12_ret_e lis2dh_enableHPIA2(void);
drivers_lis2dh12_ret_e lis2dh_disableHPIA2(void);
itsdk_bool_e lis2dh_isHPIA2Enabled(void);
drivers_lis2dh12_ret_e lis2dh_enableHPFDS(void);
drivers_lis2dh12_ret_e lis2dh_disableHPFDS(void);
itsdk_bool_e lis2dh_isHPFDSEnabled(void);
drivers_lis2dh12_ret_e lis2dh_enableAxisXYZ(void);
drivers_lis2dh12_ret_e lis2dh_disableAxisXYZ(void);

drivers_lis2dh12_ret_e lis2dh_enableInterruptPin1(uint8_t _int);
drivers_lis2dh12_ret_e lis2dh_disableInterruptPin1(uint8_t _int);
drivers_lis2dh12_ret_e lis2dh_enableInterruptPin2(uint8_t _int);
drivers_lis2dh12_ret_e lis2dh_disableInterruptPin2(uint8_t _int);
drivers_lis2dh12_ret_e lis2dh_disableAllInterrupt();
drivers_lis2dh12_ret_e lis2dh_setInterruptPolarity(drivers_lis2dh12_interruptPol_e polarity);
drivers_lis2dh12_ret_e lis2dh_triggerSelect(drivers_lis2dh12_triggers_e triggerMode);
drivers_lis2dh12_ret_e lis2dh_intWorkingMode(uint8_t _int, drivers_lis2dh12_intmode_e _mode);              // Set the interrupt mode (OR, AND, 6D Movement, 6D Positions) see LIS2DH_INT_MODE_XXX
drivers_lis2dh12_ret_e lis2dh_enableInterruptEvent(uint8_t _int, drivers_lis2dh12_intevent_e _intEvent);   // Select the interrupt event source X,Y,Z Higher or Lower see LIS2DH_INTEVENT_XX
itsdk_bool_e lis2dh_isInterruptFired(uint8_t _int);
itsdk_bool_e lis2dh_isInterruptZHighFired(uint8_t _int);
itsdk_bool_e lis2dh_isInterruptZLowFired(uint8_t _int);
itsdk_bool_e lis2dh_isInterruptYHighFired(uint8_t _int);
itsdk_bool_e lis2dh_isInterruptYLowFired(uint8_t _int);
itsdk_bool_e lis2dh_isInterruptXHighFired(uint8_t _int);
itsdk_bool_e lis2dh_isInterruptXLowFired(uint8_t _int);
drivers_lis2dh12_ret_e lis2dh_setInterruptThreshold(uint8_t _int, uint8_t raw);                            // Set the raw limit for an interrupt to be fired
drivers_lis2dh12_ret_e lis2dh_setInterruptThresholdMg(uint8_t _int, uint8_t mg, const drivers_lis2dh12_scale_e scale);      // Set the limit in mG for an interrupt to be fired
drivers_lis2dh12_ret_e lis2dh_setInterruptDuration(uint8_t _int, uint8_t raw);
drivers_lis2dh12_ret_e lis2dh_setInterruptDurationMs(uint8_t _int, uint32_t ms, const uint8_t odr);
drivers_lis2dh12_ret_e lis2dh_enableLatchInterrupt(uint8_t _int, itsdk_bool_e enable);                             // When true, Interrupt is cleared only when INTx_SRC register is read

drivers_lis2dh12_ret_e lis2dh_setLittleEndian();
drivers_lis2dh12_ret_e lis2dh_setBitEndian();
drivers_lis2dh12_ret_e lis2dh_setContinuousUpdate(itsdk_bool_e continuous);
drivers_lis2dh12_ret_e lis2dh_setAccelerationScale(drivers_lis2dh12_scale_e scale);
drivers_lis2dh12_scale_e lis2dh_getAccelerationScale();
drivers_lis2dh12_ret_e lis2dh_setHighResolutionMode(itsdk_bool_e hr);
itsdk_bool_e lis2dh_isHighResolutionMode();

drivers_lis2dh12_ret_e lis2dh_reboot();
drivers_lis2dh12_ret_e lis2dh_enableFifo(itsdk_bool_e enable);

drivers_lis2dh12_ret_e lis2dh_setReference(uint8_t ref);
drivers_lis2dh12_ret_e lis2dh_getReference(uint8_t * ref);
drivers_lis2dh12_ret_e lis2dh_resetFilteringBlock();
uint8_t lis2dh_getDataStatus();
drivers_lis2dh12_ret_e lis2dh_setResolutionMode(drivers_lis2dh12_resolution_e res);
drivers_lis2dh12_resolution_e lis2dh_getResolutionMode();

drivers_lis2dh12_ret_e lis2dh_setFiFoMode(drivers_lis2dh12_fifomode_e fifoMode);
drivers_lis2dh12_fifomode_e lis2dh_getFiFoMode();
drivers_lis2dh12_ret_e lis2dh_setFiFoThreshold(uint8_t threshold);
itsdk_bool_e lis2dh_isFiFoWatermarkExceeded();
itsdk_bool_e lis2dh_isFiFoFull();
itsdk_bool_e lis2dh_isFiFoEmpty();
uint8_t lis2dh_getFiFoSize();

drivers_lis2dh12_ret_e lis2dh_enableClickEvent(itsdk_accel_trigger_e clicTrigger);
itsdk_bool_e lis2dh_isClickInterruptFired();
itsdk_bool_e lis2dh_isDoubleClickFired();
itsdk_bool_e lis2dh_isSimpleClickFired();
itsdk_bool_e lis2dh_isClickFiredOnZ();
itsdk_bool_e lis2dh_isClickFiredOnY();
itsdk_bool_e lis2dh_isClickFiredOnX();
itsdk_bool_e lis2dh_isSignClickFired();
itsdk_accel_trigger_e lis2dh_getClickStatus();
drivers_lis2dh12_ret_e lis2dh_setClickThreshold(uint8_t ths);
drivers_lis2dh12_ret_e lis2dh_setClickThresholdMg(uint16_t mg, const drivers_lis2dh12_scale_e scale);
drivers_lis2dh12_ret_e lis2dh_setClickInterruptMode(uint8_t _mode);
drivers_lis2dh12_ret_e lis2dh_setClickTimeLimitMs(uint16_t ms, const drivers_lis2dh12_frequency_e dr);
drivers_lis2dh12_ret_e lis2dh_setClickTimeLatencyMs(uint16_t ms, const drivers_lis2dh12_frequency_e dr);
drivers_lis2dh12_ret_e lis2dh_setClickTimeWindowMs(uint16_t ms, const drivers_lis2dh12_frequency_e dr);
drivers_lis2dh12_ret_e lis2dh_setClickInterrupt(itsdk_bool_e enable);

// Internal function not supposed to be used other than lis driver
void __lis2dh_readSetting();                                                                                                   // Restore scale, resolution, frequency based on chip current config


drivers_lis2dh12_ret_e __lis2dh_getAcceleration_i(const drivers_lis2dh12_resolution_e resolution, const drivers_lis2dh12_scale_e scale, int16_t * ax, int16_t * ay, int16_t * az);        // return acceleration in mg instead of raw values
drivers_lis2dh12_ret_e __lis2dh_getAcceleration(int16_t * x, int16_t * y, int16_t * z);                                                          // equivalent but use internal known config for this.

drivers_lis2dh12_ret_e __lis2dh_writeRegister(const uint8_t register_addr, const uint8_t value);
drivers_lis2dh12_ret_e __lis2dh_writeRegisters(const uint8_t msb_register, const uint8_t msb_value, const uint8_t lsb_register, const uint8_t lsb_value);
drivers_lis2dh12_ret_e __lis2dh_writeMaskedRegister8(const uint8_t register_addr, const uint8_t mask, const uint8_t value);
drivers_lis2dh12_ret_e __lis2dh_writeMaskedRegisterI(const int register_addr, const int mask, const int value);
uint8_t __lis2dh_readRegister(const uint8_t register_addr);
uint16_t __lis2dh_readRegisters(const uint8_t msb_register, const uint8_t lsb_register);
uint8_t __lis2dh_readMaskedRegister(const uint8_t register_addr, const uint8_t mask);
//uint8_t __lis2dh_readFifo(int16_t * _buffer,const uint8_t maxSz);
uint8_t __lis2dh_readFifo(itsdk_accel_data_t * _buffer,const uint8_t maxSz);

drivers_lis2dh12_ret_e __lis2dh_convertMgToRaw(uint8_t * _raw, uint16_t mg, drivers_lis2dh12_scale_e scale);
drivers_lis2dh12_ret_e __lis2dh_convertMsToRaw(uint8_t * _raw, uint32_t ms, const drivers_lis2dh12_frequency_e odr);

// Converter function to adapt with itsdk acces generic driver
drivers_lis2dh12_scale_e lis2dh12_convertScale(itsdk_accel_scale_e intput);
drivers_lis2dh12_frequency_e lis2dh_converFrequency(itsdk_accel_frequency_e input, itsdk_accel_precision_e p);
drivers_lis2dh12_resolution_e lis2dh_convertPrecision(itsdk_accel_precision_e input);
drivers_lis2dh12_ret_e lis2dh_convertRawToMg(itsdk_accel_data_t * data);
drivers_lis2dh12_hpcfmode_e lis2dh_convertHPF(itsdk_accel_hpf_e hpfm) ;

// Logger wrapper

#if ITSDK_DRIVERS_ACCEL_LIS2DH_LOG_LEVEL >= __LOG_LEVEL_VERBOSE_STD && (ITSDK_LOGGER_MODULE & __LOG_MOD_ACCEL) > 0
#define LIS2DH_LOG_DEBUG(x) log_debug x
#else
#define LIS2DH_LOG_DEBUG(x)
#endif

#if ITSDK_DRIVERS_ACCEL_LIS2DH_LOG_LEVEL >= __LOG_LEVEL_STANDARD && (ITSDK_LOGGER_MODULE & __LOG_MOD_ACCEL) > 0
#define LIS2DH_LOG_INFO(x) log_info x
#else
#define LIS2DH_LOG_INFO(x)
#endif

#if ITSDK_DRIVERS_ACCEL_LIS2DH_LOG_LEVEL >= __LOG_LEVEL_QUIET && (ITSDK_LOGGER_MODULE & __LOG_MOD_ACCEL) > 0
#define LIS2DH_LOG_WARN(x) log_warn x
#else
#define LIS2DH_LOG_WARN(x)
#endif

#if ITSDK_DRIVERS_ACCEL_LIS2DH_LOG_LEVEL >= __LOG_LEVEL_CRITIAL_ONLY && (ITSDK_LOGGER_MODULE & __LOG_MOD_ACCEL) > 0
#define LIS2DH_LOG_ERROR(x) log_error x
#else
#define LIS2DH_LOG_ERROR(x)
#endif

#if ITSDK_DRIVERS_ACCEL_LIS2DH_LOG_LEVEL >= __LOG_LEVEL_STANDARD && (ITSDK_LOGGER_MODULE & __LOG_MOD_ACCEL) > 0
#define LIS2DH_LOG_ANY(x) log_info x
#else
#define LIS2DH_LOG_ANY(x)
#endif



#endif /* SRC_DRIVERS_ACCEL_ST_LIS2DH12_LIS2DH12_H_ */
