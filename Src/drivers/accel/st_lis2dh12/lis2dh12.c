/* ==========================================================
 * lis2dh12.c - ST accelerometer lis2dh12 driver
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

/** Based on ST MicroElectronics LIS2DH datasheet http://www.st.com/web/en/resource/technical/document/datasheet/DM00042751.pdf
  * 30/09/2014 by Conor Forde <me@conorforde.com>
  * Updates should be available at https://github.com/Snowda/LIS2DH
  *
  * Changelog:
  *     ... - ongoing development release
  *     ... - May 2018 - Update by Disk91 / Paul Pinault to make it working
  *           maintained at https://github.com/disk91/LIS2DH
  * NOTE: THIS IS ONLY A PARIAL RELEASE.
  * THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE DEVELOPMENT AND IS MISSING MOST FEATURES.
  * PLEASE KEEP THIS IN MIND IF YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.
  */
#include <it_sdk/config.h>
#if ITSDK_WITH_DRIVERS == __ENABLE

#include <it_sdk/configDrivers.h>
#if ITSDK_DRIVERS_ACCEL_LIS2DH12 == __ENABLE
#include <math.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/config_defines.h>
#include <drivers/accel/st_lis2dh12/lis2dh12.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/logger/logger.h>


// =============================================================================================
// Power consumption information
// =============================================================================================
// Default configuration : 188uA (bootup, no setting)
// Over consumption was due to SA0 set to low ... in this case a pull-up resistor is consuming energy
// Undocumented register 0 should fix that but I did not success...
//
// With the default configuration the power consumption is about 130uA, far away from spec any info
// on how to reduce power consumption is welcome.

drivers_lis2dh12_conf_t __lis2dh_conf;

// =============================================================================================
// HIGH LEVEL FUNCTIONS
// =============================================================================================

// -----------------------------------------------------
// INIT
// -----------------------------------------------------

/**
 * Set the LIS2DH chip address based on the configuration selected for
 * SA0 pin.
 */

drivers_lis2dh12_ret_e lis2dh_init() {
  return lis2dh_init_i(LIS2DH_RESOLUTION_MODE_8B,LIS2DH_FREQUENCY_10HZ,LIS2DH_SCALE_FACTOR_2G);
}

drivers_lis2dh12_ret_e lis2dh_init_i(
		drivers_lis2dh12_resolution_e resolution,
		drivers_lis2dh12_frequency_e frequency,
		drivers_lis2dh12_scale_e scale
) {
    drivers_lis2dh12_ret_e ret = LIS2DH_SUCCESS;
    __lis2dh_conf._address = ITSDK_DRIVERS_ACCEL_LIS2DH_ADDRESS;

    if ( ( ITSDK_DRIVERS_ACCEL_LIS2DH_ADDRESS & __ACCEL_LIS2DH12_SA0_HIGH ) == 0 ) {
    	// SA0 is low
        // Not working way to disable pull-down
        // __lis2dh_writeMaskedRegisterI(LIS2DH_CTRL_REG0, LIS2DH_REG0_SA0PULLUP_MASK, LIS2DH_REG0_SA0PULLUP_DISABLE);
    }

    if ( lis2dh_whoAmI() == LIS2DH_SUCCESS ) {
      // connection success

      // set a default configuration with 10Hz - Low Power ( 8b resolution )
      ret |= lis2dh_setDataRate(frequency);
      ret |= lis2dh_setResolutionMode(resolution);
      ret |= lis2dh_setAccelerationScale(scale);

      // shutdown interrupt
      ret |= lis2dh_disableAllInterrupt();

      // disable temperature
      ret |= lis2dh_setTempEnabled(BOOL_FALSE);

      // enable all the axis
      ret |= lis2dh_enableAxisXYZ();
      ret |= lis2dh_setFiFoMode(LIS2DH_FM_STREAM);
      ret |= lis2dh_enableFifo(BOOL_TRUE);

      // set highpass filter (Reset when reading xhlREFERENCE register)
      // activate High Pass filter on Output and Int1
      ret |= lis2dh_setHPFilterMode(LIS2DH_HPM_NORMAL_RESET);
      ret |= lis2dh_setHPFilterCutOff(LIS2DH_HPCF_ODR_50); // 0.2Hz high pass Fcut
      ret |= lis2dh_enableHPFDS();
      ret |= lis2dh_disableHPIA1();
      ret |= lis2dh_enableHPIA2();
      ret |= lis2dh_enableHPClick();

    } else {
      ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_LIS2DH_NOTFOUND,0);
      LIS2DH_LOG_ERROR(("LIS2DH Init failed : not found\r\n"));
      return LIS2DH_FAILED;
    }
    return ret;
}



/**
 * Reinit is not changing the device configuration but is preparing
 * the lisd2h12 software structure to be used. This is used when the
 * MCU is siwtched off when the lis2dh was running in background.
 * The MCU is waking up, calling this function to be prepared to
 * get data from the LIS2DH. Config is obtained from the LIS2DH
 */
drivers_lis2dh12_ret_e lis2dh_reinit() {
    drivers_lis2dh12_ret_e ret = LIS2DH_SUCCESS;
    if ( lis2dh_whoAmI() ) {
      __lis2dh_readSetting();
    }else {
      ITSDK_ERROR_REPORT(ITSDK_ERROR_DRV_LIS2DH_NOTFOUND,0);
      LIS2DH_LOG_ERROR(("LIS2DH FRe-Init failed : not found\r\n"));
    }
    return ret;
}


/**
 * Restore the frequency, scale & resolution from the
 * register - used when the LIS2DH structure has been
 * resetted but device still configured
 */
void __lis2dh_readSetting() {
	__lis2dh_conf._scale = lis2dh_getAccelerationScale();
	__lis2dh_conf._frequency = lis2dh_getDataRate();
	__lis2dh_conf._resolution = lis2dh_getResolutionMode();
	__lis2dh_conf._fifoMode = lis2dh_getFiFoMode();
	__lis2dh_conf._hpcf = lis2dh_getHPFilterCutOff();
}


void lis2dh_dumpConfig(void) {
  LIS2DH_LOG_ANY(("---LIS2DH Config---\r\n"));
  LIS2DH_LOG_ANY(("CTRL_REG1 : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_CTRL_REG1)));
  LIS2DH_LOG_ANY(("CTRL_REG2 : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_CTRL_REG2)));
  LIS2DH_LOG_ANY(("CTRL_REG3 : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_CTRL_REG3)));
  LIS2DH_LOG_ANY(("CTRL_REG4 : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_CTRL_REG4)));
  LIS2DH_LOG_ANY(("CTRL_REG5 : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_CTRL_REG5)));
  LIS2DH_LOG_ANY(("CTRL_REG6 : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_CTRL_REG6)));
  LIS2DH_LOG_ANY(("STATUS_REG : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_STATUS_REG2)));
  LIS2DH_LOG_ANY(("FIFO_CTRL_REG : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_FIFO_CTRL_REG)));
  LIS2DH_LOG_ANY(("FIFO_SRC_REG : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_FIFO_SRC_REG)));
  LIS2DH_LOG_ANY(("INT1_CFG : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_INT1_CFG)));
  LIS2DH_LOG_ANY(("INT1_THS : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_INT1_THS)));
  LIS2DH_LOG_ANY(("INT2_CFG : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_INT2_CFG)));
  LIS2DH_LOG_ANY(("INT2_THS : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_INT2_THS)));


  LIS2DH_LOG_ANY(("INT1_SRC : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_INT1_SOURCE)));
  LIS2DH_LOG_ANY(("INT2_SRC : 0x%02X\r\n",__lis2dh_readRegister(LIS2DH_INT2_SOURCE)));

  LIS2DH_LOG_ANY(("Resolution : %d\r\n",__lis2dh_conf._resolution));
  LIS2DH_LOG_ANY(("Frequency : %d\r\n",__lis2dh_conf._frequency));
  LIS2DH_LOG_ANY(("Scale : %d\r\n",__lis2dh_conf._scale));
  LIS2DH_LOG_ANY(("FiFo Mode : %d\r\n",__lis2dh_conf._fifoMode));
  LIS2DH_LOG_ANY(("Odr : %d\r\n",__lis2dh_conf._hpcf));
}


// -----------------------------------------------------
// Acceleration & Motion access
// -----------------------------------------------------


/**
 * Read the Fifo buffer until empty. Load the result into a
 * int8_t/int16_t [size][3] table and return the number of element
 * loaded in the table.
 * The bypass flag indicate if the fifo is activated of not
 * The resolution indicates how to read the data and also the buffer
 * type.
 */
uint8_t lis2dh_getPendingMotions( int16_t * _buffer, uint8_t size) {
  return __lis2dh_readFifo(_buffer,size);
}

/**
 * Return the FiFo content like for getPendingMotions but the result
 * is in mG taking into consideration the device configuration
 * return the fifo size really read
 */
uint8_t lis2dh_getPendingAcceleration( int16_t * _buffer, uint8_t size) {
  uint8_t sz=__lis2dh_readFifo(_buffer,size);
  int16_t (*buffer16)[3] = (int16_t (*)[3]) _buffer;
  for ( int i = 0 ; i < sz ; i++ ) {
    __lis2dh_getAcceleration(&buffer16[i][0],&buffer16[i][1],&buffer16[i][2]);
  }
  return sz;
}

/**
 * Instead of returning the acceleration on the 3 axis, this function
 * returns the force of the acceleration. only one value is returned
 * for all the axes.
 */
uint8_t lis2dh_getPendingForces( int16_t * _buffer, uint8_t size) {
  int16_t buffer16[32][3];
  uint32_t _force;
  uint8_t sz=__lis2dh_readFifo((int16_t *)buffer16,size);
  for ( int i = 0 ; i < sz ; i++ ) {
    __lis2dh_getAcceleration(&buffer16[i][0],&buffer16[i][1],&buffer16[i][2]);
    _force = (int32_t)buffer16[i][0]*buffer16[i][0] + (int32_t)buffer16[i][1]*buffer16[i][1] + (int32_t)buffer16[i][2]*buffer16[i][2];
    _force = sqrt(_force);
    _buffer[i] = _force;
  }
  return sz;
}


// -----------------------------------------------------
// BACKGROUND TILT DETECTION
// -----------------------------------------------------

/**
 * Set lis2dh interrupt 2 to be fired on the given force.
 * The interrupt will be kept until a call to hasTiltDectected()
 * The captured duration will depends on the _frequency parameter
 * from 1/10s. by default 100ms
 */
drivers_lis2dh12_ret_e lis2dh_runBackgroundTiltDetection(uint16_t forceMg) {
  drivers_lis2dh12_ret_e ret = LIS2DH_FAILED;
  if ( lis2dh_setInterruptThresholdMg(LIS2DH_INTERRUPT2, forceMg, __lis2dh_conf._scale) ) {
      ret  = lis2dh_enableLatchInterrupt(LIS2DH_INTERRUPT2, BOOL_TRUE);
      ret |= lis2dh_intWorkingMode(LIS2DH_INTERRUPT2, LIS2DH_INT_MODE_OR);
      ret |= lis2dh_enableInterruptEvent(LIS2DH_INTERRUPT2, LIS2DH_INTEVENT_Z_HIGH | LIS2DH_INTEVENT_Y_HIGH | LIS2DH_INTEVENT_X_HIGH);
  } else {
    LIS2DH_LOG_ERROR(("LIS2DH - runBackgroundTiltDetection - Invalid forceMg value\r\n"));
  }
  return ret;
}

drivers_lis2dh12_ret_e lis2dh_hasTiltDetected() {
  uint8_t status = __lis2dh_readRegister(LIS2DH_INT2_SOURCE);
  return ( (status & LIS2DH_INT_IA_MASK) > 0 );
}

// -----------------------------------------------------
// 6D Positions
// -----------------------------------------------------

/**
 * Init 6D position detection on INT1
 * With automatic restart every 1 S
 */
drivers_lis2dh12_ret_e lis2dh_initPosition6D() {
   drivers_lis2dh12_ret_e ret;
   ret  = lis2dh_intWorkingMode(LIS2DH_INTERRUPT1, LIS2DH_INT_MODE_POS);
   ret |= lis2dh_enableInterruptEvent(LIS2DH_INTERRUPT1, LIS2DH_INTEVENT_ALL);
   ret |= lis2dh_enableLatchInterrupt(LIS2DH_INTERRUPT1, BOOL_FALSE);
   ret |= lis2dh_setInterruptThresholdMg(LIS2DH_INTERRUPT1, 100, __lis2dh_conf._scale);
   ret |= lis2dh_setInterruptDurationMs(LIS2DH_INTERRUPT1, 1000, __lis2dh_conf._frequency);
   return ret;
}

/**
 * Read the position and return it when stable
 * Otherwize return LIS2DH_POSITION_TOP_ON_NONE
 */
uint8_t lis2dh_getPosition6D() {
  uint8_t position = __lis2dh_readRegister(LIS2DH_INT1_SOURCE);
  if ( (position & LIS2DH_INT_IA_MASK) > 0 ) {
    return ( position & LIS2DH_INT_POS_MASK );
  }
  return LIS2DH_POSITION_TOP_ON_NONE;
}



// =============================================================================================
// LOW LEVEL FUNCTIONS
// =============================================================================================


// -----------------------------------------------------
// Data Access
// -----------------------------------------------------


/** Read the X axis registers
 * @see LIS2DH_OUT_X_H
 * @see LIS2DH_OUT_X_L
 */
int16_t lis2dh_getAxisX(void) {
  return __lis2dh_readRegisters(LIS2DH_OUT_X_H, LIS2DH_OUT_X_L);
}


/** Read the Y axis registers
 * @see LIS2DH_OUT_Y_H
 * @see LIS2DH_OUT_Y_L
 */
int16_t lis2dh_getAxisY(void) {
  return __lis2dh_readRegisters(LIS2DH_OUT_Y_H, LIS2DH_OUT_Y_L);
}

/** Read the Z axis registers
 * @see LIS2DH_OUT_Z_H
 * @see LIS2DH_OUT_Z_L
 */
int16_t lis2dh_getAxisZ(void) {
  return __lis2dh_readRegisters(LIS2DH_OUT_Z_H, LIS2DH_OUT_Z_L);
}


/** Read the X axis registers ( 8bit mode)
 *  As data are left justified, the result for 8bits
 *  is in the High register
 * @see LIS2DH_OUT_X_H
 */
int8_t lis2dh_getAxisX_LR(void) {
  return __lis2dh_readRegister(LIS2DH_OUT_X_H);
}


/** Read the Y axis registers ( 8bit mode)
 *  As data are left justified, the result for 8bits
 *  is in the High register
 * @see LIS2DH_OUT_Y_H
 */
int8_t lis2dh_getAxisY_LR(void) {
  return __lis2dh_readRegister(LIS2DH_OUT_Y_H);
}

/** Read the Z axis registers ( 8bit mode)
 *  As data are left justified, the result for 8bits
 *  is in the High register
 * @see LIS2DH_OUT_Z_H
 */
int8_t lis2dh_getAxisZ_LR(void) {
  return __lis2dh_readRegister(LIS2DH_OUT_Z_H);
}

/** Read the all axis registers
 * @see getAxisZ()
 * @see getAxisY()
 * @see getAxisZ()
 */
void lis2dh_getMotion(int16_t* ax, int16_t* ay, int16_t* az) {
    *ax = lis2dh_getAxisX();
    *ay = lis2dh_getAxisY();
    *az = lis2dh_getAxisZ();
}

void lis2dh_getMotion_LR(int8_t* ax, int8_t* ay, int8_t* az) {
    *ax = lis2dh_getAxisX_LR();
    *ay = lis2dh_getAxisY_LR();
    *az = lis2dh_getAxisZ_LR();
}






// ======= Temperature

/**
 * Temperature is 8bit usually but it switch to 10bits if LPen bit in CTRL_REG1 is
 * cleared (high resolution mode). In this mode the way to decode the temperature
 * is not documented in the datasheet. Test and function modification will be needed
 * ...Temperature refresh is ODR.
 * @TODO ...
 * When the Temperature enable flag is on, the over consumption is about 40uA
 */

drivers_lis2dh12_ret_e lis2dh_getTempEnabled(void) {
    return (__lis2dh_readMaskedRegister(LIS2DH_TEMP_CFG_REG, LIS2DH_TEMP_EN_MASK) != 0);
}

drivers_lis2dh12_ret_e lis2dh_setTempEnabled(itsdk_bool_e enable) {
    drivers_lis2dh12_ret_e ret = __lis2dh_writeRegister(LIS2DH_TEMP_CFG_REG, enable ? LIS2DH_TEMP_EN_MASK : 0);
    ret     |= __lis2dh_writeMaskedRegisterI(LIS2DH_CTRL_REG4, LIS2DH_BDU_MASK, enable ? LIS2DH_BDU_MASK : 0);
    return ret;
}

/**
 * Return the temperature. The LIS2DH does not gives an absolute temperature but a relative
 * Temperature. The Datasheet or (Datashit ?) does not define the temperature is relative to.
 * This is a try to a relative 25°C as this Temperature has been printed in the Datasheet.
 */
int16_t lis2dh_getTemperature(void) {
    int i = 0;
    while ( !lis2dh_tempDataAvailable() && i < 200 ) { i++; itsdk_delayMs(1); }
    if ( i < 200 ) {
        int16_t t = (int16_t)(__lis2dh_readRegisters(LIS2DH_OUT_TEMP_H, LIS2DH_OUT_TEMP_L));
        int shift = 0;
        if ( __lis2dh_conf._resolution == LIS2DH_RESOLUTION_8B ) {
          shift=8;
        } else {
          shift = ( __lis2dh_conf._resolution == LIS2DH_RESOLUTION_10B)?6:4;
        }
        return 25 + (t >> shift);
    } else {
        //if new data isn't available - timeout
        return LIS2DH_TEMPERATURE_INVALID;
    }
}


drivers_lis2dh12_ret_e lis2dh_tempHasOverrun(void) {
    uint8_t overrun = __lis2dh_readMaskedRegister(LIS2DH_STATUS_REG_AUX, LIS2DH_TOR_MASK);
    return (overrun != 0)?LIS2DH_SUCCESS:LIS2DH_FAILED;
}

drivers_lis2dh12_ret_e lis2dh_tempDataAvailable(void) {
    uint8_t data = __lis2dh_readMaskedRegister(LIS2DH_STATUS_REG_AUX, LIS2DH_TDA_MASK);
    return (data != 0)?LIS2DH_SUCCESS:LIS2DH_FAILED;
}


// ======== Data Rate

drivers_lis2dh12_frequency_e lis2dh_getDataRate(void) {
    uint8_t v = __lis2dh_readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_ODR_MASK);
    v >>= LIS2DH_ODR_SHIFT;
    return (drivers_lis2dh12_frequency_e)v;
}

drivers_lis2dh12_ret_e lis2dh_setDataRate(drivers_lis2dh12_frequency_e data_rate) {
    if ( data_rate > LIS2DH_ODR_MAXVALUE ) return LIS2DH_FAILED;
    __lis2dh_conf._frequency = data_rate;
    data_rate <<= LIS2DH_ODR_SHIFT;
    return __lis2dh_writeMaskedRegisterI(LIS2DH_CTRL_REG1, LIS2DH_ODR_MASK, data_rate);
}

// ========= Power Management

drivers_lis2dh12_ret_e lis2dh_enableLowPower(void) {
    drivers_lis2dh12_ret_e ret = __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK, BOOL_TRUE);
    ret |= lis2dh_setHighResolutionMode(BOOL_FALSE);
    return ret;
}


drivers_lis2dh12_ret_e lis2dh_disableLowPower(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK, BOOL_FALSE);
}


itsdk_bool_e lis2dh_isLowPowerEnabled(void) {
    return (    __lis2dh_readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_LPEN_MASK) != 0
             && lis2dh_isHighResolutionMode() == BOOL_TRUE)?BOOL_TRUE:BOOL_FALSE;
}

/**
 *  Power down the device, power up is made by changing the
 *  DataRate to another value.
 *
 */
drivers_lis2dh12_ret_e lis2dh_setPowerDown(void) {
    return lis2dh_setDataRate(LIS2DH_ODR_POWER_DOWN);
}

/**
 * Select the activity level that wake the sensor up from low_power mode.
 * This is for mode other than low_power. When this level of activity is identify
 * the sensor is waking up to the normal mode you set. Under, it goes to low-power mode
 * The unit depends on the Scale factor. you can use setSleepNWakeUpThresholdMg instead
 */
drivers_lis2dh12_ret_e lis2dh_setSleepNWakeUpThreshold(uint8_t raw) {
  if(raw > LIS2DH_ACT_THS_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  raw = raw << LIS2DH_ACT_THS_SHIFT;
  return __lis2dh_writeMaskedRegisterI(LIS2DH_ACT_THS, LIS2DH_ACT_THS_MASK, raw);
}

/**
 * Select the activity level that wake the sensor up from low-power mode in Mg.
 * The scale is given as parameter LIS2DH_FS_SCALE_ 2/4/8/16 G
 * Step is 16mG @ 2G / 32mG @ 4G / 62mG @ 8g / 186mG à 16G
 */
drivers_lis2dh12_ret_e lis2dh_setSleepNWakeUpThresholdMg( uint8_t mg, const uint8_t scale) {
  uint8_t raw = 0;
  if ( scale > LIS2DH_FS_MAXVALUE ) return LIS2DH_FAILED;

  if ( __lis2dh_convertMgToRaw(&raw, mg, scale) ) {
    return lis2dh_setSleepNWakeUpThreshold(raw);
  }
  return LIS2DH_FAILED;
}

/**
 * Select the Activity duration for low-power to wake up and return to low-power duration
 * This is for mode other than low_power.
 * The unit depends on the ODR factor. see LIS2DH_ODR_ 1/10/25...1620 HZ
 * You can use setSleepNWakeUpDurationMs instead
 */
drivers_lis2dh12_ret_e lis2dh_setSleepNWakeUpDuration(uint8_t raw) {
  if(raw > LIS2DH_ACT_DUR_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  raw = raw << LIS2DH_ACT_DUR_SHIFT;
  return __lis2dh_writeMaskedRegisterI(LIS2DH_ACT_DUR, LIS2DH_ACT_DUR_MASK, raw);
}

/**
 * Select the Sleep and Wake-up activity duration from a duration given in Ms
 * The duration is S = (8*rawValue+1) / ODR
 */
drivers_lis2dh12_ret_e lis2dh_setSleepNWakeUpDurationMs(uint8_t _int, uint32_t ms, const uint8_t odr) {
  uint8_t raw = 0;

  // Basically this is an approximation as I did not take +1 in consideration
  ms = ms>>3;

  if ( __lis2dh_convertMsToRaw(&raw, ms, odr) ) {
    if ( raw > 0 ) raw--; // this is to take -1 in consideration.
    return lis2dh_setInterruptDuration(_int,raw);
  } else {
    return LIS2DH_FAILED;
  }
}



// ========== Resolution mode

/**
 * Set the high Resolution mode
 * HR mode
 */
drivers_lis2dh12_ret_e lis2dh_setHighResolutionMode(itsdk_bool_e hr) {
  if ( hr) __lis2dh_conf._resolution=LIS2DH_RESOLUTION_12B;
  return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG4,LIS2DH_HR_MASK,hr);
}


itsdk_bool_e lis2dh_isHighResolutionMode() {
  return ( __lis2dh_readMaskedRegister(LIS2DH_CTRL_REG4,LIS2DH_HR_MASK) != 0 )?BOOL_TRUE:BOOL_FALSE;
}


/**
 * Set the expected acceleration resolution in bit
 * Possible option : LIS2DH_RESOLUTION_XXB (8,10,12)
 * This is changing the LowPower mode and HighResolution mode
 */
drivers_lis2dh12_ret_e lis2dh_setResolutionMode(drivers_lis2dh12_resolution_e res) {
  if (res > LIS2DH_RESOLUTION_MAXVALUE) return LIS2DH_FAILED;
  drivers_lis2dh12_ret_e ret;
  __lis2dh_conf._resolution=res;
  switch (res) {
    default:
    case LIS2DH_RESOLUTION_MODE_8B:
          ret = lis2dh_setHighResolutionMode(BOOL_FALSE);
          ret |= lis2dh_enableLowPower();
          return ret;
    case LIS2DH_RESOLUTION_MODE_10B:
          ret = lis2dh_disableLowPower();
          ret |= lis2dh_setHighResolutionMode(BOOL_FALSE);
          return ret;
    case LIS2DH_RESOLUTION_MODE_12B:
          ret = lis2dh_disableLowPower();
          ret |= lis2dh_setHighResolutionMode(BOOL_TRUE);
          return ret;
  }
}


drivers_lis2dh12_resolution_e lis2dh_getResolutionMode() {
  if ( lis2dh_isHighResolutionMode() == BOOL_TRUE ) return LIS2DH_RESOLUTION_MODE_12B;
  else if ( lis2dh_isLowPowerEnabled() == BOOL_TRUE ) return LIS2DH_RESOLUTION_MODE_8B;
  else return LIS2DH_RESOLUTION_MODE_10B;
}

// ========== Axis management

drivers_lis2dh12_ret_e lis2dh_enableAxisX(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_X_EN_MASK, BOOL_TRUE);
}

drivers_lis2dh12_ret_e lis2dh_disableAxisX(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_X_EN_MASK, BOOL_FALSE);
}

itsdk_bool_e lis2dh_isXAxisEnabled(void) {
    return (__lis2dh_readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_X_EN_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}

drivers_lis2dh12_ret_e lis2dh_enableAxisY(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_Y_EN_MASK, BOOL_TRUE);
}

drivers_lis2dh12_ret_e lis2dh_disableAxisY(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_Y_EN_MASK, BOOL_FALSE);
}

itsdk_bool_e lis2dh_isYAxisEnabled(void) {
    return (__lis2dh_readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_Y_EN_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}

drivers_lis2dh12_ret_e lis2dh_enableAxisZ(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_Z_EN_MASK, BOOL_TRUE);
}

drivers_lis2dh12_ret_e lis2dh_disableAxisZ(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_Z_EN_MASK, BOOL_FALSE);
}

itsdk_bool_e lis2dh_isZAxisEnabled(void) {
    return (__lis2dh_readMaskedRegister(LIS2DH_CTRL_REG1, LIS2DH_Z_EN_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}

drivers_lis2dh12_ret_e lis2dh_enableAxisXYZ(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_XYZ_EN_MASK, BOOL_TRUE);
}

drivers_lis2dh12_ret_e lis2dh_disableAxisXYZ(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG1, LIS2DH_XYZ_EN_MASK, BOOL_FALSE);
}

// ====== HIGH Pass filter mode
// see http://www.st.com/content/ccc/resource/technical/document/application_note/60/52/bd/69/28/f4/48/2b/DM00165265.pdf/files/DM00165265.pdf/jcr:content/translations/en.DM00165265.pdf
// HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
//  * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
//  * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
//  * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
//  * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz


/*
 * Enable/Disable High Pass Filter standard output
 */
drivers_lis2dh12_ret_e lis2dh_enableHPFDS(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_FDS_MASK, BOOL_TRUE);
}

drivers_lis2dh12_ret_e lis2dh_disableHPFDS(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_FDS_MASK, BOOL_FALSE);
}

itsdk_bool_e lis2dh_isHPFDSEnabled(void) {
    return (__lis2dh_readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_FDS_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}

/**
 * High Pass filter allow to remove the continous component to only keep the
 * dynamic acceleration => basically it remove the gravity... as any stable
 * acceleration
 */
uint8_t lis2dh_getHPFilterMode() {
    uint8_t v = __lis2dh_readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPM_MASK);
    v >>= LIS2DH_HPM_SHIFT;
    return v;
}

drivers_lis2dh12_ret_e lis2dh_setHPFilterMode(uint8_t mode) {
    if(mode > LIS2DH_HPM_MAXVALUE) {
        return LIS2DH_FAILED;
    }
    uint8_t filter_mode = mode << LIS2DH_HPM_SHIFT;
    return __lis2dh_writeMaskedRegisterI(LIS2DH_CTRL_REG2, LIS2DH_HPM_MASK, filter_mode);
}

/*
 * Cut-Off Frequency
 * the reference is the ODR frequency (acquisition frequency), the Cut Off
 * frequency is this frequency divide par a given number from 9 to 400
 */
drivers_lis2dh12_hpcf_e lis2dh_getHPFilterCutOff() {
    uint8_t mode = __lis2dh_readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPM_MASK);
    mode >>= LIS2DH_HPCF_SHIFT;
    return (drivers_lis2dh12_hpcf_e)mode;
}

drivers_lis2dh12_ret_e lis2dh_setHPFilterCutOff(drivers_lis2dh12_hpcf_e mode) {
    if(mode > LIS2DH_HPCF_MAXVALUE) {
        return LIS2DH_FAILED;
    }
    __lis2dh_conf._hpcf = mode;
    uint8_t fcut = mode << LIS2DH_HPCF_SHIFT;
    return __lis2dh_writeMaskedRegisterI(LIS2DH_CTRL_REG2, LIS2DH_HPCF_MASK, fcut);
}

/*
 * Enable/Disable High Pass Filter on Click
 */
drivers_lis2dh12_ret_e lis2dh_enableHPClick(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPCLICK_MASK, BOOL_TRUE);
}

drivers_lis2dh12_ret_e lis2dh_disableHPClick(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPCLICK_MASK, BOOL_FALSE);
}

itsdk_bool_e lis2dh_isHPClickEnabled(void) {
    return (__lis2dh_readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPCLICK_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}

/*
 * Enable/Disable High Pass Filter on Interrupt 2
 */
drivers_lis2dh12_ret_e lis2dh_enableHPIA2(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIA2_MASK, BOOL_TRUE);
}

drivers_lis2dh12_ret_e lis2dh_disableHPIA2(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIA2_MASK, BOOL_FALSE);
}

itsdk_bool_e lis2dh_isHPIA2Enabled(void) {
    return (__lis2dh_readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPIA2_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}

/*
 * Enable/Disable High Pass Filter on Interrupt 1
 */
drivers_lis2dh12_ret_e lis2dh_enableHPIA1(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIA1_MASK, BOOL_TRUE);
}

drivers_lis2dh12_ret_e lis2dh_disableHPIA1(void) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG2, LIS2DH_HPIA1_MASK, BOOL_FALSE);
}

itsdk_bool_e lis2dh_isHPIA1Enabled(void) {
    return (__lis2dh_readMaskedRegister(LIS2DH_CTRL_REG2, LIS2DH_HPIA1_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}

// ========== INTERRUPT MANAGEMENT

/**
 * Enable / Disable an interrupt source on INT1
 */
drivers_lis2dh12_ret_e lis2dh_enableInterruptInt1(uint8_t _int) {
  return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG3,_int,BOOL_TRUE);
}

drivers_lis2dh12_ret_e lis2dh_disableInterruptInt1(uint8_t _int) {
  return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG3,_int,BOOL_FALSE);
}

/**
 * Enable / Disable an interrupt source on INT2
 */
drivers_lis2dh12_ret_e lis2dh_enableInterruptInt2(uint8_t _int) {
  return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG6,_int,BOOL_TRUE);
}

drivers_lis2dh12_ret_e lis2dh_disableInterruptInt2(uint8_t _int) {
  return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG6,_int,BOOL_FALSE);
}

/**
 * Disable all interrupts
 */
drivers_lis2dh12_ret_e lis2dh_disableAllInterrupt() {
  drivers_lis2dh12_ret_e ret = __lis2dh_writeRegister(LIS2DH_CTRL_REG3,LIS2DH_I1_INTERRUPT_NONE);
  ret |= __lis2dh_writeMaskedRegisterI(LIS2DH_CTRL_REG6,LIS2DH_I2_MASK,LIS2DH_I2_INTERRUPT_NONE);
  return ret;
}

/**
 * Set the interrupt polarity
 */
drivers_lis2dh12_ret_e lis2dh_setInterruptPolarity(drivers_lis2dh12_interruptPol_e polarity) {
  switch ( polarity ) {
    case LIS2DH_INTERRUPT_POLARITY_HIGH:
      return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG6,LIS2DH_INT_POLARITY,BOOL_FALSE);
    case LIS2DH_INTERRUPT_POLARITY_LOW:
      return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG6,LIS2DH_INT_POLARITY,BOOL_TRUE);
    default:
      return LIS2DH_FAILED;
  }
}


/**
 * Latch Interrupt 1/2 => the interrupt is only cleared when INT1_SRC / INT2_SRC register is read
 */
drivers_lis2dh12_ret_e lis2dh_enableLatchInterrupt(uint8_t _int, itsdk_bool_e enable) {
  if ( _int > 2 || _int < 1 ) return LIS2DH_FAILED;
  switch (_int) {
    case 1 :
        return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG5,LIS2DH_LIR_INT1_MASK,enable);
    case 2 :
        return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG5,LIS2DH_LIR_INT2_MASK,enable);
  }
  return LIS2DH_FAILED;
}

/**
 * Select trigger event trigger signal on INT1/2
 * See LIS2DH_TR_XXX
 */
drivers_lis2dh12_ret_e lis2dh_triggerSelect(drivers_lis2dh12_triggers_e triggerMode) {
  if(triggerMode > LIS2DH_TR_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  triggerMode = triggerMode << LIS2DH_TR_SHIFT;
  return __lis2dh_writeMaskedRegisterI(LIS2DH_FIFO_CTRL_REG, LIS2DH_TR_MASK, (uint8_t)triggerMode);
}

/**
 * Select the interruption mode
 * See mode : LIS2DH_INT_MODE_XXX
 * Default mode is OR
 * Give the interrupt 1 for INT1 and 2 for INT2
 */
drivers_lis2dh12_ret_e lis2dh_intWorkingMode(uint8_t _int, drivers_lis2dh12_intmode_e _mode) {
  if(_mode > LIS2DH_INT_MODE_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  _mode = _mode << LIS2DH_INT_MODE_SHIFT;
  if ( _int > 2 || _int < 1 ) return LIS2DH_FAILED;
  switch (_int) {
    case 1 :
        return __lis2dh_writeMaskedRegisterI(LIS2DH_INT1_CFG, LIS2DH_INT_MODE_MASK, (uint8_t) _mode);
    case 2 :
        return __lis2dh_writeMaskedRegisterI(LIS2DH_INT2_CFG, LIS2DH_INT_MODE_MASK, (uint8_t) _mode);
  }
  return LIS2DH_FAILED;
}

/**
 * Enable the interrupt events. See list
 * It is possible to activate multiple interrupt event by adding them with | operator
 * See LIS2DH_INTEVENT_XX possible events
 * Give the interrupt 1 for INT1 and 2 for INT2
 */
drivers_lis2dh12_ret_e lis2dh_enableInterruptEvent(uint8_t _int, drivers_lis2dh12_intevent_e _intEvent) {
  if(_intEvent > LIS2DH_INTEVENT_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  _intEvent = _intEvent << LIS2DH_INTEVENT_SHIFT;
  if ( _int > 2 || _int < 1 ) return LIS2DH_FAILED;
  switch (_int) {
    case 1 :
        return __lis2dh_writeMaskedRegisterI(LIS2DH_INT1_CFG, LIS2DH_INTEVENT_MASK, (uint8_t)_intEvent);
    case 2 :
        return __lis2dh_writeMaskedRegisterI(LIS2DH_INT2_CFG, LIS2DH_INTEVENT_MASK, (uint8_t)_intEvent);
  }
  return LIS2DH_FAILED;
}

/**
 * Get interrupt status
 * Give the interrupt 1 for INT1 and 2 for INT2 as parameter
 */
itsdk_bool_e lis2dh_isInterruptFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return BOOL_FALSE;
  switch(_int) {
    case 1: return (__lis2dh_readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_INT_IA_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
    case 2: return (__lis2dh_readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_INT_IA_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
  }
  return LIS2DH_FAILED;
}

itsdk_bool_e lis2dh_isInterruptZHighFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return BOOL_FALSE;
  switch(_int) {
    case 1: return (__lis2dh_readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_ZH_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
    case 2: return (__lis2dh_readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_ZH_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
  }
  return LIS2DH_FAILED;
}

itsdk_bool_e lis2dh_isInterruptZLowFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return BOOL_FALSE;
  switch(_int) {
    case 1: return (__lis2dh_readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_ZL_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
    case 2: return (__lis2dh_readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_ZL_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
  }
  return LIS2DH_FAILED;
}

itsdk_bool_e lis2dh_isInterruptYHighFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return BOOL_FALSE;
  switch(_int) {
    case 1: return (__lis2dh_readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_YH_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
    case 2: return (__lis2dh_readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_YH_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
  }
  return LIS2DH_FAILED;
}

itsdk_bool_e lis2dh_isInterruptYLowFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return BOOL_FALSE;
  switch(_int) {
    case 1: return (__lis2dh_readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_YL_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
    case 2: return (__lis2dh_readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_YL_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
  }
  return LIS2DH_FAILED;
}

itsdk_bool_e lis2dh_isInterruptXHighFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return BOOL_FALSE;
  switch(_int) {
    case 1: return (__lis2dh_readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_XH_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
    case 2: return (__lis2dh_readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_XH_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
  }
  return LIS2DH_FAILED;
}

itsdk_bool_e lis2dh_isInterruptXLowFired(uint8_t _int) {
  if ( _int > 2 || _int < 1 ) return BOOL_FALSE;
  switch(_int) {
    case 1: return (__lis2dh_readMaskedRegister(LIS2DH_INT1_SOURCE, LIS2DH_XL_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
    case 2: return (__lis2dh_readMaskedRegister(LIS2DH_INT2_SOURCE, LIS2DH_XL_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
  }
  return LIS2DH_FAILED;
}

/**
 * Set the interrupt Threshold on selected axis
 * his value is used for High and Low value comparison on any axis to generate an interruption
 * This function write raw value. You can use the setInterruptThresholdMg function to set it in mG
 */
drivers_lis2dh12_ret_e lis2dh_setInterruptThreshold(uint8_t _int, uint8_t raw) {
  if(raw > LIS2DH_THS_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  raw = raw << LIS2DH_THS_SHIFT;
  if ( _int > 2 || _int < 1 ) return LIS2DH_FAILED;
  switch (_int) {
    case 1 :
        return __lis2dh_writeMaskedRegisterI(LIS2DH_INT1_THS, LIS2DH_THS_MASK, raw);
    case 2 :
        return __lis2dh_writeMaskedRegisterI(LIS2DH_INT2_THS, LIS2DH_THS_MASK, raw);
  }
  return LIS2DH_FAILED;
}

/**
 * Set the interrupt Threshold for selected axis in Mg. The scale is given as parameter
 * LIS2DH_FS_SCALE_ 2/4/8/16 G
 * Step is 16mG @ 2G / 32mG @ 4G / 62mG @ 8g / 186mG à 16G
 */
drivers_lis2dh12_ret_e lis2dh_setInterruptThresholdMg(uint8_t _int, uint8_t mg, const drivers_lis2dh12_scale_e scale) {
  uint8_t raw = 0;
  if ( scale > LIS2DH_FS_MAXVALUE ) return LIS2DH_FAILED;

  if ( __lis2dh_convertMgToRaw(&raw, mg, scale) ) {
    return lis2dh_setInterruptThreshold(_int,raw);
  }
  return LIS2DH_FAILED;
}

/**
 * Duration of the interrupt, the duration is function
 * of ODR value -> 1 lsb = 1/ODR
 */
drivers_lis2dh12_ret_e lis2dh_setInterruptDuration(uint8_t _int, uint8_t raw) {
  if(raw > LIS2DH_DUR_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  raw = raw << LIS2DH_DUR_SHIFT;
  if ( _int > 2 || _int < 1 ) return LIS2DH_FAILED;
  switch (_int) {
    case 1 :
        return __lis2dh_writeMaskedRegisterI(LIS2DH_INT1_DURATION, LIS2DH_D_MASK, raw);
    case 2 :
        return __lis2dh_writeMaskedRegisterI(LIS2DH_INT2_DURATION, LIS2DH_D_MASK, raw);
  }
  return LIS2DH_FAILED;
}

/**
 * Set the interrupt duration in Ms. The ODR value is given as parameter
 * LIS2DH_ODR_ 1/10/25...1620 HZ
 * step is 1/ODR
 */
drivers_lis2dh12_ret_e lis2dh_setInterruptDurationMs(uint8_t _int, uint32_t ms, const drivers_lis2dh12_frequency_e dr) {
  uint8_t raw = 0;
  if ( __lis2dh_convertMsToRaw(&raw, ms, dr) ) {
    return lis2dh_setInterruptDuration(_int,raw);
  } else {
    return LIS2DH_FAILED;
  }
}


// ========== Misc settings

/**
 * Set little / big endian
 * Only valid when HR mode is on
 */
drivers_lis2dh12_ret_e lis2dh_setLittleEndian() {
  if ( __lis2dh_readMaskedRegister(LIS2DH_CTRL_REG4,LIS2DH_HR_MASK) != 0 ) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG4,LIS2DH_BLE_MASK,BOOL_FALSE);
  }
  return LIS2DH_FAILED;
}

drivers_lis2dh12_ret_e lis2dh_setBitEndian() {
  if ( __lis2dh_readMaskedRegister(LIS2DH_CTRL_REG4,LIS2DH_HR_MASK) != 0 ) {
    return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG4,LIS2DH_BLE_MASK,BOOL_TRUE);
  }
  return LIS2DH_FAILED;
}

/**
 *  Select a Block Data contunious update or an update only once the MSB & LSB has been read
 *  BDU ensure the MSB or MSB value of a sample will not been changed until both MSB and LSB have been read.
 */
drivers_lis2dh12_ret_e lis2dh_setContinuousUpdate(itsdk_bool_e continuous) {
	if ( continuous == BOOL_TRUE ) {
		return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG4,LIS2DH_BDU_MASK,BOOL_FALSE);
	} else {
		return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG4,LIS2DH_BDU_MASK,BOOL_TRUE);
	}
}

/**
 * Set the accelerations scale
 */
drivers_lis2dh12_ret_e lis2dh_setAccelerationScale(drivers_lis2dh12_scale_e scale) {
    if(scale > LIS2DH_FS_MAXVALUE) {
        return LIS2DH_FAILED;
    }
    __lis2dh_conf._scale = scale;
    scale = scale << LIS2DH_FS_SHIFT;
    return __lis2dh_writeMaskedRegisterI(LIS2DH_CTRL_REG4, LIS2DH_FS_MASK, scale);
}

drivers_lis2dh12_scale_e lis2dh_getAccelerationScale() {
    uint8_t v=__lis2dh_readMaskedRegister(LIS2DH_CTRL_REG4,LIS2DH_FS_MASK);
    v >>= LIS2DH_FS_SHIFT;
    return (drivers_lis2dh12_scale_e)v;
}


/**
 *  Reboot memory content
 *  Reload the calibration parameters.
 */
drivers_lis2dh12_ret_e lis2dh_reboot() {
  return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG5,LIS2DH_BOOT_MASK,BOOL_TRUE);
}

/**
 * Return true if the WHOAMI register returned th expected Value
 * 0x33
 */
drivers_lis2dh12_ret_e lis2dh_whoAmI(void) {
    return (LIS2DH_I_AM_VALUE == __lis2dh_readRegister(LIS2DH_WHO_AM_I))?LIS2DH_SUCCESS:LIS2DH_FAILED;
}

/**
 * Enable / Disable Fifo
 */
drivers_lis2dh12_ret_e lis2dh_enableFifo(itsdk_bool_e enable) {
  return __lis2dh_writeMaskedRegister8(LIS2DH_CTRL_REG5,LIS2DH_FIFO_EN_MASK,enable);
}


// ======== REFERENCE used for interrupt generation

drivers_lis2dh12_ret_e lis2dh_setReference(uint8_t ref) {
  return __lis2dh_writeRegister(LIS2DH_REFERENCE,ref);
}

// ========= STATUS
/**
 * Return the Data status bit filed.
 * See LIS2DH_STATUS_XXXX for different possible status
 */
uint8_t lis2dh_getDataStatus() {
  return __lis2dh_readRegister(LIS2DH_STATUS_REG2);
}

// ========= FIFO CTRL

/**
 * Select the FIFO working mode
 * see LIS2DH_FM_XXX possible mode
 */
drivers_lis2dh12_ret_e lis2dh_setFiFoMode(drivers_lis2dh12_fifomode_e fifoMode) {
    if(fifoMode > LIS2DH_FM_MAXVALUE) {
        return LIS2DH_FAILED;
    }
    __lis2dh_conf._fifoMode = fifoMode;
    fifoMode = fifoMode << LIS2DH_FM_SHIFT;
    return __lis2dh_writeMaskedRegisterI(LIS2DH_FIFO_CTRL_REG, LIS2DH_FM_MASK, (uint8_t)fifoMode);
}

/**
 * Get the fifoMode from register
 */
drivers_lis2dh12_fifomode_e lis2dh_getFiFoMode() {
   uint8_t v = __lis2dh_readMaskedRegister(LIS2DH_FIFO_CTRL_REG, LIS2DH_FM_MASK);
   v >>= LIS2DH_FM_SHIFT;
   return (drivers_lis2dh12_fifomode_e)v;
}

/**
 * Set the Fifo Threshold : as soon as this level of data is filled in FiFo the
 * Threshold event is triggered for reading
 * This is used for reading FiFo before it overrun.
 * The value is from 0 to 31.
 */
drivers_lis2dh12_ret_e lis2dh_setFiFoThreshold(uint8_t threshold) {
  if ( threshold > LIS2DH_FTH_MAXVALUE ) return LIS2DH_FAILED;
  threshold <<= LIS2DH_FTH_SHIFT;
  return __lis2dh_writeMaskedRegisterI(LIS2DH_FIFO_CTRL_REG, LIS2DH_FTH_MASK, threshold);
}

/**
 * Return true when FIFO watermark level exceeded
 */
itsdk_bool_e lis2dh_isFiFoWatermarkExceeded() {
  return ( __lis2dh_readMaskedRegister(LIS2DH_FIFO_SRC_REG, LIS2DH_WTM_MASK) > 0 )?BOOL_TRUE:BOOL_FALSE;
}

/**
 * Return true when the FIFO is full => 32 unread samples
 */
itsdk_bool_e lis2dh_isFiFoFull() {
  return ( __lis2dh_readMaskedRegister(LIS2DH_FIFO_SRC_REG, LIS2DH_OVRN_FIFO_MASK) > 0 )?BOOL_TRUE:BOOL_FALSE;
}

/**
 * Return true when the FIFO is empty
 */
itsdk_bool_e lis2dh_isFiFoEmpty() {
  return ( __lis2dh_readMaskedRegister(LIS2DH_FIFO_SRC_REG, LIS2DH_EMPTY_MASK) > 0 )?BOOL_TRUE:BOOL_FALSE;
}

/**
 * Return the number of sample actually in the FiFo
 */
uint8_t lis2dh_getFiFoSize() {
  return __lis2dh_readMaskedRegister(LIS2DH_FIFO_SRC_REG, LIS2DH_FSS_MASK);
}


// ======== CLICK Management

/**
 * Enable the CLICK function interrupt events. See list
 * It is possible to activate multiple interrupt event by adding them with | operator
 * See LIS2DH_CLICEVENT_XXX possible events
 * Give the interrupt 1 for INT1 and 2 for INT2
 */
drivers_lis2dh12_ret_e lis2dh_enableClickEvent(uint8_t _clicEvent) {
  if(_clicEvent > LIS2DH_CLICEVENT_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  _clicEvent = _clicEvent << LIS2DH_CLICEVENT_SHIFT;
  return __lis2dh_writeMaskedRegisterI(LIS2DH_CLICK_CFG, LIS2DH_CLICEVENT_MASK, _clicEvent);
}


/**
 * Get clic interrupt status
 * Get the global information for all the interrupts
 */
itsdk_bool_e lis2dh_isClickInterruptFired() {
  return (__lis2dh_readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_CLK_IA_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}
itsdk_bool_e lis2dh_isDoubleClickFired() {
  return (__lis2dh_readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_DCLICK_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}
itsdk_bool_e lis2dh_isSimpleClickFired() {
  return (__lis2dh_readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_SCLICK_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}
itsdk_bool_e lis2dh_isClickFiredOnZ() {
  return (__lis2dh_readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_Z_CLICK_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}
itsdk_bool_e lis2dh_isClickFiredOnY() {
  return (__lis2dh_readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_Y_CLICK_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}
itsdk_bool_e lis2dh_isClickFiredOnX() {
  return (__lis2dh_readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_X_CLICK_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}
itsdk_bool_e lis2dh_isSignClickFired() {
  return (__lis2dh_readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_SIGN_MASK) != 0)?BOOL_TRUE:BOOL_FALSE;
}

/**
 * From the clic interrupt status, determine the type of pending
 * clic interrupt
 * see LIS2DH_CLIC_XXXX for the possible clic. Multiple clic can be
 * reported. 1bit per clic
 * By reading the register, the insterrup will fall depends on click_ths configuration
 */
uint16_t lis2dh_getClickStatus() {
  uint8_t clic=__lis2dh_readMaskedRegister(LIS2DH_CLICK_SRC, LIS2DH_CLICK_SRC_MASK);
  uint16_t ret = LIS2DH_CLIC_NONE;
  if ( (clic & LIS2DH_CLK_IA_MASK) > 0 ) {
    if ( (clic & LIS2DH_X_CLICK_MASK) > 0 ) {
      if ( (clic & LIS2DH_SCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_SIN_X_NEG;
        } else{
          ret |= LIS2DH_CLIC_SIN_X_POS;
        }
      }
      if ( (clic & LIS2DH_DCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_DBL_X_NEG;
        } else{
          ret |= LIS2DH_CLIC_DBL_X_POS;
        }
      }
    }

    if ( (clic & LIS2DH_Y_CLICK_MASK) > 0 ) {
      if ( (clic & LIS2DH_SCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_SIN_Y_NEG;
        } else{
          ret |= LIS2DH_CLIC_SIN_Y_POS;
        }
      }
      if ( (clic & LIS2DH_DCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_DBL_Y_NEG;
        } else{
          ret |= LIS2DH_CLIC_DBL_Y_POS;
        }
      }
    }

    if ( (clic & LIS2DH_Z_CLICK_MASK) > 0 ) {
      if ( (clic & LIS2DH_SCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_SIN_Z_NEG;
        } else{
          ret |= LIS2DH_CLIC_SIN_Z_POS;
        }
      }
      if ( (clic & LIS2DH_DCLICK_MASK) > 0 ) {
        if ( (clic & LIS2DH_SIGN_MASK ) > 0 ) {
          ret |= LIS2DH_CLIC_DBL_Z_NEG;
        } else{
          ret |= LIS2DH_CLIC_DBL_Z_POS;
        }
      }
    }

  }
  return LIS2DH_CLIC_NONE;
}



/**
 * Set the Click threshold. The default value is zero.
 * The pretty poor documentation do not gives unit for
 * this paramtere.
 * We could imagine it is the same as interrupt threshold :
 * Step is 16mG @ 2G / 32mG @ 4G / 62mG @ 8g / 186mG à 16G
 * but this needs to be confirmed.
 */
drivers_lis2dh12_ret_e lis2dh_setClickThreshold(uint8_t ths) {
  if(ths > LIS2DH_CLK_THS_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  ths = ths << LIS2DH_CLK_THS_SHIFT;
  return __lis2dh_writeMaskedRegisterI(LIS2DH_CLICK_THS, LIS2DH_CLK_THS_MASK, ths);
}

drivers_lis2dh12_ret_e lis2dh_setClickThresholdMg(uint16_t mg, const drivers_lis2dh12_scale_e scale) {
  // Not having information of the threshold unit it is not possible
  // to make this function working correctly.
  // Any information is welcome to correctly implement it.
  // Currently assuming it works like the other functions...
  uint8_t raw = 0;
  if ( scale > LIS2DH_FS_MAXVALUE ) return LIS2DH_FAILED;

  if ( __lis2dh_convertMgToRaw(&raw, mg, scale) ) {
    return lis2dh_setClickThreshold(raw);
  }
  return LIS2DH_FAILED;
}

/**
 * Set the Interrupt mode for click
 * It can be LIS2DH_CLK_INTDUR_UNTILREAD ( interrupt pending unteil CLICK_SRC register read )
 * or LIS2DH_CLK_INTDUR_LATWINDOW ( interrupt cancel afet TIME_LATENCY duration
 */
drivers_lis2dh12_ret_e lis2dh_setClickInterruptMode(uint8_t _mode) {
  if(_mode > LIS2DH_CLK_INTDUR_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  _mode = _mode << LIS2DH_CLK_INTDUR_SHIFT;
  return __lis2dh_writeMaskedRegisterI(LIS2DH_CLICK_THS, LIS2DH_CLK_INTDUR_MASK, _mode);
}

/**
 * Set the Time limit -> due to a lack of documentation in the datasheet and application notes
 * It is hard to say what is this register and what is the unit to use in the register
 * (Question to ST : how can you such bad for documentation ?)
 * I assume time limit is the maximum time to detect a clic or double clic, but starting from ?
 */
drivers_lis2dh12_ret_e lis2dh_setClickTimeLimit(uint8_t raw){
  if(raw > LIS2DH_TLI_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  raw = raw << LIS2DH_TLI_SHIFT;
  return __lis2dh_writeMaskedRegisterI(LIS2DH_TIME_LIMIT, LIS2DH_TLI_MASK, raw);
}

/**
 * Set the Time latency for click -> due to a lack of documentation in the datasheet and application notes
 * It is hard to say what is this register and what is the unit to use in the register
 * (Question to ST : how can you such bad for documentation ?)
 * I do not know what this timer do
 */
drivers_lis2dh12_ret_e lis2dh_setClickTimeLatency(uint8_t raw){
  if(raw > LIS2DH_TIME_LATENCY_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  raw = raw << LIS2DH_TIME_LATENCY_SHIFT;
  return __lis2dh_writeMaskedRegisterI(LIS2DH_TIME_LATENCY, LIS2DH_TIME_LATENCY_MASK, raw);
}


/**
 * Set the Time Window for click -> due to a lack of documentation in the datasheet and application notes
 * It is hard to say what is this register and what is the unit to use in the register
 * (Question to ST : how can you such bad for documentation ?)
 * I do not know what this timer do
 */
drivers_lis2dh12_ret_e lis2dh_setClickTimeWindow(uint8_t raw){
  if(raw > LIS2DH_TIME_WINDOW_MAXVALUE) {
        return LIS2DH_FAILED;
  }
  raw = raw << LIS2DH_TIME_WINDOW_SHIFT;
  return __lis2dh_writeMaskedRegisterI(LIS2DH_TIME_WINDOW, LIS2DH_TIME_WINDOW_MASK, raw);
}


// ======== 4D/6D Device Positionning

/**
 * Honestly even if the documentation is claiming this feature exists in the device,
 * for real, there is no information on How you can get it from the registers.
 * If some developers find a way to use it, let me know.
 */


// -----------------------------------------------------
// Write to LIS2DH
// -----------------------------------------------------

/**
 * Write a 8b register on the chip
 */
drivers_lis2dh12_ret_e __lis2dh_writeRegister(const uint8_t register_addr, const uint8_t value) {

	if ( i2c_write8BRegister(
			&ITSDK_DRIVERS_ACCEL_LIS2DH_I2C,
			__lis2dh_conf._address,		// Non shifted device address
			register_addr,				// Register address (8b or 16b�
			value,						// 8B value to be written
			1							// Register address size 1B or 2B
	     ) == I2C_OK ) return LIS2DH_SUCCESS;
	return LIS2DH_FAILED;

}

/**
 * Write a 16b register
 */
drivers_lis2dh12_ret_e __lis2dh_writeRegisters(const uint8_t msb_register, const uint8_t msb_value, const uint8_t lsb_register, const uint8_t lsb_value) {
    //send write call to sensor address
    //send register address to sensor
    //send value to register
    bool msb_bool, lsb_bool;
    msb_bool = __lis2dh_writeRegister(msb_register, msb_value);
    lsb_bool = __lis2dh_writeRegister(lsb_register, lsb_value);
    return msb_bool | lsb_bool;
}

/**
 * Change one bit of the given register
 * when value is true, all the bit is forced to 1
 * when value is false, all the bit is forced to 0
 */
drivers_lis2dh12_ret_e __lis2dh_writeMaskedRegister8(const uint8_t register_addr, const uint8_t mask, const itsdk_bool_e value) {
    uint8_t data = __lis2dh_readRegister(register_addr);
    uint8_t combo;
    if(value) {
        combo = (mask | data);
    } else {
        combo = ((~mask) & data);
    }
    return __lis2dh_writeRegister(register_addr, combo);
}

/**
 * Change a register content. The mask is applied to the value.
 * The content of the register is read then clear before
 * the value is applied.
 * Value is not shift
 */
drivers_lis2dh12_ret_e __lis2dh_writeMaskedRegisterI(const int register_addr, const int mask, const int value) {
    uint8_t data = __lis2dh_readRegister(register_addr);
    uint8_t masked_value = (( data & ~mask) | (mask & value));
    return __lis2dh_writeRegister(register_addr, masked_value);
}

// -----------------------------------------------------
// Read from LIS2DH
// -----------------------------------------------------

/**
 * Read 8bits of data from the chip, return the value
 */
uint8_t __lis2dh_readRegister(const uint8_t register_addr) {
	uint8_t v;
	if ( i2c_read8BRegister(
			&ITSDK_DRIVERS_ACCEL_LIS2DH_I2C,
			__lis2dh_conf._address,			// Non shifted device address
			register_addr,					// Register address (8b or 16b�
			&v,								// 8B value to be read
			1								// Register address size 1B or 2B
		) == I2C_OK
	) {
      return v;
	} else {
		LIS2DH_LOG_ERROR(("Lis2dh - I2C read error\r\n"));
 	}
    return 0;
}

/**
 * Read 16b data from the Fifo in bust/sequential mode
 * The data are store in a int8/16_t[][3] regarding the RESOLUTION MODE
 * Empty the fifo if the buffer size is larger. maxSz is the max number
 * of X,Y,Z triple
 * Return the number of triple read from Fifo
 */
uint8_t __lis2dh_readFifo(int16_t * _buffer,const uint8_t maxSz) {
	//uint8_t fifoReg = 0x80 | LIS2DH_OUT_X_L; // Force most significant bit to 1 to indicate a multiple read (according to doc)
    int16_t (*buffer16)[3] = (int16_t (*)[3]) _buffer;
    int sz = (__lis2dh_conf._fifoMode == LIS2DH_FM_BYPASS)?1:lis2dh_getFiFoSize();
    if ( sz > maxSz ) sz = maxSz;
    if ( sz > 0 ) {
      int toRead = sz*6;                      // 6 byte to read for each of the X,Y,Z
      int k = 0;
      while ( toRead > 0 ) {
        int transferSize = (toRead >= 30)?30:toRead;
        int8_t __buff[30];
        i2c_memRead(
         		&ITSDK_DRIVERS_ACCEL_LIS2DH_I2C,			// i2c handler
 				__lis2dh_conf._address,						// Device Address => 7 bits non shifted
         		(0x80 | LIS2DH_OUT_X_L),					// Memory address to access
         		8,											// 8 for 8b, 16 for 16 bits ...
         		(uint8_t *)__buff,							// Where to store data to be read
				transferSize								// Size of the data to be read
        );
        for ( int i = 0 ; i < transferSize/6 ; i++ ) {
          if ( __lis2dh_conf._resolution == LIS2DH_RESOLUTION_MODE_8B ) {
            buffer16[k][0] = __buff[(i*6)+1];
            buffer16[k][1] = __buff[(i*6)+3];
            buffer16[k][2] = __buff[(i*6)+5];
          } else {
             int shift = ( __lis2dh_conf._resolution == LIS2DH_RESOLUTION_MODE_10B)?6:4;
             buffer16[k][0] = __buff[(i*6)+0];
             buffer16[k][0] += ((int16_t)__buff[(i*6)+1]) << 8;
             buffer16[k][0] >>= shift;

             buffer16[k][1] = __buff[(i*6)+2];
             buffer16[k][1] += ((int16_t)__buff[(i*6)+3]) << 8;
             buffer16[k][1] >>= shift;

             buffer16[k][2] = __buff[(i*6)+4];
             buffer16[k][2] += ((int16_t)__buff[(i*6)+5]) << 8;
             buffer16[k][2] >>= shift;
          }
          k++;
        }
        toRead -= transferSize;
      }
    }
    return sz;
}


/**
 * Read 16 bits of data by reading two different registers
 */
uint16_t __lis2dh_readRegisters(const uint8_t msb_register, const uint8_t lsb_register) {
    uint8_t msb = __lis2dh_readRegister(msb_register);
    uint8_t lsb = __lis2dh_readRegister(lsb_register);
    return (((int16_t)msb) << 8) | lsb;
}

/**
 * Read and mask a register from the chip
 * The returned value is not shift
 */
uint8_t __lis2dh_readMaskedRegister(const uint8_t register_addr, const uint8_t mask) {
    uint8_t data = __lis2dh_readRegister(register_addr);
    return (data & mask);
}


// ----------------------------------------------------
// Internal common converters
// ----------------------------------------------------

/**
 * From a scale LIS2DH_FS_SCALE_ 2/4/8/16 G value, return
 * the Raw equivalent value to the given mG
 * Step is 16mG @ 2G / 32mG @ 4G / 62mG @ 8g / 186mG à 16G
 * The result is return to _raw
 * Returns false in case of convertion error
 */
drivers_lis2dh12_ret_e __lis2dh_convertMgToRaw(uint8_t * _raw, uint16_t mg, drivers_lis2dh12_scale_e scale) {
  uint16_t raw;
  switch ( scale ) {
    default:
    case LIS2DH_SCALE_FACTOR_2G:
        raw = mg / 16;
        if ( raw == 0 && mg > 0 ) return LIS2DH_FAILED;
        break;
    case LIS2DH_SCALE_FACTOR_4G:
        raw = mg / 32;
        if ( raw == 0 && mg > 0 ) return LIS2DH_FAILED;
        break;
    case LIS2DH_SCALE_FACTOR_8G:
        raw = mg / 62;
        if ( raw == 0 && mg > 0 ) return LIS2DH_FAILED;
        break;
    case LIS2DH_SCALE_FACTOR_16G:
        raw = mg / 186;
        if ( raw == 0 && mg > 0 ) return LIS2DH_FAILED;
        break;
  }
  if ( raw < 256 ) {
    *_raw = (uint8_t)raw;
    return LIS2DH_SUCCESS;
  } else {
    return LIS2DH_FAILED;
  }
}

/**
 * Convert a duration in Ms to a raw duration based on the ODR value given as parameter
 * LIS2DH_ODR_ 1/10/25...1620 HZ
 * step is 1/ODR
 */
drivers_lis2dh12_ret_e __lis2dh_convertMsToRaw(uint8_t * _raw, uint32_t ms, const drivers_lis2dh12_frequency_e odr) {
  uint8_t raw = 0;
  if ( odr >= LIS2DH_ODR_MAXVALUE ) return LIS2DH_FAILED;

  switch ( odr ) {
    case LIS2DH_ODR_1HZ:
        raw = (uint8_t)(ms / 1000);
        if ( raw == 0 && ms > 0 ) return LIS2DH_FAILED;
        break;
    case LIS2DH_ODR_10HZ:
        raw = (uint8_t)(ms / 100);
        if ( raw == 0 && ms > 0 ) return LIS2DH_FAILED;
        break;
    case LIS2DH_ODR_25HZ:
        raw = (uint8_t)(ms / 40);
        if ( raw == 0 && ms > 0 ) return LIS2DH_FAILED;
        break;
    case LIS2DH_ODR_50HZ:
        raw = (uint8_t)(ms / 20);
        if ( raw == 0 && ms > 0 ) return LIS2DH_FAILED;
        break;
    case LIS2DH_ODR_100HZ:
        raw = (uint8_t)(ms / 10);
        if ( raw == 0 && ms > 0 ) return LIS2DH_FAILED;
        break;
    case LIS2DH_ODR_200HZ:
        raw = (uint8_t)(ms / 5);
        if ( raw == 0 && ms > 0 ) return LIS2DH_FAILED;
        break;
    case LIS2DH_ODR_400HZ:
        raw = (uint8_t)((ms * 2) / 5) ;
        if ( raw == 0 && ms > 0 ) return LIS2DH_FAILED;
        break;
    case LIS2DH_ODR_1620HZ:
        raw = (uint8_t)((ms * 1000) / 617);
        if ( raw == 0 && ms > 0 ) return LIS2DH_FAILED;
        break;
    default:
    	break;
  }
  if ( raw < 256 ) {
    *_raw = (uint8_t)raw;
    return LIS2DH_SUCCESS;
  } else {
    return LIS2DH_FAILED;
  }
}

/**
 * Return the acceleration in mg taking into account
 * the given
 * - Resolution ( 8,10,12 bits)
 * - Scale - see LIS2DH_FS_SCALE_XG
 * The result is given in mG
 */
 drivers_lis2dh12_ret_e __lis2dh_getAcceleration(int16_t * x, int16_t * y, int16_t * z){
   return __lis2dh_getAcceleration_i(__lis2dh_conf._resolution,__lis2dh_conf._scale,x,y,z);
 }

drivers_lis2dh12_ret_e __lis2dh_getAcceleration_i(
		const drivers_lis2dh12_resolution_e resolution,
		const drivers_lis2dh12_scale_e scale,
		int16_t * ax, int16_t * ay, int16_t * az
) {

  if ( resolution > LIS2DH_RESOLUTION_MAXVALUE || scale > LIS2DH_FS_MAXVALUE ) return LIS2DH_FAILED;

  int16_t maxValue;
  drivers_lis2dh12_ret_e ret = LIS2DH_SUCCESS;
  switch ( resolution ) {
    default:
    case LIS2DH_RESOLUTION_MODE_8B:
          maxValue = 128;
          break;
    case LIS2DH_RESOLUTION_MODE_10B:
          maxValue = 512;
          break;
    case LIS2DH_RESOLUTION_MODE_12B:
          maxValue = 2048;
          break;
  }

  switch ( scale) {
    case LIS2DH_SCALE_FACTOR_2G:
         *ax = (*ax*2000)/maxValue;
         *ay = (*ay*2000)/maxValue;
         *az = (*az*2000)/maxValue;
         break;
    case LIS2DH_SCALE_FACTOR_4G:
         *ax = (*ax*4000)/maxValue;
         *ay = (*ay*4000)/maxValue;
         *az = (*az*4000)/maxValue;
         break;
    case LIS2DH_SCALE_FACTOR_8G:
         *ax = (*ax*8000)/maxValue;
         *ay = (*ay*8000)/maxValue;
         *az = (*az*8000)/maxValue;
         break;
    case LIS2DH_SCALE_FACTOR_16G:
         *ax = (*ax*16000)/maxValue;
         *ay = (*ay*16000)/maxValue;
         *az = (*az*16000)/maxValue;
         break;
  }

  return ret;
}
#endif
#endif
