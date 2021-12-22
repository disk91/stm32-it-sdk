/* ==========================================================
 * si72xx.c - Driver for the Si72xx Hall Effect Sensor
 * ----------------------------------------------------------
 *
 *  Created on: 21 dec 2021
 *      Author: Zbynek Kocur
 * ----------------------------------------------------------
 * Copyright (C) 2021 CTU in Prague
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
 * Datasheet - https://www.silabs.com/documents/public/data-sheets/si7210-datasheet.pdf
 * App Note - AN1018: Using the Si72xx Hall-effect Magnetic Position Sensors - https://www.silabs.com/documents/public/application-notes/an1018-si72xx-sensors.pdf
 *
 * ==========================================================
 * Some peaces of that code directly comes from Silicon Laboratories Inc.
 * and identified with << Copyright 2018 Silicon Laboratories Inc. www.silabs.com >>
 *
 * ==========================================================
 */


#include <it_sdk/wrappers.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/itsdk.h>

#include <drivers/hall/si72xx/si72xx.h>

#if ITSDK_DRIVERS_SI72XX == __ENABLE

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

#define SI72XX_SWOP_LATCHMODE           0x7F
#define SI72XX_FIELDPOLSEL_LATCHMODE    2
#define SI72XX_FIELDPOLSEL_SHIFT        6
#define SI72XX_SWHYST_LATCHMODE         0x12

#define SI72XX_OTP_BUSY_MASK            1
#define SI72XX_OTP_READ_EN_MASK         2

#define SI72XX_DSPSIGSEL_MASK           0x07
#define SI72XX_DSPSIGSEL_TEMP           0x01

#define SI72XX_ZERO_FIELD               16384
#define SI72XX_FRESH_BIT_MASK           0x80
#define SI72XX_FRESH_BIT_SHIFT          7
#define SI72XX_DSPSIGM_MASK             0x7F
#define SI72XX_DSPSIGM_SHIFT            8

#define SI72XX_SLEEP_MASK               1
#define SI72XX_STOP_MASK                2
#define SI72XX_ONEBURST_MASK            4
#define SI72XX_USESTORE_MASK            8
#define SI72XX_POWERCTRL_MASK           0x0F
#define SI72XX_MEASRO_MASK              0x80
#define SI72XX_MEASRO_SHIFT             7

#define SI72XX_SW_LOW4FIELD_MASK        0x80
#define SI72XX_SW_OP_MASK               0x7F

#define SI72XX_SWFIELDPOLSEL_MASK       0xC0
#define SI72XX_SWHYST_MASK              0x3F

#define SI72XX_SLTIMEENA_MASK           1
#define SI72XX_SL_FAST_MASK             2
#define SI72XX_SW_TAMPER_MASK           0xFC

/* Burst sizes for mag. measurement */
#define SI72XX_DF_BW_1             0x0U << 1
#define SI72XX_DF_BW_2             0x1U << 1
#define SI72XX_DF_BW_4             0x2U << 1
#define SI72XX_DF_BW_8             0x3U << 1
#define SI72XX_DF_BW_16            0x4U << 1
#define SI72XX_DF_BW_32            0x5U << 1
#define SI72XX_DF_BW_64            0x6U << 1
#define SI72XX_DF_BW_128           0x7U << 1		/* default */
#define SI72XX_DF_BW_256           0x8U << 1
#define SI72XX_DF_BW_512           0x9U << 1
#define SI72XX_DF_BW_1024          0xAU << 1
#define SI72XX_DF_BW_2048          0xBU << 1
#define SI72XX_DF_BW_4096          0xCU << 1
#define SI72XX_DF_BURSTSIZE_1      0x0U << 5		/* default */
#define SI72XX_DF_BURSTSIZE_2      0x1U << 5
#define SI72XX_DF_BURSTSIZE_4      0x2U << 5
#define SI72XX_DF_BURSTSIZE_8      0x3U << 5
#define SI72XX_DF_BURSTSIZE_16     0x4U << 5
#define SI72XX_DF_BURSTSIZE_32     0x5U << 5
#define SI72XX_DF_BURSTSIZE_64     0x6U << 5
#define SI72XX_DF_BURSTSIZE_128    0x7U << 5
#define SI72XX_DFBW_MASK                0x1E
#define SI72XX_DFIIR_MASK               1

#define SI72XX_REV_MASK                 0x0F
#define SI72XX_ID_MASK                  0xF0
#define SI72XX_ID_SHIFT                 4

#define SI72XX_BASE_PART_NUMBER         0x14
#define SI72XX_PART_VARIANT             0x15

#define SI72XX_OFFSET_ADDR              0x1D
#define SI72XX_GAIN_ADDR                0x1E
/** @endcond */

// macros

// convert type _I2C_Status -> drivers_si72xx_ret_e
// (which reflects original I2C errors, but it has some extra values)
#define SI72XX_FROM_I2C_STATUS(i2c_st)                                          \
    i2c_st == __I2C_ERROR   ? __SI72XX_ERROR   :                                \
    i2c_st == __I2C_BUSY    ? __SI72XX_BUSY    :                                \
    i2c_st == __I2C_TIMEOUT ? __SI72XX_TIMEOUT :                                \
                              __SI72XX_UNKNOWN

/***********************************************************************//**
 * @brief
 *  Reads register from the Si72xx sensor.
 *  Command can only be issued if Si72xx is idle mode.
 * @param[in] i2c
 *   The I2C peripheral to use (not used).
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[in] reg_addr
 *   The register address to read from in the sensor.
 * @param[out] read_ptr
 *   The data read from the sensor.
 * @return
 * the behavior can be changed easily so that it, e.g., does not check the status and only
 * OR's the I2C status as follows: i2c_st |= i2c_read8BRegister(...);
 **************************************************************************/
#define SI72XX_READ_8B(reg_addr, read_ptr)                                      \
    i2c_st = i2c_read8BRegister(&ITSDK_DRIVERS_SI72XX_I2C, addr, reg_addr, read_ptr, 1);              \
    if (i2c_st != __I2C_OK)                                                     \
    {                                                                           \
        res = SI72XX_FROM_I2C_STATUS(i2c_st);                                   \
        goto RETURN;                                                            \
    }

/***********************************************************************//**
 * @brief
 *   Writes register in the Si72xx sensor.
 *   Command can only be issued if Si72xx is idle mode.
 * @param[in] i2c
 *   The I2C peripheral to use (not used).
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[in] reg_addr
 *   The register address to write to in the sensor.
 * @param[in] write_val
 *   The data to write to the sensor.
 * @return
 * the behavior can be changed easily so that it, e.g., does not check the status and only
 * OR's the I2C status as follows: i2c_st |= i2c_write8BRegister(...);
 **************************************************************************/
#define SI72XX_WRITE_8B(reg_addr, write_val)                                    \
    i2c_st = i2c_write8BRegister(&ITSDK_DRIVERS_SI72XX_I2C, addr, reg_addr, write_val, 1);            \
    if (i2c_st != __I2C_OK)                                                     \
    {                                                                           \
        res = SI72XX_FROM_I2C_STATUS(i2c_st);                                   \
        goto RETURN;                                                            \
    }

#define SI72XX_INT_CALL(func)                                                   \
    res = func;                                                                 \
    if (res != __SI72XX_OK) goto RETURN;

// "private" functions

static inline int32_t Si72xx_ConvertDataCodesToMagneticField(Si72xxFieldScale_t fieldScale,
                                                             int16_t dataCode);


/***********************************************************************//**
 * @brief
 *   Read out Si72xx Magnetic Field Conversion Data
 *   Command can only be issued if Si72xx is idle mode.
 * @param[in] addr
 *   The I2C address of the sensor
 * @param[out] magData
 *   Mag-field conversion reading, signed 16-bit integer
 **************************************************************************/
drivers_si72xx_ret_e Si72xx_Read_MagField_Data(uint8_t addr,
                                               int16_t *magData)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t read = 0;
    uint8_t freshBit = 0;

    SI72XX_READ_8B(SI72XX_DSPSIGM, &read);

    freshBit = ((read & SI72XX_FRESH_BIT_MASK) >> SI72XX_FRESH_BIT_SHIFT);
    if (freshBit == 0)
    {
        res = __SI72XX_NODATA;
        goto RETURN;
    }

    *magData = ((((uint16_t)read) & SI72XX_DSPSIGM_MASK) << SI72XX_DSPSIGM_SHIFT);

    SI72XX_READ_8B(SI72XX_DSPSIGL, &read);
    /* Data code output is 15-bit unsigned value where 0mT=16384 */
    *magData |= read;
    /* Converts data code output to a signed integer */
    *magData = *magData - SI72XX_ZERO_FIELD;

RETURN:
    return res;
}

/***********************************************************************//**
 * @brief
 *   Puts Si72xx into Sleep mode (lowest power)..
 *   Command can only be issued if Si72xx is idle mode.
 * @param[in] addr
 *   The I2C address of the sensor.
 **************************************************************************/
drivers_si72xx_ret_e Si72xx_FromIdle_GoToSleep(uint8_t addr)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t read = 0;

    SI72XX_READ_8B(SI72XX_CTRL3, &read);
    read = (read & (~SI72XX_SLTIMEENA_MASK));
    SI72XX_WRITE_8B(SI72XX_CTRL3, read);
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = (read | SI72XX_SLEEP_MASK);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

RETURN:
    return res;
}

/***********************************************************************//**
 * @brief
 *   Puts Si72xx into Sleep-Timer-Enable mode.
 *   Si72xx periodically wakes-up, samples the magnetic field, updates the
 *   output, and goes back to sleep-timer-enabled mode.
 *   Command can only be issued if Si72xx is idle mode.
 * @param[in] addr
 *   The I2C address of the sensor.
 **************************************************************************/
drivers_si72xx_ret_e Si72xx_FromIdle_GoToSltimeena(uint8_t addr)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t read = 0;

    SI72XX_READ_8B(SI72XX_CTRL3, &read);
    read = ((read & ~SI72XX_SL_FAST_MASK) | SI72XX_SLTIMEENA_MASK);
    SI72XX_WRITE_8B(SI72XX_CTRL3, read);
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = (read & ~( SI72XX_ONEBURST_MASK |
                      SI72XX_STOP_MASK     |
                      SI72XX_SLEEP_MASK));
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

RETURN:
    return res;
}

/***********************************************************************//**
 * @brief
 *   Wake-up Si72xx and places sensor in idle-mode.
 * @param[in] addr
 *   The I2C address of the sensor.
 * @return
 *   Returns zero on success. Otherwise returns error codes
 *   based on the I2CCSPM
 **************************************************************************/

drivers_si72xx_ret_e Si72xx_WakeUpAndIdle(uint8_t devAdr)
{
    uint8_t value;

    if (i2c_write(&ITSDK_DRIVERS_SI72XX_I2C, devAdr, NULL, 0) != __I2C_OK)
        return __I2C_ERROR;

    return i2c_read(&ITSDK_DRIVERS_SI72XX_I2C, devAdr, &value, 1);

}


/***********************************************************************//**
 * @brief
 *   Read Si72xx OTP Data
 *   Command can only be issued if Si72xx is idle mode.
 * @param[in] addr
 *   The I2C address of the sensor
 * @param[in] otpAddr
 *       The OTB Byte address of the coefficients
 * @param[out] data
 *       OTP data read out
 **************************************************************************/
drivers_si72xx_ret_e Si72xx_Read_OTP(uint8_t addr,
                                     uint8_t otpAddr,
                                     uint8_t *otpData)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t read = 0;

    SI72XX_READ_8B(SI72XX_OTP_CTRL, &read);
    if (read & SI72XX_OTP_BUSY_MASK)
        return __SI72XX_BUSY;

    SI72XX_WRITE_8B(SI72XX_OTP_ADDR, otpAddr);
    SI72XX_WRITE_8B(SI72XX_OTP_CTRL, SI72XX_OTP_READ_EN_MASK);
    SI72XX_READ_8B(SI72XX_OTP_DATA, &read);

    *otpData = read;

RETURN:
    return res;
}

/***********************************************************************//**
 * @brief
 *   Set magnetic-field output range, 20mT or 200mT full-scale
 *   Command can only be issued if Si72xx is idle mode.
 * @param[in] addr
 *   The I2C address of the sensor
 * @param[in] mTScale
 *   20mT or 200mT
 **************************************************************************/
drivers_si72xx_ret_e Si72xx_Set_mT_Range(uint8_t addr,
                                         Si72xxFieldScale_t mTScale)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t srcAddr = 0;
    uint8_t data = 0;

    if (mTScale == SI7210_20MT)
    {
        srcAddr = SI72XX_OTP_20MT_ADDR;
    }
    else if (mTScale == SI7210_200MT)
    {
        srcAddr = SI72XX_OTP_200MT_ADDR;
    }

    SI72XX_INT_CALL(Si72xx_Read_OTP(addr, srcAddr++, &data))
    SI72XX_WRITE_8B(SI72XX_A0, data);
    SI72XX_INT_CALL(Si72xx_Read_OTP(addr, srcAddr++, &data))
    SI72XX_WRITE_8B(SI72XX_A1, data);
    SI72XX_INT_CALL(Si72xx_Read_OTP(addr, srcAddr++, &data))
    SI72XX_WRITE_8B(SI72XX_A2, data);
    SI72XX_INT_CALL(Si72xx_Read_OTP(addr, srcAddr++, &data))
    SI72XX_WRITE_8B(SI72XX_A3, data);
    SI72XX_INT_CALL(Si72xx_Read_OTP(addr, srcAddr++, &data))
    SI72XX_WRITE_8B(SI72XX_A4, data);
    SI72XX_INT_CALL(Si72xx_Read_OTP(addr, srcAddr++, &data))
    SI72XX_WRITE_8B(SI72XX_A5, data);

RETURN:
    return res;
}

/***********************************************************************//**
 * @brief
 *   Wake-up SI72xx, performs a magnetic-field conversion with FIR,
 *   and places Si72xx back to sleep-mode or idle.
 * @param[in] addr
 *   The I2C address of the sensor
 * @param[in] mTScale
 *   mTScale= Si7210_20MT: 20mT full-scale magnetic-field range
 *   mTScale= Si7210_200MT: 200mT full-scale magnetic-field range
 * @param[in] sleepMode
 *   SI72XX_SLEEP: Sleep mode. Lowest power & doesn't update output
 *   SI72XX_SLTIMEENA: Sleep-Timer-Enabled mode. Updates output periodically
 *   SI72XX_IDLE: No sleep
 * @param[out] magFieldData
 *   Magnetic-field conversion reading, signed 16-bit integer
 **************************************************************************/
drivers_si72xx_ret_e Si72xx_ReadMagFieldDataAndSleep(uint8_t addr,
                                                     Si72xxFieldScale_t mTScale,
                                                     Si72xxSleepMode_t sleepMode,
                                                     int16_t *magFieldData)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t read = 0;

    SI72XX_INT_CALL(Si72xx_WakeUpAndIdle(addr))

    /* set the stop-bit */
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = ((read & ~SI72XX_POWERCTRL_MASK) | SI72XX_STOP_MASK);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

    SI72XX_INT_CALL(Si72xx_Set_mT_Range(addr, mTScale))

    /* Set the burst-size for averaging */
    SI72XX_WRITE_8B(SI72XX_CTRL4, SI72XX_DF_BURSTSIZE_1 | SI72XX_DF_BW_128);

    /* Perform a magnetic field conversion */
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = ((read & ~SI72XX_POWERCTRL_MASK) | SI72XX_ONEBURST_MASK);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

    /* Wait for measurement to complete */
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    while ((read & SI72XX_MEASRO_MASK) >> SI72XX_MEASRO_SHIFT)
    {
        SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    }

    SI72XX_INT_CALL(Si72xx_Read_MagField_Data(addr, magFieldData))

    switch (sleepMode)
    {
        case SI72XX_SLEEP_MODE :
            SI72XX_INT_CALL(Si72xx_FromIdle_GoToSleep(addr))
            break;
        case SI72XX_SLTIMEENA_MODE :
            SI72XX_INT_CALL(Si72xx_FromIdle_GoToSltimeena(addr))
            break;
        case SI72XX_IDLE_MODE :
            //TODO
            break;
        default :
            ;
    }

RETURN:
    return res;
}

/***********************************************************************//**
 * @brief
 *   Wake-up Si72xx, and set sleep-mode option.
 *   If Si72xx is in a sleep-mode, it requires a wake-up command first.
 *   Useful for placing Si72xx in SLTIMEENA mode from SLEEP mode,
 *   or vice-versa.
 * @param[in] addr
 *   The I2C address of the sensor
 * @param[in] sleepMode
 *   SI72XX_SLEEP: Puts Si72xx into sleep mode. Lowest power & doesn't update
 *   SI72XX_SLTIMEENA: Si72xx into sltimeena mode. Updates output periodically
 **************************************************************************/
drivers_si72xx_ret_e Si72xx_EnterSleepMode(uint8_t addr,
                                           Si72xxSleepMode_t sleepMode)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;

    SI72XX_INT_CALL(Si72xx_WakeUpAndIdle(addr))

    switch (sleepMode)
    {
        case SI72XX_SLEEP_MODE :
            SI72XX_INT_CALL(Si72xx_FromIdle_GoToSleep(addr))
            break;
        case SI72XX_SLTIMEENA_MODE :
            SI72XX_INT_CALL(Si72xx_FromIdle_GoToSltimeena(addr))
            break;
        case SI72XX_IDLE_MODE :
            //TODO
            break;
        default :
            ;
    }

RETURN:
    return res;
}

/***********************************************************************//**
 * @brief
 *   Wake-up Si72xx, and configures output for Latch mode.
 *   Switch point = 0mT w/ 0.2mT hysteresis
 * @param[in] addr
 *   The I2C address of the sensor
 **************************************************************************/
drivers_si72xx_ret_e Si72xx_EnterLatchMode(uint8_t addr)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t read = 0;

    SI72XX_INT_CALL(Si72xx_WakeUpAndIdle(addr))

    /* Set Stop-bit */
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = (read | SI72XX_USESTORE_MASK | SI72XX_STOP_MASK);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

    /* Set output high for low magnetic field */
    /* Set sw_op to zero for latch mode */
    read = (SI72XX_SWOP_LATCHMODE | SI72XX_SW_LOW4FIELD_MASK);
    SI72XX_WRITE_8B(SI72XX_CTRL1, read);

    /* Set output to unipolar positive with hysteresis = 0.2mT */
    read = ((SI72XX_FIELDPOLSEL_LATCHMODE << SI72XX_FIELDPOLSEL_SHIFT)
            | SI72XX_SWHYST_LATCHMODE);
    SI72XX_WRITE_8B(SI72XX_CTRL2, read);

    /* Enable the sleep-timer for periodic measurements */
    SI72XX_READ_8B(SI72XX_CTRL3, &read);
    read = (read | SI72XX_SLTIMEENA_MASK);
    SI72XX_WRITE_8B(SI72XX_CTRL3, read);

    /* Clear stop-bit */
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = (read & ~SI72XX_STOP_MASK);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

RETURN:
    return res;
}

/***********************************************************************//**
 * @brief
 *   Wakes up SI72xx, performs temperature conversion and places Si72xx
 *   into SI72XX_SLEEP sleep-mode.
 * @param[in] addr
 *   The I2C address of the sensor
 * @param[out] temp
 *   Temperature measurement in millidegree Celsius
 **************************************************************************/
drivers_si72xx_ret_e Si72xx_ReadTemperatureAndSleep(uint8_t addr,
                                                    int32_t *rawTemp)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t read = 0;
    uint8_t dspSigM, dspSigL;
    uint8_t freshBit;
    int16_t dataValue;
    int32_t milliCelsius;

    uint16_t tempCtrl4Setting = 0x00;

    SI72XX_INT_CALL(Si72xx_WakeUpAndIdle(addr))

    /* Set stop-bit */
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = ((read & ~SI72XX_POWERCTRL_MASK) | SI72XX_STOP_MASK);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

    /* clear IIR & FIR filtering */
    SI72XX_WRITE_8B(SI72XX_CTRL4, tempCtrl4Setting);

    /* Select temperature conversion */
    SI72XX_READ_8B(SI72XX_DSPSIGSEL, &read);
    read = ((read & ~SI72XX_DSPSIGSEL_MASK) | SI72XX_DSPSIGSEL_TEMP);
    SI72XX_WRITE_8B(SI72XX_DSPSIGSEL, read);

    /* Perform temperature conversion */
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = (((read & (~SI72XX_STOP_MASK)) & ~(SI72XX_SLEEP_MASK))
            | SI72XX_ONEBURST_MASK);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

    /* Read conversion res */
    SI72XX_READ_8B(SI72XX_DSPSIGM, &dspSigM);
    SI72XX_READ_8B(SI72XX_DSPSIGL, &dspSigL);

    SI72XX_INT_CALL(Si72xx_FromIdle_GoToSleep(addr))

    freshBit = dspSigM >> SI72XX_FRESH_BIT_SHIFT;
    if (freshBit == 0)
    {
        res = __SI72XX_NODATA;
        goto RETURN;
    }

    /* dataValue = (Dspigm[6:0]<<5) + (Dspigl[7:0]>>3) */
    dataValue = (((uint16_t)dspSigM) & SI72XX_DSPSIGM_MASK) << 8;
    dataValue = (dataValue | dspSigL) >> 3;

    /* rawTemp(mC) = ((dataValue^2)*(-3.83*10^-6))+(0.16094*dataValue)-279.8 */
    milliCelsius = ((int32_t)dataValue * (int32_t)dataValue * -383 / 100000)
                   + ((16094 * dataValue) / 100) - 279800;

    *rawTemp = milliCelsius;

RETURN:
    return res;
}

/***********************************************************************//**
 * @brief
 *   Wakes up SI72xx, performs temperature conversion and places Si72xx
 *   into SI72XX_SLEEP sleep-mode.
 * @param[in] addr
 *   The I2C address of the sensor
 * @param[out] offsetValue
 *   Temperature offset correction
 * @param[out] gainValue
 *   Temperature gain correction
 **************************************************************************/
drivers_si72xx_ret_e Si72xx_ReadTempCorrectionDataAndSleep(uint8_t addr,
                                                           int16_t *offsetValue,
                                                           int16_t *gainValue)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t read = 0;

    SI72XX_INT_CALL(Si72xx_WakeUpAndIdle(addr))

    /* Set Stop-bit */
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = ((read & ~SI72XX_POWERCTRL_MASK) | SI72XX_STOP_MASK);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

    /* Read offset register value */
    SI72XX_INT_CALL(Si72xx_Read_OTP(addr, SI72XX_OFFSET_ADDR, &read))
    /* Calculate offset: Offset = value(0x1D)/16 */
    *offsetValue = (int8_t)read * 1000 / 16;

    /* Read gain register value */
    SI72XX_INT_CALL(Si72xx_Read_OTP(addr, SI72XX_GAIN_ADDR, &read))
    /* calculate gain: Gain = (value(0x1E)/2048) + 1 */
    *gainValue = ((int8_t)read * 1000 / 2048) + 1000;

    SI72XX_INT_CALL(Si72xx_FromIdle_GoToSleep(addr))

RETURN:
    return res;
}

/**************************************************************************//**
 * @brief
 *   Wakes up SI72xx, performs a temperature conversion, and places sensor
 *   back to sleep. Temperature calculation is performed using compensation
 *   data.
 * @param[out] temp
 *   Temperature measurement in millidegree Celsius
 * @param[in] offsetData
 *   Offset correction data
 * @param[in] gainData
 *   Gain correction data
 *****************************************************************************/
drivers_si72xx_ret_e Si72xx_ReadCorrectedTempAndSleep(uint8_t addr,
                                                      int16_t offsetData,
                                                      int16_t gainData,
                                                      int32_t *correctedTemp)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t read = 0;
    uint8_t dspSigM, dspSigL;
    uint8_t freshBit;
    int16_t dataValue;
    int32_t rawTemp;
    int32_t milliCelsius;

    uint16_t tempCtrl4Setting = 0x00;

    SI72XX_INT_CALL(Si72xx_WakeUpAndIdle(addr))

    /* Set stop-bit */
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = ((read & ~SI72XX_POWERCTRL_MASK) | SI72XX_STOP_MASK);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

    /* Clear IIR and FIR filtering */
    SI72XX_WRITE_8B(SI72XX_CTRL4, tempCtrl4Setting);

    /* Select Temperature conversion */
    SI72XX_READ_8B(SI72XX_DSPSIGSEL, &read);
    read = ((read & ~SI72XX_DSPSIGSEL_MASK) | SI72XX_DSPSIGSEL_TEMP);
    SI72XX_WRITE_8B(SI72XX_DSPSIGSEL, read);

    /* Perform temperature conversion */
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    read = (((read & (~SI72XX_STOP_MASK)) & ~(SI72XX_SLEEP_MASK))
            | SI72XX_ONEBURST_MASK);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, read);

    /* Read temperature conversion result */
    SI72XX_READ_8B(SI72XX_DSPSIGM, &dspSigM);
    SI72XX_READ_8B(SI72XX_DSPSIGL, &dspSigL);

    SI72XX_INT_CALL(Si72xx_FromIdle_GoToSleep(addr))

    freshBit = dspSigM >> SI72XX_FRESH_BIT_SHIFT;
    if (freshBit == 0)
    {
        res = __SI72XX_NODATA;
        goto RETURN;
    }

    /* dataValue = (Dspigm[6:0]<<5) + (Dspigl[7:0]>>3) */
    dataValue = (((uint16_t)dspSigM) & SI72XX_DSPSIGM_MASK) << 8;
    dataValue = (dataValue | dspSigL) >> 3;

    /* rawTemp equation is from Si7210 datasheet */
    /* rawTemp(mC) = ((dataValue^2)*(-3.83*10^-6))+(0.16094*dataValue)-279.8 */
    rawTemp = ((int32_t)dataValue * (int32_t)dataValue * -383 / 100000)
                + ((16094 * dataValue) / 100) - 279800;

    milliCelsius = ((rawTemp * (int32_t)gainData) + offsetData) / 1000;

    *correctedTemp = milliCelsius;

RETURN:
    return res;
}

/**************************************************************************
* @brief
*   Wake-up Si72xx, read out part Revision and ID, and place Si72xx
*   back to SLEEP sleep-mode.
* @param[in] addr
*   The I2C address of the sensor
* @param[out] partId
*        Si7210 part ID
* @param[out] partRev
*        Si72xx part Revision
**************************************************************************/
drivers_si72xx_ret_e Si72xx_IdentifyAndSleep(uint8_t addr,
                                             uint8_t *partId,
                                             uint8_t *partRev)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    uint8_t read = 0;

    SI72XX_INT_CALL(Si72xx_WakeUpAndIdle(addr))
    SI72XX_READ_8B(SI72XX_POWER_CTRL, &read);
    SI72XX_WRITE_8B(SI72XX_POWER_CTRL, (read | SI72XX_STOP_MASK));
    SI72XX_READ_8B(SI72XX_HREVID, &read);
    SI72XX_INT_CALL(Si72xx_FromIdle_GoToSleep(addr))

    *partRev = read &  SI72XX_REV_MASK;
    *partId  = read >> SI72XX_ID_SHIFT;

RETURN:
    return res;
}

/**************************************************************************
* @brief
*   Wake-up Si72xx, read out Si72xx base part-number and variant, and
*   place sensor back to SLEEP sleep-mode.
* @param[in] addr
*   The I2C address of the sensor
* @param[out] basePn
*        Si7210 part ID
* @param[out] partRev
*        Si72xx part Revision
**************************************************************************/
drivers_si72xx_ret_e Si72xx_ReadVariantAndSleep(uint8_t addr,
                                                uint8_t *basePn,
                                                uint8_t *pnVariant)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;

    uint8_t read = 0;

    SI72XX_INT_CALL(Si72xx_WakeUpAndIdle(addr))
    SI72XX_INT_CALL(Si72xx_Read_OTP(addr, SI72XX_BASE_PART_NUMBER, &read))
    *basePn = read;
    SI72XX_INT_CALL(Si72xx_Read_OTP(addr, SI72XX_PART_VARIANT, &read))
    *pnVariant = read;
    SI72XX_INT_CALL(Si72xx_FromIdle_GoToSleep(addr))

RETURN:
    return res;
}

/**************************************************************************
* @brief
*   Self-test sequence offered by the device. It uses an internal
*   coil to generate and test the + and - field.
* @param[in] addr
*   The I2C address of the sensor
**************************************************************************/
drivers_si72xx_ret_e Si72xx_SelfTest(uint8_t addr)
{
    // required for SI72XX_READ/WRITE_8B / SI72XX_INT_CALL macro call
    drivers_si72xx_ret_e res = __SI72XX_OK;
    _I2C_Status i2c_st = __I2C_OK;

    int16_t field_pos = 0;
    int16_t field_neg = 0;

    /* Enable test field generator coil in POSITIVE direction. */
    SI72XX_INT_CALL(Si72xx_WakeUpAndIdle(addr))
    SI72XX_WRITE_8B(SI72XX_TM_FG, 1);

    /* Measure field strength */
    SI72XX_INT_CALL(Si72xx_ReadMagFieldDataAndSleep(addr, SI7210_200MT, SI72XX_IDLE_MODE, &field_pos)) // 200mT
    field_pos = Si72xx_ConvertDataCodesToMagneticField(SI7210_200MT, field_pos)/1000;

    /* Enable test field generator coil in POSITIVE direction. */
    SI72XX_WRITE_8B(SI72XX_TM_FG, 2);

    /* Measure field strength */
    SI72XX_INT_CALL(Si72xx_ReadMagFieldDataAndSleep(addr, SI7210_200MT, SI72XX_IDLE_MODE, &field_neg))
    field_neg = Si72xx_ConvertDataCodesToMagneticField(SI7210_200MT, field_neg)/1000;

    /* Disable test field generator coil. */
    SI72XX_WRITE_8B(SI72XX_TM_FG, 0);

    /* Send to sleep mode */
    SI72XX_INT_CALL(Si72xx_EnterSleepMode(addr, SI72XX_SLEEP_MODE))

    /* Vdd of SI7210. This is used in device's self-test calculations */
    #define SI72xx_VDD          (3.3f)

    float b_out = 1.16 * SI72xx_VDD;
    float b_upper = b_out + (b_out * 0.25); /* +25% */
    float b_lower = b_out - (b_out * 0.25); /* -25% */

    if( (field_pos <= b_upper) &&
        (field_pos >= b_lower) &&
        (field_neg >= (b_upper * -1)) &&
        (field_neg <= (b_lower * -1)))
    {
        log_info("(i) Sensor 0x%X self test PASS\r\n", addr);
    }
    else
    {
        log_info("(i) Sensor 0x%X self test FAIL!\r\n", addr);
        res = __SI72XX_MISCALIB;
        goto RETURN;
    }

RETURN:
    return res;
}

/**************************************************************************//**
 * @brief  Convert Si7210 I2C Data Readings to Magnetic Field in microTeslas
 * @param[in] fieldScale
 *   20mT or 200mT full-scale magnetic field range
 * @param[in] dataCodes
 *   signed 15bit value read from hall sensor after magnetic field conversion
 * @return microTeslas
 *****************************************************************************/
static inline int32_t Si72xx_ConvertDataCodesToMagneticField(Si72xxFieldScale_t fieldScale,
                                                             int16_t dataCode)
{
    switch (fieldScale)
    {
        case SI7210_20MT :
            /* 20mT: 1(LSB) = 1.25uT */
            return (dataCode * 125) / 100;
        case SI7210_200MT :
            /* 200mT: 1(LSB) = 12.5uT */
            return (dataCode * 125) / 10;
        default :
            return 0;
    }
}
#endif
