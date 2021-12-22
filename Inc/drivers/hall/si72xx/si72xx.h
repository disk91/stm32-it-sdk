/* ==========================================================
 * si72xx.h - Driver for the Si72xx Hall Effect Sensor
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

#ifndef DRIVERS_HALL_SI72XX_SI72XX_H
#define DRIVERS_HALL_SI72XX_SI72XX_H

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** I2C device address for Si72xx */
#define SI72XX_ADDR_0      0x30
#define SI72XX_ADDR_1      0x31
#define SI72XX_ADDR_2      0x32
#define SI72XX_ADDR_3      0x33

/** I2C registers for Si72xx */
#define SI72XX_HREVID             0xC0
#define SI72XX_DSPSIGM            0xC1
#define SI72XX_DSPSIGL            0xC2
#define SI72XX_DSPSIGSEL          0xC3
#define SI72XX_POWER_CTRL         0xC4
#define SI72XX_ARAUTOINC          0xC5
#define SI72XX_CTRL1              0xC6
#define SI72XX_CTRL2              0xC7
#define SI72XX_SLTIME             0xC8
#define SI72XX_CTRL3              0xC9
#define SI72XX_A0                 0xCA
#define SI72XX_A1                 0xCB
#define SI72XX_A2                 0xCC
#define SI72XX_CTRL4              0xCD
#define SI72XX_A3                 0xCE
#define SI72XX_A4                 0xCF
#define SI72XX_A5                 0xD0
#define SI72XX_OTP_ADDR           0xE1
#define SI72XX_OTP_DATA           0xE2
#define SI72XX_OTP_CTRL           0xE3
#define SI72XX_TM_FG              0xE4

#define SI72XX_OTP_20MT_ADDR      0x21
#define SI72XX_OTP_200MT_ADDR     0x27

/*******************************************************************************
 ********************************   ENUMS   ************************************
 ******************************************************************************/
/** Si72xx magnetic field full-scales */
typedef enum {
  SI7210_20MT,
  SI7210_200MT
} Si72xxFieldScale_t;

/** Si72xx sleep modes */
typedef enum {
  SI72XX_SLEEP_MODE,
  SI72XX_SLTIMEENA_MODE,
  SI72XX_IDLE_MODE
} Si72xxSleepMode_t;


/*!
 * @brief Si72xx API status result code.
 */
typedef enum drivers_si72xx_ret_e
{
    __SI72XX_OK       =  0,
    __SI72XX_ERROR    =  1,
    __SI72XX_BUSY     =  2,
    __SI72XX_TIMEOUT  =  3,   /* until here, I2C errors (cf. _I2C_Status) */
    __SI72XX_NODATA   =  4,
    __SI72XX_MISCALIB =  5,
    __SI72XX_UNKNOWN  =  6,
} drivers_si72xx_ret_e;

/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

drivers_si72xx_ret_e Si72xx_WakeUpAndIdle(uint8_t addr);
drivers_si72xx_ret_e Si72xx_Read_MagField_Data(uint8_t addr,
                                   int16_t *magData);
drivers_si72xx_ret_e Si72xx_FromIdle_GoToSleep(uint8_t addr);
drivers_si72xx_ret_e Si72xx_FromIdle_GoToSltimeena(uint8_t addr);

drivers_si72xx_ret_e Si72xx_ReadMagFieldDataAndSleep(uint8_t addr,
                                         Si72xxFieldScale_t mTScale,
                                         Si72xxSleepMode_t sleepMode,
                                         int16_t *magFieldData);
drivers_si72xx_ret_e Si72xx_EnterSleepMode(uint8_t addr,
                               Si72xxSleepMode_t sleepMode);
drivers_si72xx_ret_e Si72xx_EnterLatchMode (uint8_t addr);
drivers_si72xx_ret_e Si72xx_ReadTemperatureAndSleep(uint8_t addr,
                                        int32_t *rawTemp);
drivers_si72xx_ret_e Si72xx_ReadCorrectedTempAndSleep(uint8_t addr,
                                          int16_t offsetData,
                                          int16_t gainData,
                                          int32_t *correctedTemp);
drivers_si72xx_ret_e Si72xx_ReadTempCorrectionDataAndSleep(uint8_t addr,
                                               int16_t *offsetValue,
                                               int16_t *gainValue);

drivers_si72xx_ret_e Si72xx_IdentifyAndSleep(uint8_t addr,
                                 uint8_t *partId,
                                 uint8_t *partRev);
drivers_si72xx_ret_e Si72xx_ReadVariantAndSleep(uint8_t addr,
                                    uint8_t *basePn,
                                    uint8_t *pnVariant);
drivers_si72xx_ret_e Si72xx_SelfTest(uint8_t addr);


#endif /* DRIVERS_HALL_SI72XX_SI72XX_H */
