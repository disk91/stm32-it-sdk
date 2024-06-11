/* ==========================================================
 * sx126x_hal.c - implementation for sigfox library, board api
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
 *  Created on: 11 june 2024
 *      Author: Paul Pinault
 * ----------------------------------------------------------
 * Copyright : Paul Pinault aka Disk91 2024
 * ==========================================================
 */
#include <it_sdk/config.h>

#if ITSDK_WITH_SIGFOX_LIB == __ENABLE && ITSDK_SIGFOX_LIB == __SIGFOX_SX126X
#include <drivers/sx126x/sigfox_sx126x.h>
#include "sigfox_ep_flags.h"
#include "sx126x_hal.h"
#include "sigfox_types.h"
#include "board/sx126x_hw_api.h"
#include "sigfox_error.h"

#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0
  #include "stm32wlxx_hal_subghz.h"
  HAL_StatusTypeDef SUBGHZSPI_Transmit(SUBGHZ_HandleTypeDef *hsubghz, uint8_t Data);
  HAL_StatusTypeDef SUBGHZSPI_Receive(SUBGHZ_HandleTypeDef *hsubghz, uint8_t *pData);
  HAL_StatusTypeDef SUBGHZ_WaitOnBusy(SUBGHZ_HandleTypeDef *hsubghz);
  HAL_StatusTypeDef SUBGHZ_CheckDeviceReady(SUBGHZ_HandleTypeDef *hsubghz);
#endif

// ----------------------------------------------------------------------
// HELPERS
// ----------------------------------------------------------------------
#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) == 0
void __sx126x_spi_nss_select() {
	#if ITSDK_SFX_SX126X_CS_PIN != __LP_GPIO_NONE
		gpio_set(ITSDK_SFX_SX126X_CS_BANK,ITSDK_SFX_SX126X_CS_PIN);
	#elif ITSDK_SFX_SX126X_CSDN_PIN != __LP_GPIO_NONE
		gpio_reset(ITSDK_SFX_SX126X_CSDN_BANK,ITSDK_SFX_SX126X_CSDN_PIN);
	#else
		#warning "You did not setup a CS pin for SX1262 SPI, are you sure ?"
	#endif
}

void __sx126x_spi_nss_unselect() {
	#if ITSDK_SFX_SX126X_CS_PIN != __LP_GPIO_NONE
		gpio_reset(ITSDK_SFX_SX126X_CS_BANK,ITSDK_SFX_SX126X_CS_PIN);
	#elif ITSDK_SFX_SX126X_CSDN_PIN != __LP_GPIO_NONE
		gpio_set(ITSDK_SFX_SX126X_CSDN_BANK,ITSDK_SFX_SX126X_CSDN_PIN);
	#else
		#warning "You did not setup a CS pin for SX1262 SPI, are you sure ?"
	#endif
}
#endif

// ============================================================
// sx126x_eplib_api.c
// ============================================================

// ----------------------------------------------------------------------
// This function is called to reconfigure the board after a wake-up event
// The hal_wakeup is called right after, don't duplicate actions
// The context parameter is SFX_NULL
// ----------------------------------------------------------------------
sx126x_hal_status_t sx126x_hal_reset( const void* context )
{
    SFX_UNUSED(context);
    LOG_DEBUG_SFXSX126X(("sx126x_hal_reset"));
    return SX126X_HAL_STATUS_OK;
}

// ----------------------------------------------------------------------
// This function is called to reconfigure the board after a wake-up event
// The context parameter is SFX_NULL
// ----------------------------------------------------------------------
sx126x_hal_status_t sx126x_hal_wakeup( const void* context )
{
	SFX_UNUSED(context);
    LOG_DEBUG_SFXSX126X(("sx126x_hal_wakeup"));
    return SX126X_HAL_STATUS_OK;
}

// ----------------------------------------------------------------------
// Transfer the data to the SPI driver
// Context is unused, make a SPI transfer of the command
// Send a byte stream composed by command & data field on the SPI
// ----------------------------------------------------------------------
sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
	SFX_UNUSED(context);
    LOG_DEBUG_SFXSX126X(("sx126x_hal_write %d %d",command_length,data_length));
	#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0

		(void)SUBGHZ_CheckDeviceReady(&ITSDK_SX126X_SPI);
		LL_PWR_SelectSUBGHZSPI_NSS();

		_SPI_Status r = __SPI_OK;
		for ( int i = 0 ; i < command_length && r == __SPI_OK; i++) {
			r =	(_SPI_Status)SUBGHZSPI_Transmit(&ITSDK_SX126X_SPI, command[i]);
		}
		if ( r != __SPI_OK ) goto failed;

		for ( int i = 0 ; i < data_length && r == __SPI_OK; i++) {
			r =	(_SPI_Status)SUBGHZSPI_Transmit(&ITSDK_SX126X_SPI, data[i]);
		}
		if ( r != __SPI_OK ) goto failed;
		LL_PWR_UnselectSUBGHZSPI_NSS();
		SUBGHZ_WaitOnBusy(&ITSDK_SX126X_SPI);

	#else
		#warning "Use of standard SPI driver, never been tested"

		__sx126x_spi_nss_select();
		_SPI_Status r = __SPI_OK;
		for ( int i = 0 ; i < command_length && r == __SPI_OK; i++) {
			r = spi_write_byte(ITSDK_SX126X_SPI, command[i]);
		}
		if ( r != __SPI_OK ) goto failed;

		for ( int i = 0 ; i < data_length && r == __SPI_OK; i++) {
			r =	spi_write_byte(ITSDK_SX126X_SPI, data[i]);
		}
		if ( r != __SPI_OK ) goto failed;
		__sx126x_spi_nss_unselect();

	#endif

    return SX126X_HAL_STATUS_OK;

failed:
	LOG_ERROR_SFXSX126X(("sx126x_hal_write - failed %d",r));
	return SX126X_HAL_STATUS_ERROR;
}

// ----------------------------------------------------------------------
// Read the data from the SPI driver
// Context is unused, make a SPI transfer of the command
// Send a byte stream composed by command & data field on the SPI
// ----------------------------------------------------------------------
sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )

{
	SFX_UNUSED(context);
    LOG_DEBUG_SFXSX126X(("sx126x_hal_read %d %d",command_length,data_length));
	#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0

		(void)SUBGHZ_CheckDeviceReady(&ITSDK_SX126X_SPI);
		LL_PWR_SelectSUBGHZSPI_NSS();

		_SPI_Status r = __SPI_OK;
		for ( int i = 0 ; i < command_length && r == __SPI_OK; i++) {
			r =	(_SPI_Status)SUBGHZSPI_Transmit(&ITSDK_SX126X_SPI, command[i]);
		}
		if ( r != __SPI_OK ) goto failed;

		// Transmit dummy byte
		r = (_SPI_Status)SUBGHZSPI_Transmit(&ITSDK_SX126X_SPI, 0U);
		if ( r != __SPI_OK ) goto failed;

		// Read data
		for ( int i = 0 ; i < data_length && r == __SPI_OK; i++) {
			r =	(_SPI_Status)SUBGHZSPI_Receive(&ITSDK_SX126X_SPI, &data[i]);
		}
		if ( r != __SPI_OK ) goto failed;
		LL_PWR_UnselectSUBGHZSPI_NSS();
		SUBGHZ_WaitOnBusy(&ITSDK_SX126X_SPI);

	#else
		#warning "Use of standard SPI driver, never been tested"

		__sx126x_spi_nss_select();
		_SPI_Status r = __SPI_OK;
		for ( int i = 0 ; i < command_length && r == __SPI_OK; i++) {
			r = spi_write_byte(ITSDK_SX126X_SPI, command[i]);
		}
		if ( r != __SPI_OK ) goto failed;
		// Transmit dummy byte
		r = spi_write_byte(ITSDK_SX126X_SPI, 0U);
		if ( r != __SPI_OK ) goto failed;

		for ( int i = 0 ; i < data_length && r == __SPI_OK; i++) {
			r =	spi_read_byte(ITSDK_SX126X_SPI, &data[i]);
		}
		if ( r != __SPI_OK ) goto failed;
		__sx126x_spi_nss_unselect();

	#endif

    return SX126X_HAL_STATUS_OK;

failed:
	LOG_ERROR_SFXSX126X(("sx126x_hal_read - failed %d",r));
	return SX126X_HAL_STATUS_ERROR;

}


// ============================================================
// sx126x_hal.c
// ============================================================

SX126X_HW_API_status_t SX126X_HW_API_open(SX126X_HW_irq_cb_t callback)
{

	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_open"));

#error todo

#ifdef ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
    SFX_UNUSED(callback);
    // Configure all hardware pin of SX126X chipset.
    // Configure SPI peripherial.
    // Configure interrupt pin to handle radio interrupt.
    // The callback function must be called when such event occurs.
    RETURN();
}

SX126X_HW_API_status_t SX126X_HW_API_close(void)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_close"));

#error todo
#ifdef ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
    //Release all pins and peripherial opened.
    RETURN();
}

// ------------------------------------------------------------------
// Wait for the given number of ms
// ------------------------------------------------------------------
SX126X_HW_API_status_t SX126X_HW_API_delayMs(unsigned short delay_ms)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_delayMs"));
	itsdk_delayMs(delay_ms);

	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif

    RETURN();
}

// ------------------------------------------------------------------
// Return the type of semtech module used between 1261 and 1262
// ------------------------------------------------------------------
SX126X_HW_API_status_t SX126X_HW_API_get_chip_name(SX126X_HW_API_chip_name_t *chipset)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_get_chip_name"));
	#if ITSDK_SFX_SX126X_CHIP == 1261
	 *chipset = SX126X_HW_API_CHIP_NAME_SX1261;
	#elif ITSDK_SFX_SX126X_CHIP == 1262
	 *chipset = SX126X_HW_API_CHIP_NAME_SX1262;
	#else
	 #error "You must select the Semtech core chip version in config file"
	#endif

	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif
    RETURN();
}

// ------------------------------------------------------------------
// Return the type of power regulator used DCDC vs LDO
// ------------------------------------------------------------------
SX126X_HW_API_status_t SX126X_HW_API_get_reg_mode(SX126X_HW_API_reg_mod_t *reg_mode)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_get_reg_mode"));

	#if ITSDK_SFX_SX1262_REGULOR == __POWER_LDO
	 *reg_mode = SX126X_HW_API_REG_MODE_LDO;
	#elif ITSDK_SFX_SX1262_REGULOR == __POWER_DCDC
	 *reg_mode = SX126X_HW_API_REG_MODE_DCDC;
	#else
	 #error "You must select the power regulator type in config file"
	#endif


	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif
    RETURN();
}

SX126X_HW_API_status_t SX126X_HW_API_get_xosc_cfg(SX126X_HW_API_xosc_cfg_t *xosc_cfg)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_get_xosc_cfg"));

#error todo
#ifdef ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
    SFX_UNUSED(xosc_cfg);
    RETURN();
}

SX126X_HW_API_status_t SX126X_HW_API_get_pa_pwr_cfg(SX126X_HW_API_pa_pwr_cfg_t *pa_pwr_cfg, sfx_u32 rf_freq_in_hz, sfx_s8 expected_output_pwr_in_dbm)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_get_pa_pwr_cfg"));

#error todo
#ifdef ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
    SFX_UNUSED(pa_pwr_cfg);
    SFX_UNUSED(rf_freq_in_hz);
    SFX_UNUSED(expected_output_pwr_in_dbm);
    RETURN();
}

SX126X_HW_API_status_t SX126X_HW_API_tx_on(void)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_tx_on"));

#error todo
#ifdef ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
    RETURN();
}

SX126X_HW_API_status_t SX126X_HW_API_tx_off(void)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_tx_off"));

#error todo
#ifdef ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
    RETURN();
}

#ifdef BIDIRECTIONAL
SX126X_HW_API_status_t SX126X_HW_API_rx_on(void)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_rx_on"));

#error todo
#ifdef ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
    RETURN();
}
#endif

#ifdef BIDIRECTIONAL
SX126X_HW_API_status_t SX126X_HW_API_rx_off(void)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_rx_off"));

#error todo
#ifdef ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
    RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
SX126X_HW_API_status_t SX126X_HW_API_get_latency(SX126X_HW_API_latency_t latency_type, sfx_u32 *latency_ms)
{
	LOG_DEBUG_SFXSX126X(("SX126X_HW_API_get_latency"));

#error todo
#ifdef ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
    SFX_UNUSED(latency_type);
    SFX_UNUSED(latency_ms);
    RETURN();
}
#endif



#endif // ITSDK_WITH_SIGFOX_LIB == __ENABLE && ITSDK_SIGFOX_LIB == __SIGFOX_SX126X
