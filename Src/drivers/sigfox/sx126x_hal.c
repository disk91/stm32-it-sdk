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
#ifdef USE_SIGFOX_EP_FLAGS_H
  #include "sigfox_ep_flags.h" // 3rdParties/sigfox/sigfox-ep-lib/inc/sigfox_ep_flags.h
#endif
#include "sx126x_hal.h"
#include "sigfox_types.h"
#include "board/sx126x_hw_api.h"
#include "sigfox_error.h"
#include "sx126x.h"

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
    LOG_DEBUG_SFXSX126X(("[SX] sx126x_hal_reset\r\n"));
    return SX126X_HAL_STATUS_OK;
}

// ----------------------------------------------------------------------
// This function is called to reconfigure the board after a wake-up event
// The context parameter is SFX_NULL
// ----------------------------------------------------------------------
sx126x_hal_status_t sx126x_hal_wakeup( const void* context )
{
	SFX_UNUSED(context);
    LOG_DEBUG_SFXSX126X(("[SX] sx126x_hal_wakeup\r\n"));
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
    LOG_DEBUG_SFXSX126X(("[SX] sx126x_hal_write %d %d\r\n",command_length,data_length));

    #if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWSIGFOX) > 0
    	LOG_DEBUG_SFXSX126X(("[SX]  cmd: "));
		for ( int i = 0 ; i < command_length ; i++ ) {
			LOG_DEBUG_SFXSX126X(("%02X ",command[i]));
		}
		LOG_DEBUG_SFXSX126X(("\r\n"));
    	LOG_DEBUG_SFXSX126X(("[SX]  data: "));
		for ( int i = 0 ; i < data_length ; i++ ) {
			LOG_DEBUG_SFXSX126X(("%02X ",data[i]));
		}
		LOG_DEBUG_SFXSX126X(("\r\n"));
	#endif

	#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0

		(void)SUBGHZ_CheckDeviceReady(&ITSDK_SFX_SX126X_SPI);
		LL_PWR_SelectSUBGHZSPI_NSS();

		_SPI_Status r = __SPI_OK;
		for ( int i = 0 ; i < command_length && r == __SPI_OK; i++) {
			r =	(_SPI_Status)SUBGHZSPI_Transmit(&ITSDK_SFX_SX126X_SPI, command[i]);
		}
		if ( r != __SPI_OK ) goto failed;

		for ( int i = 0 ; i < data_length && r == __SPI_OK; i++) {
			r =	(_SPI_Status)SUBGHZSPI_Transmit(&ITSDK_SFX_SX126X_SPI, data[i]);
		}
		if ( r != __SPI_OK ) goto failed;
		LL_PWR_UnselectSUBGHZSPI_NSS();
		SUBGHZ_WaitOnBusy(&ITSDK_SFX_SX126X_SPI);

	#else
		#warning "Use of standard SPI driver, never been tested"

		__sx126x_spi_nss_select();
		_SPI_Status r = __SPI_OK;
		for ( int i = 0 ; i < command_length && r == __SPI_OK; i++) {
			r = spi_write_byte(ITSDK_SFX_SX126X_SPI, command[i]);
		}
		if ( r != __SPI_OK ) goto failed;

		for ( int i = 0 ; i < data_length && r == __SPI_OK; i++) {
			r =	spi_write_byte(ITSDK_SFX_SX126X_SPI, data[i]);
		}
		if ( r != __SPI_OK ) goto failed;
		__sx126x_spi_nss_unselect();

	#endif

    return SX126X_HAL_STATUS_OK;

failed:
	LOG_ERROR_SFXSX126X(("[SX] sx126x_hal_write - failed %d\r\n",r));
	return SX126X_HAL_STATUS_ERROR;
}

// ----------------------------------------------------------------------
// Read the data from the SPI driver
// Context is unused, make a SPI transfer of the command
// Send a byte stream composed by command & data field on the SPI
// The command sent by the upper layer contains the NOP instruction when required
// ----------------------------------------------------------------------
sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )

{
	SFX_UNUSED(context);
    LOG_DEBUG_SFXSX126X(("[SX] sx126x_hal_read %d %d\r\n",command_length,data_length));
    LOG_DEBUG_SFXSX126X(("[SX] cmd[0] 0x%02X adr 0x%02X%02X\r\n",command[0],command[1],command[2]));

	#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0

		(void)SUBGHZ_CheckDeviceReady(&ITSDK_SFX_SX126X_SPI);
		LL_PWR_SelectSUBGHZSPI_NSS();

		_SPI_Status r = __SPI_OK;
		for ( int i = 0 ; i < command_length && r == __SPI_OK; i++) {
			r =	(_SPI_Status)SUBGHZSPI_Transmit(&ITSDK_SFX_SX126X_SPI, command[i]);
		}
		if ( r != __SPI_OK ) goto failed;

		// Read data
		LOG_DEBUG_SFXSX126X(("[SX] resp : "));
		for ( int i = 0 ; i < data_length && r == __SPI_OK; i++) {
			r =	(_SPI_Status)SUBGHZSPI_Receive(&ITSDK_SFX_SX126X_SPI, &data[i]);
			LOG_DEBUG_SFXSX126X(("%02X ",data[i]));
		}
		LOG_DEBUG_SFXSX126X(("\r\n"));
		if ( r != __SPI_OK ) goto failed;

		LL_PWR_UnselectSUBGHZSPI_NSS();
		SUBGHZ_WaitOnBusy(&ITSDK_SFX_SX126X_SPI);

	#else
		#warning "Use of standard SPI driver, never been tested"

		__sx126x_spi_nss_select();
		_SPI_Status r = __SPI_OK;
		for ( int i = 0 ; i < command_length && r == __SPI_OK; i++) {
			r = spi_write_byte(ITSDK_SFX_SX126X_SPI, command[i]);
		}
		if ( r != __SPI_OK ) goto failed;
		// Transmit dummy byte
		r = spi_write_byte(ITSDK_SFX_SX126X_SPI, 0U);
		if ( r != __SPI_OK ) goto failed;

		for ( int i = 0 ; i < data_length && r == __SPI_OK; i++) {
			r =	spi_read_byte(ITSDK_SFX_SX126X_SPI, &data[i]);
		}
		if ( r != __SPI_OK ) goto failed;
		__sx126x_spi_nss_unselect();

	#endif

    return SX126X_HAL_STATUS_OK;

failed:
	#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0
		LL_PWR_UnselectSUBGHZSPI_NSS();
	#else
		__sx126x_spi_nss_unselect();
	#endif
	LOG_ERROR_SFXSX126X(("[SX] sx126x_hal_read - failed %d\r\n",r));
	return SX126X_HAL_STATUS_ERROR;

}


// ============================================================
// sx126x_hal.c
// ============================================================

// ============================================================
// Override the IRQ handlers
// As Subghz driver already decode and split the different
// interruption we need to regroup them for the sigfox handler
// this avoid to modify the global irq handler and makes integration
// easier
// ============================================================

#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0

// the SX126X_RF_API_process callback function is bypassed as the subghz handler already
// split / clear and route the irq to the dedicated handler.
// the other way is to remove the hal sughz handler but it will be override by the cube mx
// regeneration and more complex to explain. Make sure this function is updated if SX126X_RF_API_process callback
// is updated in the future


// Approach #1 - Not working
// The function passed to SX126X_RF_API_open is SX126X_irq
// static void SX126X_irq(void) {
//    if (sx126x_ctx.irq_en == 1) {
//    	sx126x_ctx.irq_flag = 1;
//#ifdef ASYNCHRONOUS
//        if (sx126x_ctx.callbacks.process_cb != SFX_NULL)
//        	sx126x_ctx.callbacks.process_cb();
//#endif
//    }
//}
// In async mode the following code from SX126X_RF_API_send is not
// processed and need to be implemented elsewhere
// if(sx126x_ctx.error_flag == 1) {
//   EXIT_ERROR((RF_API_status_t) RF_API_ERROR);
// }
// But in fact in this step there is no errors generated, as the error coming
// from HAL_SUBGHZ_TxCpltCallback does not exist with this implementation.
//
// Rx requires a change in the sx126x_rf_api - pull request pending for SX126X_RF_API_receive
// while (1) becomes while(sx126x_ctx.error_flag != 1)
// In sync mode, the data are not reported as the following code will
// never be reached in SX126X_RF_API_receive
// if(sx126x_ctx.rx_done_flag == 1) {
//   rx_data->data_received = SFX_TRUE;
//   break;
// }
// The upper layer function will have to verify the Rx complete status calling the added function above
// Problem, the sigfox context is static :(
//
// Approach #2 - should work
// Override the sx126x_get_and_clear_irq_status to fake the sx registers and return the right status
// function of the previous IRQ handler seen, so internal process function may work normally

#define __SX126X_IRQ_CLEARED    0
#define __SX126X_IRQ_TXCOMPLETE	1
#define __SX126X_IRQ_RXCOMPLETE 2
static uint8_t __sx126x_irq_status = __SX126X_IRQ_CLEARED;
static SX126X_HW_irq_cb_t * __sx1262_irq_cb = NULL;
sx126x_status_t SX126X_RF_API_get_and_clear_irq_status( const void* context, sx126x_irq_mask_t* irq ) {

	// override the default function
	if ( __sx126x_irq_status & __SX126X_IRQ_TXCOMPLETE ) *irq |= SX126X_IRQ_TX_DONE;
	if ( __sx126x_irq_status & __SX126X_IRQ_RXCOMPLETE ) *irq |= SX126X_IRQ_RX_DONE;
	__sx126x_irq_status = __SX126X_IRQ_CLEARED;

	#if (ITSDK_LOGGER_MODULE & __LOG_MOD_LOWSIGFOX) > 0
		if ( *irq != 0 ) LOG_DEBUG_SFXSX126X(("[SX] SX126X_RF_API_get_and_clear_irq_status\r\n"));
	#endif

	return SX126X_STATUS_OK;
}


void HAL_SUBGHZ_TxCpltCallback(SUBGHZ_HandleTypeDef *hsubghz) {
	LOG_DEBUG_SFXSX126X(("[SX] HAL_SUBGHZ_TxCpltCallback\r\n"));
	__sx126x_irq_status |= __SX126X_IRQ_TXCOMPLETE;
	if ( __sx1262_irq_cb != NULL ) __sx1262_irq_cb();

	/* -- approach #1 not compiling
	if (sx126x_ctx.irq_en == 1) {
		sx126x_ctx.irq_flag = 0;
		SX126X_HW_API_tx_off();
		sx126x_ctx.tx_done_flag = 1;
	    #ifdef ASYNCHRONOUS
  	     if (sx126x_ctx.callbacks.tx_cplt_cb != SFX_NULL)
			sx126x_ctx.callbacks.tx_cplt_cb();
		#endif
	}
	*/

}

#ifdef BIDIRECTIONAL

 static volatile itsdk_bool_e __sx126x_dataReceived = BOOL_FALSE;
 void HAL_SUBGHZ_RxCpltCallback(SUBGHZ_HandleTypeDef *hsubghz) {
	LOG_DEBUG_SFXSX126X(("[SX] HAL_SUBGHZ_RxCpltCallback\r\n"));
	__sx126x_irq_status |= __SX126X_IRQ_RXCOMPLETE;
	if ( __sx1262_irq_cb != NULL ) __sx1262_irq_cb();

	/* -- approach #1 not compiling
	if (sx126x_ctx.irq_en == 1) {
		sx126x_ctx.irq_flag = 0;

		SX126X_HW_API_rx_off();
		sx126x_ctx.rx_done_flag = 1;
		#ifdef ASYNCHRONOUS
		 if (sx126x_ctx.callbacks.rx_data_received_cb != SFX_NULL)
			sx126x_ctx.callbacks.rx_data_received_cb();
		#else
			__sx126x_dataReceived = BOOL_TRUE;
		#endif
	}
	*/
 }

 #ifndef ASYNCHRONOUS
  itsdk_bool_e sx126x_hasDataReceived() {
	return __sx126x_dataReceived;
  }

  void sx126x_resetDataReceived() {
	__sx126x_dataReceived = BOOL_FALSE;
  }
 #endif // ASYNCHRONOUS

#endif	// BIDIRECTIONAL

// Unused irq handlers
// void HAL_SUBGHZ_PreambleDetectedCallback(SUBGHZ_HandleTypeDef *hsubghz){}
// void HAL_SUBGHZ_SyncWordValidCallback(SUBGHZ_HandleTypeDef *hsubghz) {}
// void HAL_SUBGHZ_HeaderValidCallback(SUBGHZ_HandleTypeDef *hsubghz) {}
// void HAL_SUBGHZ_HeaderErrorCallback(SUBGHZ_HandleTypeDef *hsubghz) {}
// void HAL_SUBGHZ_CRCErrorCallback(SUBGHZ_HandleTypeDef *hsubghz){}
// void HAL_SUBGHZ_CADStatusCallback(SUBGHZ_HandleTypeDef *hsubghz,HAL_SUBGHZ_CadStatusTypeDef cadstatus){}
// void HAL_SUBGHZ_RxTxTimeoutCallback(SUBGHZ_HandleTypeDef *hsubghz){}
// void HAL_SUBGHZ_LrFhssHopCallback(SUBGHZ_HandleTypeDef *hsubghz) {}
#else
  // integration with external irq
  static gpio_irq_chain_t __sx1262_irq;
#endif // (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0



// ============================================================
// Call when sigfox lib is initialized; prepare the hardware
// and set the DIO3 irq to call the given callback function when
// rising.
// ============================================================
SX126X_HW_API_status_t SX126X_HW_API_open(SX126X_HW_irq_cb_t callback)
{
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_open\r\n"));

	#ifdef ERROR_CODES
	 SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif


	// Configure the pin for Chip Select
	#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) == 0
		#if ITSDK_SFX_SX126X_CS_PIN != __LP_GPIO_NONE
	     gpio_configure(ITSDK_SFX_SX126X_CS_BANK, ITSDK_SFX_SX126X_CS_PIN, GPIO_OUTPUT_PP );
	     gpio_reset(ITSDK_SFX_SX126X_CS_BANK,ITSDK_SFX_SX126X_CS_PIN);
		#elif ITSDK_SFX_SX126X_CSDN_PIN != __LP_GPIO_NONE
	     gpio_configure(ITSDK_SFX_SX126X_CSDN_BANK, ITSDK_SFX_SX126X_CSDN_PIN, GPIO_OUTPUT_PP );
		 gpio_set(ITSDK_SFX_SX126X_CSDN_BANK,ITSDK_SFX_SX126X_CSDN_PIN);
		#else
		  #warning "You did not setup a CS pin for SX1262 SPI, are you sure ?"
		#endif
	#endif

	// Configure the pin used for RFSwitch
	if ( ITSDK_SFX_SX126X_RFSW1_PIN != __LP_GPIO_NONE ) {
		gpio_configure_ext(ITSDK_SFX_SX126X_RFSW1_BANK,ITSDK_SFX_SX126X_RFSW1_PIN,GPIO_OUTPUT_PP, ITSDK_GPIO_SPEED_HIGH, ITSDK_GPIO_ALT_NONE);
		gpio_reset(ITSDK_SFX_SX126X_RFSW1_BANK,ITSDK_SFX_SX126X_RFSW1_PIN);
	}
	if ( ITSDK_SFX_SX126X_RFSW2_PIN != __LP_GPIO_NONE ) {
		gpio_configure_ext(ITSDK_SFX_SX126X_RFSW2_BANK,ITSDK_SFX_SX126X_RFSW2_PIN,GPIO_OUTPUT_PP, ITSDK_GPIO_SPEED_HIGH, ITSDK_GPIO_ALT_NONE);
		gpio_reset(ITSDK_SFX_SX126X_RFSW2_BANK,ITSDK_SFX_SX126X_RFSW2_PIN);
	}

	// Configure the interruption handler and SPI
	#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0

		ITSDK_SFX_SX126X_SPI.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
		HAL_StatusTypeDef r = HAL_SUBGHZ_Init(&ITSDK_SFX_SX126X_SPI);
	    if (r != HAL_OK) {
		  #ifdef ERROR_CODES
	    	status = SX126X_HW_API_ERROR;
		  #endif
	      LOG_DEBUG_SFXSX126X(("[SX] Failed to init subghz %d\r\n",r));
	      RETURN();
	    }

	    __sx1262_irq_cb = callback;
		__HAL_RCC_SUBGHZSPI_CLK_ENABLE();
		HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);

	#else
		if ( ITSDK_SFX_IRQ_PIN != __LP_GPIO_NONE ) {
			gpio_interruptClear(ITSDK_SFX_IRQ_BANK, ITSDK_SFX_IRQ_PIN);
			gpio_configure(ITSDK_SFX_IRQ_BANK, ITSDK_SFX_IRQ_PIN, GPIO_INTERRUPT_RISING );
			gpio_interruptPriority(ITSDK_SFX_IRQ_BANK,ITSDK_SFX_IRQ_PIN,0,0);
			__sx1262_irq.irq_func = callback;
			__sx1262_irq.pinMask = ITSDK_SFX_IRQ_PIN;
			gpio_registerIrqAction(&__sx1262_irq);
			gpio_interruptEnable(ITSDK_SFX_IRQ_BANK, ITSDK_SFX_IRQ_PIN);
		}
	#endif

    RETURN();
}

SX126X_HW_API_status_t SX126X_HW_API_close(void)
{
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_close\r\n"));

	// Unset the pin for Chip Select
	#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) == 0
		#if ITSDK_SFX_SX126X_CS_PIN != __LP_GPIO_NONE
	     gpio_configure(ITSDK_SFX_SX126X_CS_BANK, ITSDK_SFX_SX126X_CS_PIN, GPIO_OFF );
		#elif ITSDK_SFX_SX126X_CSDN_PIN != __LP_GPIO_NONE
	     gpio_configure(ITSDK_SFX_SX126X_CSDN_BANK, ITSDK_SFX_SX126X_CSDN_PIN, GPIO_OFF );
		#else
		  #warning "You did not setup a CS pin for SX1262 SPI, are you sure ?"
		#endif
	#endif

	// Unset pin used for RFSwitch
	if ( ITSDK_SFX_SX126X_RFSW1_PIN != __LP_GPIO_NONE ) gpio_reset(ITSDK_SFX_SX126X_RFSW1_BANK,ITSDK_SFX_SX126X_RFSW1_PIN);
	if ( ITSDK_SFX_SX126X_RFSW2_PIN != __LP_GPIO_NONE ) gpio_reset(ITSDK_SFX_SX126X_RFSW2_BANK,ITSDK_SFX_SX126X_RFSW2_PIN);
	if ( ITSDK_SFX_SX126X_RFSW1_PIN != __LP_GPIO_NONE ) gpio_configure(ITSDK_SFX_SX126X_RFSW1_BANK,ITSDK_SFX_SX126X_RFSW1_PIN,GPIO_OFF);
	if ( ITSDK_SFX_SX126X_RFSW2_PIN != __LP_GPIO_NONE ) gpio_configure(ITSDK_SFX_SX126X_RFSW2_BANK,ITSDK_SFX_SX126X_RFSW2_PIN,GPIO_OFF);

	// disable interrupt
	#if ( (ITSDK_WITH_SPI) & __SPI_SUBGHZ ) > 0

		__HAL_RCC_SUBGHZSPI_CLK_DISABLE();
		HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);

	#else
		if ( ITSDK_SFX_IRQ_PIN != __LP_GPIO_NONE ) {
			gpio_interruptClear(ITSDK_SFX_IRQ_BANK, ITSDK_SFX_IRQ_PIN);
			gpio_configure(ITSDK_SFX_IRQ_BANK, ITSDK_SFX_IRQ_PIN, GPIO_STOP );
			gpio_removeIrqAction(&__sx1262_irq);
		}
	#endif

	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif

    RETURN();
}

// ------------------------------------------------------------------
// Wait for the given number of ms
// ------------------------------------------------------------------
SX126X_HW_API_status_t SX126X_HW_API_delayMs(unsigned short delay_ms)
{
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_delayMs\r\n"));
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
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_get_chip_name\r\n"));
	#if ITSDK_SFX_SX126X_CHIP == __SX1261
	 *chipset = SX126X_HW_API_CHIP_NAME_SX1261;
	#elif ITSDK_SFX_SX126X_CHIP == __SX1262 || ITSDK_SFX_SX126X_CHIP == __E5WL
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
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_get_reg_mode\r\n"));

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


// ------------------------------------------------------------------
// Return the oscillator configuration
// TCXO powered can be controlled by the SX126x DIO3 pin, in this case,
//  the tcxo_is_radio_controlled is set to 1 else 0. When controlled by
//  sx126x the power voltage can be set with supply_voltage, else this
//  voltage is ignored.
//  The startup_time_in_tick also only used when TCXO controlled by SX126x
// ------------------------------------------------------------------
SX126X_HW_API_status_t SX126X_HW_API_get_xosc_cfg(SX126X_HW_API_xosc_cfg_t *xosc_cfg)
{
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_get_xosc_cfg\r\n"));
	#if !defined ITSDK_SFX_SX126X_TCXO_STIKS || !defined ITSDK_SFX_SX126X_TCXO_SXCTL
	  #error "The ITSDK_SFX_SX126X_TCXO_STIKS must be defined"
	#endif

	xosc_cfg->startup_time_in_tick = ITSDK_SFX_SX126X_TCXO_STIKS;
	#if ITSDK_SFX_SX126X_TCXO_SXCTL == __DISABLE
	 xosc_cfg->tcxo_is_radio_controlled = 0;
	#else
	 xosc_cfg->tcxo_is_radio_controlled = 1;
	#endif

	#if ITSDK_SFX_SX126X_TCXO_PWR == 16
		xosc_cfg->supply_voltage = SX126X_HW_API_TCXO_CTRL_1_6V;
	#elif ITSDK_SFX_SX126X_TCXO_PWR == 17
		xosc_cfg->supply_voltage = SX126X_HW_API_TCXO_CTRL_1_7V;
	#elif ITSDK_SFX_SX126X_TCXO_PWR == 18
		xosc_cfg->supply_voltage = SX126X_HW_API_TCXO_CTRL_1_8V;
	#elif ITSDK_SFX_SX126X_TCXO_PWR == 22
		xosc_cfg->supply_voltage = SX126X_HW_API_TCXO_CTRL_2_2V;
	#elif ITSDK_SFX_SX126X_TCXO_PWR == 24
		xosc_cfg->supply_voltage = SX126X_HW_API_TCXO_CTRL_2_4V;
	#elif ITSDK_SFX_SX126X_TCXO_PWR == 27
		xosc_cfg->supply_voltage = SX126X_HW_API_TCXO_CTRL_2_7V;
	#elif ITSDK_SFX_SX126X_TCXO_PWR == 30
		xosc_cfg->supply_voltage = SX126X_HW_API_TCXO_CTRL_3_0V;
	#elif ITSDK_SFX_SX126X_TCXO_PWR == 33
		xosc_cfg->supply_voltage = SX126X_HW_API_TCXO_CTRL_3_3V;
	#else
		#error "The ITSDK_SFX_SX126X_TCXO_PWR must be defined"
	#endif

	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif

    RETURN();
}

// -----------------------------------------------------------------
// Configure the Rf Switch and the powering based on the
// configuration required
// the way PA is configured can be board specific, so it is
// possible to override this function for specific modules / implementation
// paSelected: __SX1262_PA_LP / __SX126X_PA_HP
// rxTx: __SX126X_RXTX_OFF, __SX126X_RX, __SX126X_TX
// -----------------------------------------------------------------
__weak void _sx126x_rfSwitchSet(uint8_t paSelected, uint8_t rxTx) {

	LOG_DEBUG_SFXSX126X(("[SX] _sx126x_rfSwitchSet (%d) (%d)\r\n",paSelected,rxTx));
	if ( ( rxTx & __SX126X_TX ) > 0 ) {
		// we are in transmission mode
		if ( paSelected == __SX126X_PA_LP ) {
		  #if ITSDK_SFX_SX1262_REGULOR  == __POWER_DCDC
			// Optimize power source (SMPS)
			uint8_t value;
			HAL_SUBGHZ_ReadRegister(&ITSDK_SFX_SX126X_SPI,__SX126X_REG_SMPSC2R,(uint8_t*)&value);
			value = (value & (~__SX126X_SMPS_DRV_MASK)) | __SX126X_SMPS_DRV_60;
			HAL_SUBGHZ_WriteRegister(&ITSDK_SFX_SX126X_SPI,__SX126X_REG_SMPSC2R,value);
		  #endif
		}
	}
	#if ITSDK_SFX_SX126X_MODULE == __SX126X_MOD_LORAE5

		if( rxTx == __SX126X_RXTX_OFF ) {
			// Radio Off
			gpio_reset(ITSDK_SFX_SX126X_RFSW1_BANK,ITSDK_SFX_SX126X_RFSW1_PIN);
			gpio_reset(ITSDK_SFX_SX126X_RFSW2_BANK,ITSDK_SFX_SX126X_RFSW2_PIN);
		} else if ( rxTx == __SX126X_RX ) {
			// Rx mode
			gpio_set(ITSDK_SFX_SX126X_RFSW1_BANK,ITSDK_SFX_SX126X_RFSW1_PIN);
			gpio_reset(ITSDK_SFX_SX126X_RFSW2_BANK,ITSDK_SFX_SX126X_RFSW2_PIN);
		} else if ( rxTx == __SX126X_TX ) {
			if ( paSelected == __SX126X_PA_LP ) {
				// Low Power
				gpio_set(ITSDK_SFX_SX126X_RFSW1_BANK,ITSDK_SFX_SX126X_RFSW1_PIN);
				gpio_set(ITSDK_SFX_SX126X_RFSW2_BANK,ITSDK_SFX_SX126X_RFSW2_PIN);
			} else if ( paSelected == __SX126X_PA_HP ) {
				// High Power
				gpio_reset(ITSDK_SFX_SX126X_RFSW1_BANK,ITSDK_SFX_SX126X_RFSW1_PIN);
				gpio_set(ITSDK_SFX_SX126X_RFSW2_BANK,ITSDK_SFX_SX126X_RFSW2_PIN);
			} else {
				// invalid situation where tx & rx are on
				LOG_ERROR_SFXSX126X(("[SX] _sx126x_rfSwitchSet - LP & HP on or off %d\r\n",paSelected));
			}
		} else {
			// invalid situation where tx & rx are on
			LOG_ERROR_SFXSX126X(("[SX] _sx126x_rfSwitchSet - Tx & Rx on\r\n"));
		}

	#else
	  #warning "The _sx126x_rfSwitchSet is not configured for your board, select a board or override the function"
	#endif

}

// ------------------------------------------------------------------
// Power Amplifier configuration
//  This is corresponding to the SetPaConfig SPI Transaction chap 13.1.14 in SX1261-2 Datasheet V2.1
// - device_sel - select between sx1261 value 1 or sx1262 value 0
// - hp_max - for sx1262 only, to limit the power 2 = +14dBm, 3 = +17dBm, 5 = +20dBm, 7 = +22dBm
//            for sx1261 use 0
// - pa_duty_cycle - impact power distribution in the harmonics
//                   sx1262 recommended values for hp_max 0x04 for +22dBm, 0x03 for +20dBm, 0x02 for +17dBm and +14dBm
//                   sx1261 recommended values 0x06 for +15dBm, 0x04 for +14dBm, 0x01 for +10dBm
//                   if Freq > 400 MHz max 0x07 for sx1261 / if Freq < 400MHz max 0x07 for sx1261
//                   max value 0x04 for sx1262
// - pa_lut - always 1, reserved
// - power - if related to the setTxParam transaction chap 13.4.4 in SX1261-2 Datasheet V2.1
//		set the output power between -17 to +14dBm for low power PA and -9 to +22dBm for high power
// ------------------------------------------------------------------
static uint8_t __sx126x_lastTxConfig = __SX126X_PA_NONE;	// store last PA for RF switch
SX126X_HW_API_status_t SX126X_HW_API_get_pa_pwr_cfg(SX126X_HW_API_pa_pwr_cfg_t *pa_pwr_cfg, sfx_u32 rf_freq_in_hz, sfx_s8 expected_output_pwr_in_dbm)
{
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_get_pa_pwr_cfg\r\n"));

	#if ITSDK_SFX_SX126X_CHIP == __E5WL
	    uint8_t pa = __SX126X_PA_LP;
		if ( ITSDK_SFX_SX126X_PA_SELECT ==  __SX126X_PA_LPHP ) {
			if ( expected_output_pwr_in_dbm  > 15 ) pa = __SX126X_PA_HP;
		} else if ( ITSDK_SFX_SX126X_PA_SELECT ==  __SX126X_PA_HP ) pa = __SX126X_PA_HP;

		__sx126x_lastTxConfig = pa;
		if ( pa == __SX126X_PA_LP ) {
			// Low Power PA
			if ( expected_output_pwr_in_dbm == 15 ) {
				 pa_pwr_cfg->pa_config.pa_duty_cycle = 6;
				 pa_pwr_cfg->pa_config.hp_max = 0;
				 pa_pwr_cfg->pa_config.device_sel = 1;
			} else {
				 pa_pwr_cfg->pa_config.pa_duty_cycle = 4;
				 pa_pwr_cfg->pa_config.hp_max = 0;
				 pa_pwr_cfg->pa_config.device_sel = 1;
			}
			if ( expected_output_pwr_in_dbm > 14 ) expected_output_pwr_in_dbm = 14;
			if ( expected_output_pwr_in_dbm < -17 ) expected_output_pwr_in_dbm = -17;

			uint8_t value = 0x18;
			HAL_SUBGHZ_WriteRegister(&ITSDK_SFX_SX126X_SPI,__SX126X_REG_OCP,value); // current max 160mA for the whole device
		} else {
			// High Power PA
			uint8_t value;
			HAL_SUBGHZ_ReadRegister(&ITSDK_SFX_SX126X_SPI,__SX126X_REG_TX_CLAMP,(uint8_t*)&value);
			value |= ( 0x0F << 1 );
			HAL_SUBGHZ_WriteRegister(&ITSDK_SFX_SX126X_SPI,__SX126X_REG_TX_CLAMP,value); // Better Resistance of the SX1262 Tx to Antenna Mismatch see chap 15.2

			pa_pwr_cfg->pa_config.pa_duty_cycle = 4;
			pa_pwr_cfg->pa_config.hp_max = 7;
			pa_pwr_cfg->pa_config.device_sel = 0;
			if ( expected_output_pwr_in_dbm > 22 ) expected_output_pwr_in_dbm = 22;
			if ( expected_output_pwr_in_dbm < -9 ) expected_output_pwr_in_dbm = -9;

			value = 0x38;
			HAL_SUBGHZ_WriteRegister(&ITSDK_SFX_SX126X_SPI,__SX126X_REG_OCP,value); // current max 160mA for the whole device
		}
	#elif ITSDK_SFX_SX126X_CHIP == __SX1261
		pa_pwr_cfg->pa_config.device_sel = 1;
		#if ITSDK_SFX_SX126X_MAX_PWR <= 10
		 pa_pwr_cfg->pa_config.pa_duty_cycle = 1;
		 pa_pwr_cfg->pa_config.hp_max = 0;
		#elif ITSDK_SFX_SX126X_MAX_PWR <= 14
		 pa_pwr_cfg->pa_config.pa_duty_cycle = 4;
		 pa_pwr_cfg->pa_config.hp_max = 0;
		#elif ITSDK_SFX_SX126X_MAX_PWR <= 15
		 pa_pwr_cfg->pa_config.pa_duty_cycle = 6;
		 pa_pwr_cfg->pa_config.hp_max = 0;
		#else
		 #warning "ITSDK_SFX_SX126X_MAX_PWR is out of range"
		#endif
	#elif ITSDK_SFX_SX126X_CHIP == __SX1262
		pa_pwr_cfg->pa_config.device_sel = 0;
		#if ITSDK_SFX_SX126X_MAX_PWR <= 14
		 pa_pwr_cfg->pa_config.pa_duty_cycle = 2;
 		 pa_pwr_cfg->pa_config.hp_max = 2;
		#elif ITSDK_SFX_SX126X_MAX_PWR <= 17
		 pa_pwr_cfg->pa_config.pa_duty_cycle = 2;
 		 pa_pwr_cfg->pa_config.hp_max = 3;
		#elif ITSDK_SFX_SX126X_MAX_PWR <= 20
		 pa_pwr_cfg->pa_config.pa_duty_cycle = 3;
 		 pa_pwr_cfg->pa_config.hp_max = 5;
		#elif ITSDK_SFX_SX126X_MAX_PWR <= 22
		 pa_pwr_cfg->pa_config.pa_duty_cycle = 4;
 		 pa_pwr_cfg->pa_config.hp_max = 7;
		#else
		 #warning "ITSDK_SFX_SX126X_MAX_PWR is out of range"
		#endif
	#else
		#error "You must define ITSDK_SFX_SX126X_CHIP with a valid value"
	#endif

	#if	ITSDK_SFX_SX126X_PA_DC_OVR != __AVOID
		pa_pwr_cfg->pa_config.pa_duty_cycle = ITSDK_SFX_SX126X_PA_DC_OVR
	#endif
	#if	ITSDK_SFX_SX126X_PA_HPM_OVR != __AVOID
		pa_pwr_cfg->pa_config.hp_max = ITSDK_SFX_SX126X_PA_HPM_OVR
	#endif

	// Make sure the pa_duty_cycle doesnot exceed the max accepted value (cf Chap 13.1.14)
	#if ITSDK_SFX_SX126X_CHIP == __SX1261
		if ( rf_freq_in_hz >= 400000000 && pa_pwr_cfg->pa_config.pa_duty_cycle > 7 ) pa_pwr_cfg->pa_config.pa_duty_cycle = 7;
		if ( rf_freq_in_hz <= 400000000 && pa_pwr_cfg->pa_config.pa_duty_cycle > 4 ) pa_pwr_cfg->pa_config.pa_duty_cycle = 4;
	#elif ITSDK_SFX_SX126X_CHIP == __SX1262 || ITSDK_SFX_SX126X_CHIP == __E5WL
		if ( pa_pwr_cfg->pa_config.pa_duty_cycle > 4 ) pa_pwr_cfg->pa_config.pa_duty_cycle = 4;
	#endif

	// reserved value
	pa_pwr_cfg->pa_config.pa_lut = 1;

	// power
	if ( expected_output_pwr_in_dbm > ITSDK_SFX_SX126X_MAX_PWR ) {
		pa_pwr_cfg->power = ITSDK_SFX_SX126X_MAX_PWR;
	} else {
		pa_pwr_cfg->power = expected_output_pwr_in_dbm;
	}


	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif
    RETURN();
}


// ------------------------------------------------------------------
// Radio switch configuration
// Set the Tx on, we need to use the right PA setting, it depends
// on the chip and the power amplifier configuration can be required
// ------------------------------------------------------------------
SX126X_HW_API_status_t SX126X_HW_API_tx_on(void)
{
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_tx_on\r\n"));
	_sx126x_rfSwitchSet(__sx126x_lastTxConfig,__SX126X_TX);
	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif
    RETURN();
}

SX126X_HW_API_status_t SX126X_HW_API_tx_off(void)
{
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_tx_off\r\n"));
	_sx126x_rfSwitchSet(__sx126x_lastTxConfig,__SX126X_RXTX_OFF);
	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif
    RETURN();
}

#ifdef BIDIRECTIONAL
SX126X_HW_API_status_t SX126X_HW_API_rx_on(void)
{
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_rx_on\r\n"));
	_sx126x_rfSwitchSet(__sx126x_lastTxConfig,__SX126X_RX);
	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif
    RETURN();
}
#endif

#ifdef BIDIRECTIONAL
SX126X_HW_API_status_t SX126X_HW_API_rx_off(void)
{
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_rx_off\r\n"));
	_sx126x_rfSwitchSet(__sx126x_lastTxConfig,__SX126X_RXTX_OFF);
	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif
    RETURN();
}
#endif


// ------------------------------------------------------------------
// LATENCY COMPENSATION
// ...
// ------------------------------------------------------------------

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
SX126X_HW_API_status_t SX126X_HW_API_get_latency(SX126X_HW_API_latency_t latency_type, sfx_u32 *latency_ms)
{
	LOG_DEBUG_SFXSX126X(("[SX] SX126X_HW_API_get_latency\r\n"));
	switch ( latency_type ) {
	case SX126X_HW_API_LATENCY_RESET:
	case SX126X_HW_API_LATENCY_WAKEUP:
		*latency_ms = ITSDK_SFX_SX126X_LATENCYMS;
		break;
	default:
		 LOG_ERROR_SFXSX126X(("[SX] SX126X_HW_API_get_latency - Invalid type %d\r\n",latency_type));
		 break;
	}

	#ifdef ERROR_CODES
     SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
	#endif
    RETURN();
}
#endif


#endif // ITSDK_WITH_SIGFOX_LIB == __ENABLE && ITSDK_SIGFOX_LIB == __SIGFOX_SX126X
