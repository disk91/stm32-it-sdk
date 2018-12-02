/*!
 * \file st_mcu_api.h
 * \brief Sigfox manufacturer functions
 * \author  AMG - RF Application team
 * \version 2.2.2
 * \date November 24, 2017
 * \copyright COPYRIGHT 2017 STMicroelectronics
 *
 * This file defines the manufacturer's MCU functions to be implemented
 * for library usage.
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ST_MCU_API_H
#define __ST_MCU_API_H

#ifdef __cplusplus
 extern "C" {
#endif
         
   
   
/*Structure to manage External PA from MCU*/
typedef enum
{
  SHUTDOWN     	= 0x00, 
  TX_BYPASS     = 0x01, 
  TX     		= 0x02, 
  RX     		= 0x03, 
}ExtPaStatus;

/********************************************************
 * External API dependencies to link with this library.
 *
 * Error codes of the MCU API functions are described below.
 * The Manufacturer can add more error code taking care of the limits defined.
 * 
 ********************************************************/


/*!******************************************************************
 * \fn void ST_MCU_API_SpiRaw(sfx_u8 n_bytes, sfx_u8* in_buffer, sfx_u8* out_buffer, sfx_u8 can_return_bef_tx)
 * \brief Performs a SPI transaction with the S2-LP SPI slave.
 * \param[in] uint8_t bytes: The number of bytes involved in the SPI transaction.
 * \param[in] uint8_t* in_buffer: pointer to the buffer to be sent to the slave.
 * \param[in] uint8_t* out_buffer: pointer to the buffer where the received bytes should be stored.
 * \param[in] uint8_t can_return_bef_tx: flag specifying if the function should be blocking (0) or not (1). 
 *              This can be useful if the SPI is aided with DMA support.
 * \note This is a function that is not required by the SIGFOX_API nor by the RF_API library.
 * \retval None
 *******************************************************************/
void ST_MCU_API_SpiRaw(sfx_u8 n_bytes, sfx_u8* in_buffer, sfx_u8* out_buffer, sfx_u8 can_return_bef_tx);

/*!******************************************************************
 * \fn void ST_MCU_API_GpioIRQ(sfx_u8 pin, sfx_u8 new_state, sfx_u8 trigger)
 * \brief Enables or Disables the external interrupt on the microcontroller side. The interrupt must be set on the rising or falling edge of the input signal according to the trigger_flag.
 *  The pin number passed represents the GPIO number of the S2-LP.
 * \param[in] uint8_t pin: the GPIO pin of the S2-LP (integer from 0 to 3).
 * \param[in] uint8_t new_state: enable or disable the EXTI (can be 0 or 1).
 * \param[in] uint8_t trigger: trigger_flag:
 *                   1: rising edge
 *                   0: falling edge
 * \note This is a function that is not required by the SIGFOX_API nor by the RF_API library.
 * \retval None
 *******************************************************************/
void ST_MCU_API_GpioIRQ(sfx_u8 pin, sfx_u8 new_state, sfx_u8 trigger);

/*!******************************************************************
 * \fn void ST_MCU_API_Shutdown(sfx_u8 value)
 * \brief Set on or off the S2-LP by GPIO.
 * \param[in] sfx_u8 value: 
 *     if 1, the device should enter shutdown (OFF).
 *     if 0, the device should exit from shutdown (ON).
 * \retval None
 *******************************************************************/
void ST_MCU_API_Shutdown(sfx_u8 value);

/*!******************************************************************
 * \fn void ST_MCU_API_LowPower(sfx_u8 low_power_flag)
 * \brief  This function instructs the mcu_api to send the microcontroller in sleep or not
 *              during the protocol operations.
 *              It is mainly used for debugging purposes.
 * \param[in] uint8_t low_power_flag : enable the low power (1, default setting) or not (0).
 * \note This is a function that is not required by the SIGFOX_API nor by the RF_API library.
 * \retval None.
 *******************************************************************/
void ST_MCU_API_LowPower(sfx_u8 low_power_flag);

/*!******************************************************************
 * \fn void ST_MCU_API_WaitForInterrupt(void)
 * \brief Microcontroller waits for interrupt.
 *      This function is continously called by the library each time it waits for an event.
 *      This is useful if the application must trigger a state machine and not block the CPU waiting for an event from the library.
 *      For example, this can be a null implementation or can activate a low power mode of the microcontroller, tick a stack for
 *      dual radio applications or other type of state machines.
 * \param[in] None.
 * \note This is a function that is not required by the SIGFOX_API nor by the RF_API library.
 * \retval None.
 *******************************************************************/
void ST_MCU_API_WaitForInterrupt(void);

/*!******************************************************************
 * \fn void ST_MCU_API_SetSysClock(void)
 * \brief  This function is used to confgure the system clock when the STM32 exits
 *  the low power or at beginning of the application.
 * \param[in] None.
 * \note This is a function that is not required by the SIGFOX_API nor by the RF_API library.
 * \retval None.
 *******************************************************************/
void ST_MCU_API_SetSysClock(void);

/*!******************************************************************
 * \fn void ST_MCU_API_TimerCalibration(sfx_u16 duration_ms)
 * \brief  This function calibrates the RTC that is used by the st_lowlevel when the
 *              device goes in sleep.
 * \param[in] sfx_u16 duration_ms : duration of the calibration process in ms.
 * \note This is a function that is not required by the SIGFOX_API nor by the RF_API library.
 * \retval None.
 *******************************************************************/
void ST_MCU_API_TimerCalibration(sfx_u16 duration_ms);

/*!******************************************************************
 * \fn void ST_MCU_API_SetEncryptionPayload(uint8_t ePayloads)
 * \brief  This function toggles the payload encryption option.
 * \param[in] sfx_u8 ePayload : set to 1 to enable encryption, 0 to disable.
 * \retval None.
 *******************************************************************/
void ST_MCU_API_SetEncryptionPayload(sfx_u8 ePayload);

/************************************************************************/
/*
                     MONARCH REF DES APIs
*/
/************************************************************************/
void ST_MCU_API_SetExtPAStatus(ExtPaStatus pa_status);
sfx_u32 ST_MCU_API_CaptureGPIO(sfx_u8 pin);
void ST_MCU_API_InitOOKGpio(sfx_u8 pin);
void ST_MCU_API_Enable16KHzSamplingTimer(void);
void ST_MCU_API_Disable16KHzSamplingTimer(void);

#ifdef __cplusplus
}
#endif

#endif /* __ST_MCU_API_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
