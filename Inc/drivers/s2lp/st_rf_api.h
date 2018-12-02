
/*!
 * \file st_rf_api.h
 * \brief Sigfox manufacturer functions
 * \author  AMG - RF Application team
 * \version 2.2.2
 * \date November 24, 2017
 * \copyright COPYRIGHT 2017 STMicroelectronics
 *
 * This file defines the manufacturer's RF functions to be implemented
 * for library usage.
 */
#ifndef IT_SDK_DRIVERS_S2LP_RF_H_
#define IT_SDK_DRIVERS_S2LP_RF_H_

#include <drivers/sigfox/sigfox_types.h>


#define ST_RF_ERR_API_ERROR                   (sfx_u8)(0x01) /*!< Error on ST_RF_API */

#define TIMER_START  0
#define TIMER_STOP   1

/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_set_rssi_offset(sfx_s8 rssi_off)
 * \brief Set an RSSI offset for the RSSI.
 * \param[in] sfx_s8 rssi_off: an integer representing the offset in dB.
 *                  Default value is 0.
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_set_rssi_offset(sfx_s8 rssi_off);


/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_get_rssi_offset(sfx_s8 *rssi_off)
 * \brief Get the RSSI offset for the RSSI.
 * \param[in] sfx_s8* rssi_off: a pointer to the integer representing the offset in dB.
 *                  Default value is 0.
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_get_rssi_offset(sfx_s8 *rssi_off);


/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_gpio_irq_pin(sfx_u8 gpio_pin)
 * \brief Configures one of the S2-LP pin to be an IRQ pin.
 * \param[in] sfx_u8 gpio_pin: an integer in the range [0,3] representing the GPIO to be used as IRQ.
 *                  Default value is 3.
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_gpio_irq_pin(sfx_u8 gpio_pin);

/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_gpio_tx_rx_pin(sfx_u8 gpio_pin)
 * \brief Configures one of the S2-LP pin to be to be configured as (RX or TX) signal.
 * \param[in] sfx_u8 gpio_pin: an integer in the range [0,3] representing the GPIO to be configured as (RX or TX) signal.
 *                  Pass the value 0xFF if the GPIO should not be configured.
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_gpio_tx_rx_pin(sfx_u8 gpio_pin);

/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_gpio_rx_pin(sfx_u8 gpio_pin)
 * \brief Configures one of the S2-LP pin to be configured as RX signal.
 * \param[in] sfx_u8 gpio_pin: an integer in the range [0,3] representing the GPIO to be configured as RX signal.
 *                  Pass the value 0xFF if the GPIO should not be configured.
 * \note Only for RCZ2/4. Uneffective for RCZ1. This function must be called before \ref ST_SIGFOX_API_open .
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_gpio_rx_pin(sfx_u8 gpio_pin);


/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_gpio_tx_pin(sfx_u8 gpio_pin)
 * \brief Configures one of the S2-LP pin to be configured as TX signal.
 * \param[in] sfx_u8 gpio_pin: an integer in the range [0,3] representing the GPIO to be configured as TX signal.
 *                      Pass the value 0xFF if the GPIO should not be configured.
 * \note Only for RCZ2/4. Uneffective for RCZ1. This function must be called before \ref ST_SIGFOX_API_open .
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_gpio_tx_pin(sfx_u8 gpio_pin);


/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_reduce_output_power(sfx_s16 reduction)
 * \brief Reduces the output power of the transmitted signal by a facor (reduction*0.5dB against the actual value).
 * \details       Each positive step of 1 reduces the power at S2-LP level of about 0.5dB. A negative value increase the power level of the same quantity.
 *        <br>The function returns an error if the output power is bigger than the one used dutring cerification.
 * \param[in] sfx_s16 reduction: the reduction factor.
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_reduce_output_power(sfx_s16 reduction);

/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_set_xtal_freq(sfx_u32 xtal)
 * \brief Sets the XTAL frequency of the S2-LP in Hertz (default is 50MHz).
 * \param[in] sfx_u32 xtal: the xtal frequency of the S2-LP in Hz as an integer.
 * \note If this function is not called, the default xtal frequency is 50MHz.
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_set_xtal_freq(sfx_u32 xtal);

/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_set_freq_offset(sfx_s32 offset)
 * \brief Sets the RF frequency offset in Hertz (default is 0 Hz).
 * \param[in] sfx_s32 offset: frequency offset in Hz as an integer.
 * \note If this function is not called, the default frequency offset is 0 Hz.
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_set_freq_offset(sfx_s32 offset);


/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_set_tcxo(sfx_u8 tcxo)
 * \brief Instructs the library to configure the S2-LP for a TCXO or for a XTAL.
 * \param[in] sfx_u8 tcxo: 1 if a TCXO, 0 if XTAL.
 * \note If this function is not called, the default is to use the S2-LP in XTAL mode.
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_set_tcxo(sfx_u8 tcxo);


/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_smps(sfx_u8 mode)
 * \brief Instructs the library to configure the S2-LP with a user defined smps frequency.
 * \param[in] sfx_u8 mode: from 1 (1.2V) to 7 (1.8V).
 * \note If this function is not called, the default is to use the S2-LP at 1.5V.
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_smps(sfx_u8 mode);

/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_set_pa(sfx_u8 pa)
 * \brief Instructs the library to configure the S2-LP for a external PA (Power Amplifier).
 * \param[in] sfx_u8 pa: 1 if a PA, 0 if not.
 * \note If this function is not called, the default is not configure an external PA.
 * \retval 0 if no error, 1 otherwise.
 *******************************************************************/
sfx_u8 ST_RF_API_set_pa(sfx_u8 pa);

/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_get_ramp_duration(void)
 * \brief Returns the duration of the initial (or final) ramp in ms.
 * \param[in] None.
 * \retval Ramp duration in ms.
 *******************************************************************/
sfx_u8 ST_RF_API_get_ramp_duration(void);

/*!******************************************************************
 * \fn void ST_RF_API_S2LP_IRQ_CB(void)
 * \brief This is a <b>callback</b> exported by the RF_API library.
 *      The RF_API module configures the S2-LP to raise interrupts and to notify them 
 *      on a GPIO. When the interrupt of that GPIO is raised, this function must be called.
 *      It must be called when the S2-LP raises the IRQ via GPIO.
 * \param[in] None.
 * \retval None.
 *******************************************************************/
void ST_RF_API_S2LP_IRQ_CB(void);

/*!******************************************************************
 * \fn void ST_RF_API_Timer_CB(void)
 * \brief This is a <b>callback</b> exported by the RF_API library. 
 *      It must be called when the timer started by \ref MCU_API_timer_start expires.
 * \param[in] sfx_u8 state: 0 for timer start, 1 for timer stop
 * \retval None.
 *******************************************************************/
void ST_RF_API_Timer_CB(sfx_u8 state);

/*!******************************************************************
 * \fn void ST_RF_API_Timer_Channel_Clear_CB(void)
 * \brief This is a <b>callback</b> exported by the RF_API library. 
 *      It must be called when the timer started by \ref MCU_API_timer_start_carrier_sense expires.
 * \param[in] None.
 * \retval None.
 *******************************************************************/
void ST_RF_API_Timer_Channel_Clear_CB(void);

/*!******************************************************************
 * \fn sfx_u8 ST_RF_API_Get_Continuous_TX_Flag(void);
 * \brief This is a function that give informations abou the tx state MCU API. 
 * 		Used for the implementation for continuos BPSK modulation.
 * \param[in] None.
 * \retval 0 IDLE state or TX send frame, 1 Continuos BPSK mode.
 *******************************************************************/
sfx_u8 ST_RF_API_Get_Continuous_TX_Flag(void);

/*!******************************************************************
 * \fn void ST_RF_API_StartTx(void)
 * \brief This is a function to force S2LP to switch in TX mode.  
 *      It is called from Monarch detection algorithm.
 * \param[in] None.
 * \retval None.
 *******************************************************************/

sfx_u8 ST_RF_API_StartTx(void);

/*!******************************************************************
 * \fn void ST_RF_API_StartRx(void)
 * \brief This is a function to force S2LP to switch in RX mode.  
 *      It is called from Monarch detection algorithm.
 * \param[in] None.
 * \retval None.
 *******************************************************************/

sfx_u8 ST_RF_API_StartRx(void);

/*!******************************************************************
 * \fn void ST_RF_API_StopRxTx(void)
 * \brief This is a function to force S2LP to switch in Ready mode.  
 *      It is called from Monarch detection algorithm.
 * \param[in] None.
 * \retval None.
 *******************************************************************/

sfx_u8 ST_RF_API_StopRxTx(void);

/*!******************************************************************
 * \fn void ST_RF_API_GetRSSI(void)
 * \brief This is a function to Get the latest detected RSSI  
 *      It is called from Monarch detection algorithm.
 * \param[in] None.
 * \retval RSSI level.
 *******************************************************************/

sfx_s16 ST_RF_API_GetRSSI(void);

#endif // IT_SDK_DRIVERS_S2LP_RF_H_
