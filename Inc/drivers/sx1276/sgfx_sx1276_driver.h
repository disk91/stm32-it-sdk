/******************************************************************************
 * @file    sgfx_sx1276_driver.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   modulation library driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
#ifndef SFX_SX1276_DRIVER_H
#define SFX_SX1276_DRIVER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum 
{
  MOD_ERROR = 0, 
  MOD_SUCCESS = !MOD_ERROR
} mod_error_t;

typedef enum{
  CS_BW_200KHZ =200,
  CS_BW_300KHZ =300,
} e_cs_bw;


/* Exported constants --------------------------------------------------------*/
#define TX_POWER_13DBM ( (int8_t) 13 )
#define TX_POWER_14DBM ( (int8_t) 14 )
#define TX_POWER_20DBM ( (int8_t) 20 )
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

/**
 * @brief void Configure the Radio to Sigfox transmit
 * @param[in] None
 */ 
void SGFX_SX1276_tx_config( void );
   
/**
 * @brief void Configure the Radio to Sigfox receive
 * @param[in] None
 */
void SGFX_SX1276_rx_config( void );

/**
 * @brief void Configure the receive bandwidth for carrier sense
 * @param[in] None
 */
void SGFX_SX1276_rx_setbw( e_cs_bw cs_bw );

/**
 * @brief void Start the Radio in Sigfox transmit mode
 * @param[in] stream: point to the tx transmit buffer
 * @param[in] size: size of the stream in bytes
 * @param[in] baudrate of the tranmittion. Either 100 bit/s or 600 bit/s 
 * @retval  mod_error_t
 */
mod_error_t SGFX_SX1276_tx( uint8_t *stream, uint8_t size, uint16_t baudRate );


/**
 * @brief Start the Radio in Sigfox receive mode
 * @param[in] none
 * @retval  mod_error_t
 */
mod_error_t SGFX_SX1276_rx_start( void );

/**
 * @brief stops the Radio in Sigfox receive mode
 * @param[in] none
 * @retval  mod_error_t if received frame length is greater than 15 bytes
 */
mod_error_t SGFX_SX1276_rx_stop(uint8_t *frame);

/**
 * @brief  Stop the Radio RF activity and back to stop mode
 *
 * @param[in] None
 */
void SGFX_SX1276_stop( void);

/**
 * @brief void Start the Radio unmodulated continuous wave mode
 * @param[in] None
 * @retval  mod_error_t
 */
mod_error_t SGFX_SX1276_start_txtest_cw(void);

/**
 * @brief void Start the Radio BPSK modulated continuous wave mode
 * @param[in] bitrate 100 or 600
 * @retval  mod_error_t
 */
mod_error_t SGFX_SX1276_start_txtest_prbs9( uint16_t bitrate );

/**
 * @brief void Stop the Radio from continuous wave mode
 * @param[in] None
 * @retval  mod_error_t
 */
mod_error_t SGFX_SX1276_stop_txtest( void );

/**
 * @brief void set the radio Tx power
 * @note internal power variable used for cw and sigfox transmission
 * @note power between 10 and 20dBm
 * @param[in] power in dBm
 * @retval  mod_error_t
 */
mod_error_t SGFX_SX1276_setPower(int8_t power );

/**
 * @brief void get the radio Tx power
 * @note internal power variable used for cw and sigfox transmission
 * @retval  power in dBm
 */
int8_t SGFX_SX1276_getPower(void );

/**
 * @brief get the modulation library version
 * @retval return a string containing the library version
 */
char* SGFX_SX1276_get_version( void );

#ifdef __cplusplus
}
#endif

#endif  /* SFX_SX1276_DRIVER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
