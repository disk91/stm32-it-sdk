/******************************************************************************
 * @file    st_lowlevel.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   interface modulation library low level 
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

#ifndef STLL_H_
#define STLL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef enum {
  STLL_ERROR = 0, 
  STLL_SUCCESS = !STLL_ERROR
} STLL_Status;

typedef enum 
{
  STLL_RESET = 0, 
  STLL_SET = !STLL_RESET
} STLL_flag;

typedef enum {
  STLL_ENABLE=0, 
  STLL_DISABLE
} STLL_State;
  
typedef enum {
  HSE_SOURCE,
  HSI_SOURCE
} stll_clockType_e;
  
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

/*!
 * @brief Read the radio registers via SPI.
 * @param[in] uint8_t address: address of the register.
 * @retval the value read
 */
uint8_t STLL_Radio_ReadReg(uint8_t address);

/*!
 * @brief Write the radio registers via SPI.
 * @param[in] uint8_t address: address of the register.
 * @param[in] uint8_t data: the data to write.
 * @retval None
 */
void STLL_Radio_WriteReg( uint8_t addr, uint8_t data );

/*!
 * @brief Read the RX FIFO of the Radio via SPI.
 * @param[in] uint8_t n_bytes: number of elements to read from the RX FIFO.
 * @param[in] uint8_t *buffer: pointer to the uC memory where the read data must be stored.
 * @retval None
 */
void STLL_Radio_ReadFifo(uint8_t size, uint8_t* buffer);

/*!
 * @brief Write the TX FIFO of the Radio via SPI.
 * @param[in] uint8_t n_bytes: number of elements to write to the RX FIFO.
 * @param[in] uint8_t *buffer: pointer to the uC memory where the data to write are be stored.
 * @retval None
 */
void STLL_Radio_WriteFifo(uint8_t size, uint8_t* buffer);

/*!
 * @brief  Initialises the Radio
 * @param[in] None
 * @retval None.
 */
void STLL_Radio_Init( void );

/*!
 * @brief  calibrate the radio Radio Rest pin
 * @param[in] None
 * @retval None.
 */
void STLL_Radio_DeInit( void );
/*!
 * @brief  Initialises the Radio DIOs
 * @param[in] None
 * @retval None.
 */
void STLL_Radio_IoInit( void );

/*!
 * @brief  De-Initialises the Radio DIOs
 * @param[in] None
 * @retval None.
 */
void STLL_Radio_IoDeInit( void );

/*!
 * @brief  Sets the Radio OPerational Mode
 * @note   set the XO state if needed, set the RF swicth state, the the radio State
 * @param[in] opmode  
 * @param[in] #define RF_OPMODE_SLEEP                             0x00
 * @param[in] #define RF_OPMODE_TRANSMITTER                       0x03
 * @param[in] #define RF_OPMODE_RECEIVER                          0x05
 * @retval None.
 */
void STLL_Radio_SetOpMode(uint8_t opMode);

/*!
 * @brief  Set Radio Frequency
 * @param[in] Frequency in Hz
 * @retval None.
 */
void STLL_Radio_SetFreq( uint32_t Freq );

/*!
 * @brief  Set the Eerpom field of Radio Output Power
 * @param[in] power in dBm
 * @retval None.
 */
void STLL_RadioPowerSetEeprom( int8_t power);

/*!
 * @brief  Get Radio Output Power value stored in Eeprom
 * @note   power is recorded in eeprom
 * @param[in] None
 * @retval power in dBm.
 */
int8_t STLL_RadioPowerGet( void );

/*!
 * @brief  Set the Radio Output Power on the board
 * @param[in] power in dBm
 * @retval None.
 */
void STLL_RadioPowerSetBoard( int8_t power);

/*!
 * @brief  This function instructs the STLL to wait for the end of the transmitt frame.
 * @note   at end of frame STLL_SetEndOfTxFrame is called.
 * @retval returns STLL_SET. when frame is trasmitted, STLL_RESET when Rx timeOut occured.
 */
STLL_flag STLL_WaitEndOfTxFrame( void );

	
/*!
 * @brief  This function instructs the STLL that tx frame is finsihed.

 * @retval none.
 */
void STLL_SetEndOfTxFrame( void );

/*!
 * @brief Sends multiple ouput data with DMA *
 * @param [IN] Data buffer
 * @param [IN] Data buffer length
 * @retval none
 */
void STLL_Transmit_DMA_Start( uint16_t *pData, uint16_t Size);

/*!
 * @brief Stops DMA *
 * @param [IN] Data buffer
 * @param [IN] Data buffer length
 * @retval none
 */
void STLL_Transmit_DMA_Stop( void );

/*!
 * @brief starts  TIM2 object
 * @param [IN] none
 */
void STLL_TIM2_Start( void );

/*!
 * @brief stops  TIM2 object
 * @param [IN] none
 * @retval none
 */
void STLL_TIM2_Stop( void );

/*!
 * @brief sets  TIM2 object Period
 * @param [IN] period
 * @retval none 
 */
void STLL_TIM2_SetPeriod( uint32_t period );

/*!
 * @brief gets  TIM2 object Period
 * @param [IN] none
 * @return   the timer 2 period
 */
uint32_t STLL_TIM2_GetPeriod(  void );

/*!
 * @brief  This function instructs the STLL to allow STM32 in stop mode or not
 *              during the protocol operations.
 * @param[in] When DISABLE, the MCU is forbiden to go in STOP mode or stand by mode
 *            When ENABLE, the MCU is allowed to go in STOP mode or stand by mode
 * @retval None.
 */
void STLL_LowPower(STLL_State State);

/*!
 * @brief  This function sets the system clock
 * @param[in] from HSE_SOURCE or from HSI_SOURCE
 * @retval None.
 */
void STLL_SetClockSource( stll_clockType_e clocktype);

/*!
 * @brief  Is called at the end of the Tx bit
 * @param[in] none
 * @retval none.
 */
void STLL_TX_IRQHandler_CB( void );

/*!
 * @brief  This function instructs the STLL to wait for the end of the recive frame.
 * @note Rx frame is either finished by end of payload (DIO0, or by the timeOut timer
 * @param[in] none
 * @retval returns STLL_SET. when payload is received, STLL_RESET when Rx timeOut occured.
 */
STLL_flag STLL_WaitEndOfRxFrame( void );


int16_t STLL_SGFX_SX1276_GetSyncRssi(void);

int16_t STLL_RxCarrierSenseGetRssi( void );

STLL_flag STLL_RxCarrierSenseGetStatus( void );

void STLL_RxCarrierSenseInitStatus( void );


#endif /* STLL_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
