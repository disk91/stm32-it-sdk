/******************************************************************************
 * @file    st_lowlevel.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   modulation library low level implementation
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

/* Includes ------------------------------------------------------------------*/
#include <it_sdk/config.h>
#if ( ITSDK_WITH_SIGFOX_LIB == __ENABLE ) && (ITSDK_SIGFOX_LIB == __SIGFOX_SX1276)

#if ITSDK_PLATFORM != __PLATFORM_STM32L0
#error The ST implementation requires use of Timer 2 and DMA complicated to abstract - need to be rewritten
#endif

#include <it_sdk/configSigfox.h>
#include <drivers/sx1276/sigfox_lowlevel.h>
#include <drivers/sx1276/sx1276.h>
#include <drivers/sx1276/sigfox_sx1276.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/eeprom/sdk_state.h>
#include "stm32l0xx.h"
#include "spi.h"


uint8_t STLL_Radio_ReadReg(uint8_t address) {
  return  SX1276Read( address);
}

void STLL_Radio_WriteReg( uint8_t addr, uint8_t data ) {
  SX1276Write(  addr,  data );
}

void STLL_Radio_ReadFifo(uint8_t size, uint8_t* buffer) {
  SX1276ReadBuffer( 0, buffer, size );
}

void STLL_Radio_WriteFifo(uint8_t size, uint8_t* buffer) {
  SX1276WriteBuffer( 0, buffer, size );
}

void STLL_Radio_Init( void ) {
  LOG_DEBUG_SFXSX1276((">> STLL_Radio_Init\r\n"));
  RadioEvents_t events = {NULL};
  SX1276Init( &events );
}

void STLL_Radio_DeInit( void ) {
  LOG_DEBUG_SFXSX1276((">> STLL_Radio_DeInit\r\n"));
  RadioEvents_t events = {NULL};
  SX1276Init( &events );
  SX1276Write(0x40 , 0x01 );	// set DIO3 from 'buffer empty' to NA to save current
}

static void __irqHandlers_dio0( uint16_t GPIO_Pin) {
	LOG_DEBUG_SFXSX1276((">> __irqHandlers_dio0\r\n"));
	sx1276_sigfox_state.rxPacketReceived = STLL_SET;		// Set a frame have been received
	sx1276_sigfox_state.timerEvent = SIGFOX_EVENT_SET;		// Clear the running timer event
//   SCH_SetEvt( TIMOUT_EVT );
}

/*!
 * @brief  Is called at the end of the synchro Ok
 * @note  Reads RSSI when syncro has occured
 * @param[in] none
 * @retval none.
 */
static void __irqHandlers_dio4( uint16_t GPIO_Pin) {
	LOG_DEBUG_SFXSX1276((">> __irqHandlers_dio4\r\n"));
	sx1276_sigfox_state.meas_rssi_dbm =  -( ((int16_t) STLL_Radio_ReadReg( REG_RSSIVALUE )) >> 1 ) -13 + itsdk_config.sdk.sigfox.rssiCal;
}

void STLL_Radio_IoInit( void )
{
  LOG_DEBUG_SFXSX1276((">> STLL_Radio_IoInit\r\n"));

  SX1276IoInit();

  gpio_interruptPriority(ITSDK_SX1276_DIO_0_BANK,ITSDK_SX1276_DIO_0_PIN,0,0);
  gpio_interruptEnable(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN);
  __sx1276_gpio_irq[0].irq_func = __irqHandlers_dio0;
  __sx1276_gpio_irq[0].pinMask = ITSDK_SX1276_DIO_0_PIN;
  gpio_registerIrqAction(&__sx1276_gpio_irq[0]);

  gpio_interruptPriority(ITSDK_SX1276_DIO_4_BANK,ITSDK_SX1276_DIO_4_PIN,0,0);
  gpio_interruptEnable(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN);
  __sx1276_gpio_irq[4].irq_func = __irqHandlers_dio4;
  __sx1276_gpio_irq[4].pinMask = ITSDK_SX1276_DIO_4_PIN;
  gpio_registerIrqAction(&__sx1276_gpio_irq[4]);

  
  //HW_GPIO_SetIrq( RADIO_DIO_0_PORT, RADIO_DIO_0_PIN, IRQ_PRIORITY_RADIO, RADIO_rx_packet_interrupt_handler );
  //HW_GPIO_SetIrq( RADIO_DIO_4_PORT, RADIO_DIO_4_PIN, IRQ_PRIORITY_RADIO, STLL_RX_SYNC_IRQHandler_CB );
}

void STLL_Radio_IoDeInit( void )
{
    LOG_DEBUG_SFXSX1276((">> STLL_Radio_IoDeInit\r\n"));

	gpio_removeIrqAction(&__sx1276_gpio_irq[0]);
	gpio_removeIrqAction(&__sx1276_gpio_irq[1]);
	gpio_removeIrqAction(&__sx1276_gpio_irq[2]);
	gpio_removeIrqAction(&__sx1276_gpio_irq[3]);
	gpio_removeIrqAction(&__sx1276_gpio_irq[4]);

//  HW_GPIO_SetIrq( RADIO_DIO_0_PORT, RADIO_DIO_0_PIN, IRQ_PRIORITY_RADIO, NULL );
//  HW_GPIO_SetIrq( RADIO_DIO_1_PORT, RADIO_DIO_1_PIN, IRQ_PRIORITY_RADIO, NULL );
//  HW_GPIO_SetIrq( RADIO_DIO_2_PORT, RADIO_DIO_2_PIN, IRQ_PRIORITY_RADIO, NULL );
//  HW_GPIO_SetIrq( RADIO_DIO_3_PORT, RADIO_DIO_3_PIN, IRQ_PRIORITY_RADIO, NULL );
//  HW_GPIO_SetIrq( RADIO_DIO_4_PORT, RADIO_DIO_4_PIN, IRQ_PRIORITY_RADIO, NULL );

  SX1276IoDeInit();
}

void STLL_Radio_SetOpMode(uint8_t opMode)
{
  SX1276SetOpMode( opMode );
}

void STLL_Radio_SetFreq( uint32_t Freq )
{
  SX1276SetChannel( Freq );
}

void STLL_RadioPowerSetEeprom( int8_t power)
{
 // HW_EEPROM_WRITE( E2pData.TxPower, power) ;
	itsdk_state.sigfox.current_power = power;
}

int8_t STLL_RadioPowerGet( void )
{
  return itsdk_state.sigfox.current_power;
  //return E2pData.TxPower;
}

void STLL_RadioPowerSetBoard( int8_t power)
{
  SX1276SetRfTxPower( power );
}

/**
 * End of Tx event seems to be executed by a external interruption
 * calling the STLL_SetEndOfTxFrame function.
 * This function is called to wait for the end of the transmission
 * This function is calling the upperlayer idle() function to eventually
 * have some action during this wait phase. The processor can goes to idle
 * mode only is the related setting is authorizing it. Basically it should be
 * a bit complicated as during this wait the DMA is tranfering orders from the memory to the SPI.
 */
STLL_flag STLL_WaitEndOfTxFrame( void )
{
  LOG_DEBUG_SFXSX1276((">> STLL_WaitEndOfTxFrame\r\n"));
  // Wait that flag EOFTX_EVT is set
  sx1276_sigfox_state.endOfTxEvent = SIGFOX_EVENT_CLEAR;
  while (sx1276_sigfox_state.endOfTxEvent == SIGFOX_EVENT_CLEAR) {
      if ( sx1276_sigfox_idle() == SX1276_SIGFOX_ERR_BREAK ) break;
      wdg_refresh();
  }
  LOG_DEBUG_SFXSX1276(("    Wait Done\r\n"));
  //SCH_WaitEvt( EOFTX_EVT );
  
  return STLL_SET;
}

/**
 * Once the Sigfox libary has decided the SPI/DMA transfer is over, this
 * function is called to stop the wait procedure
 */
void STLL_SetEndOfTxFrame( void ) {
	// Called by the DMA an interrupt
	LOG_DEBUG_SFXSX1276((">> STLL_SetEndOfTxFrame\r\n"));
	sx1276_sigfox_state.endOfTxEvent = SIGFOX_EVENT_SET;
}


/**
 * This function will we called on DMA transfer complete
 */
void STLL_onSpiDmaTxComplete(void) {
	STLL_TX_IRQHandler_CB();
}


/**
 * Switch the SPI to DMA mode, NSS is setup with the TIM2 Timer instead of being piloted by the software
 */
extern DMA_HandleTypeDef UTSDK_SX1276_SPIDMATX;
void STLL_Transmit_DMA_Start( uint16_t *pDataSource, uint16_t Size)
{
  	  LOG_DEBUG_SFXSX1276((">> STLL_Transmit_DMA_Start\r\n"));

	  // LL_SPI_SetDataWidth(SpiHandle.Instance,  LL_SPI_DATAWIDTH_16BIT );
  	  // Configure SPI as 16bits words
	  ITSDK_SX1276_SPI.Instance->CR1 &= SPI_CR1_DFF_Msk;
	  ITSDK_SX1276_SPI.Instance->CR1 |= SPI_DATASIZE_16BIT;

	  // Configure NSS to be piloted by TIM2 for DMA access
	  gpio_configure_ext(ITSDK_SX1276_NSS_BANK, ITSDK_SX1276_NSS_PIN, GPIO_ALTERNATE_PP_PULLUP,ITSDK_GPIO_SPEED_HIGH,ITSDK_GPIO_ALT_TIMER2_C1);
	  /*
	  __GPIOA_CLK_ENABLE();
	  // LL_GPIO_SetPinMode(RADIO_NSS_PORT, RADIO_NSS_PIN, LL_GPIO_MODE_ALTERNATE);
	  MODIFY_REG(bank_nss->MODER, ((ITSDK_SX1276_NSS_PIN * ITSDK_SX1276_NSS_PIN) * GPIO_MODER_MODE0), ((ITSDK_SX1276_NSS_PIN * ITSDK_SX1276_NSS_PIN) * GPIO_MODER_MODE0_1));
	  // LL_GPIO_SetPinOutputType(RADIO_NSS_PORT, RADIO_NSS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	  MODIFY_REG(bank_nss->OTYPER, ITSDK_SX1276_NSS_PIN, (ITSDK_SX1276_NSS_PIN * ((uint32_t)0x00000000U)));
	  // LL_GPIO_SetPinSpeed(RADIO_NSS_PORT, RADIO_NSS_PIN, LL_GPIO_SPEED_FREQ_HIGH);
      MODIFY_REG(bank_nss->OSPEEDR, ((ITSDK_SX1276_NSS_PIN * ITSDK_SX1276_NSS_PIN) * GPIO_OSPEEDER_OSPEED0), ((ITSDK_SX1276_NSS_PIN * ITSDK_SX1276_NSS_PIN) * GPIO_OSPEEDER_OSPEED0_1));
	  // LL_GPIO_SetPinPull(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PULLUP);
	  MODIFY_REG(bank_nss->PUPDR, ((ITSDK_SX1276_NSS_PIN * ITSDK_SX1276_NSS_PIN) * GPIO_PUPDR_PUPD0), ((ITSDK_SX1276_NSS_PIN * ITSDK_SX1276_NSS_PIN) * GPIO_PULLUP));
	  // LL_GPIO_SetAFPin_8_15(RADIO_NSS_PORT, RADIO_NSS_PIN, TIMx_GPIO_AF_CHANNEL1);
	  GPIO_TypeDef * bank_nss = getPortFromBankId(ITSDK_SX1276_NSS_BANK);
	  MODIFY_REG(bank_nss->AFR[1], (((((ITSDK_SX1276_NSS_PIN >> 8U) * (ITSDK_SX1276_NSS_PIN >> 8U)) * (ITSDK_SX1276_NSS_PIN >> 8U)) * (ITSDK_SX1276_NSS_PIN >> 8U)) * GPIO_AFRH_AFRH0),
	             (((((ITSDK_SX1276_NSS_PIN >> 8U) * (ITSDK_SX1276_NSS_PIN >> 8U)) * (ITSDK_SX1276_NSS_PIN >> 8U)) * (ITSDK_SX1276_NSS_PIN >> 8U)) * TIMx_GPIO_AF_CHANNEL1));
	  */

	  // Configure DMA
	  __HAL_RCC_DMA1_CLK_ENABLE();
	  UTSDK_SX1276_SPIDMATX.Instance = DMA1_Channel3;
	  UTSDK_SX1276_SPIDMATX.Init.Request = DMA_REQUEST_8;					// DMA_REQUEST_8   was 1
	  UTSDK_SX1276_SPIDMATX.Init.Direction = DMA_MEMORY_TO_PERIPH;
	  UTSDK_SX1276_SPIDMATX.Init.PeriphInc = DMA_PINC_DISABLE;
	  UTSDK_SX1276_SPIDMATX.Init.MemInc = DMA_MINC_ENABLE;
	  UTSDK_SX1276_SPIDMATX.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	  UTSDK_SX1276_SPIDMATX.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;   // DMA_PDATAALIGN_HALFWORD; was : DMA_MDATAALIGN_BYTE
	  UTSDK_SX1276_SPIDMATX.Init.Mode = DMA_CIRCULAR;
	  UTSDK_SX1276_SPIDMATX.Init.Priority = DMA_PRIORITY_HIGH;
	  HAL_DMA_Init(&UTSDK_SX1276_SPIDMATX);
      __HAL_LINKDMA(&ITSDK_SX1276_SPI,hdmatx,UTSDK_SX1276_SPIDMATX);

      HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	 // HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
	 // HAL_NVIC_EnableIRQ(SPI1_IRQn);

	  spi_transmit_dma_start(
			&ITSDK_SX1276_SPI,
			(uint8_t *)pDataSource,
			Size,
			STLL_onSpiDmaTxComplete
	  );
}

/**
 * Switch SPI configuration back to normal mode - 8b with NSS software based.
 */
void STLL_Transmit_DMA_Stop( void )
{
    // Called by an interrupt from the sigfox library
	LOG_DEBUG_SFXSX1276((">> STLL_Transmit_DMA_Stop\r\n"));

    spi_transmit_dma_stop(&ITSDK_SX1276_SPI);
	HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);

    // Restore the normal SPI configuration
#if ITSDK_SX1276_SPI == hspi1
	MX_SPI1_Init();
#else
	#error "Please update this part for the use of another spi handler"
#endif
	HAL_SPI_MspInit(&ITSDK_SX1276_SPI);

     //ITSDK_SX1276_SPI.Instance->CR1 &= SPI_CR1_DFF_Msk;
	 //ITSDK_SX1276_SPI.Instance->CR1 |= SPI_DATASIZE_8BIT;

	 // Restore the NSS pin configuration avec a normal GPIO
	 gpio_configure_ext(ITSDK_SX1276_NSS_BANK, ITSDK_SX1276_NSS_PIN, GPIO_OUTPUT_PULLUP,ITSDK_GPIO_SPEED_HIGH,ITSDK_GPIO_ALT_NONE);
	 gpio_set(ITSDK_SX1276_NSS_BANK,ITSDK_SX1276_NSS_PIN);

}

/* =========================================================================
 * Timer 2 is directly used by the underlaying library with no
 * details on the API and the use. Sounds like it participate to SPI/DMA
 * communications to control the sx1276 order transmission for RF signal generation
 * ---
 * During transmission the MCU have HSE (from TCXO) as a clock source. The frequency
 * is 32MHz and this HSE is directly route to SYSCLK (no PLL here)
 * =========================================================================
 */

#define  TIM2_PULSE_WIDTH  ((uint32_t) 2 )
#define  TIM2_PERIOD       ((uint32_t) 400)        // 12,5us @ 32MHz
#define  SPI_LAUNCH_DELAY  ((uint32_t) 69)

void STLL_TIM2_Init( void ) {
	LOG_DEBUG_SFXSX1276((">> STLL_TIM2_Init\r\n"));
	HAL_TIM_Base_MspInit(&ITSDK_SX1276_TIM);
	__HAL_RCC_TIM2_CLK_ENABLE();

	// -1- Timer Output Compare Configuration Structure declaration
	TIM_OC_InitTypeDef sConfig;

	//ITSDK_SX1276_TIM.Instance = TIM2;		// Set by CubeMx config

	ITSDK_SX1276_TIM.Init.Period        = TIM2_PERIOD-1;
	ITSDK_SX1276_TIM.Init.Prescaler     = 0;
	ITSDK_SX1276_TIM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	ITSDK_SX1276_TIM.Init.CounterMode   = TIM_COUNTERMODE_UP;

	HAL_TIM_OC_Init(&ITSDK_SX1276_TIM);

	// ##-2- Configure the Output Compare channels
	// Pilot NSS signal
	sConfig.OCMode     = TIM_OCMODE_PWM1;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.Pulse = TIM2_PULSE_WIDTH;

	HAL_TIM_OC_ConfigChannel(&ITSDK_SX1276_TIM, &sConfig, TIM_CHANNEL_1);

	// Pilot DMA transfer
	sConfig.OCMode     = TIM_OCMODE_ACTIVE;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.Pulse = SPI_LAUNCH_DELAY;

	HAL_TIM_OC_ConfigChannel(&ITSDK_SX1276_TIM, &sConfig, TIM_CHANNEL_2);

	// ##-3- Start signals generation
	// Start channel 2 in Output compare mode
    __HAL_TIM_ENABLE_DMA(&ITSDK_SX1276_TIM, TIM_DMA_CC2);
}


void STLL_TIM2_Start( void ) {
  LOG_DEBUG_SFXSX1276((">> STLL_TIM2_Start\r\n"));

  __HAL_TIM_SET_COUNTER(&ITSDK_SX1276_TIM, 0) ;
  HAL_TIM_OC_Start(&ITSDK_SX1276_TIM, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&ITSDK_SX1276_TIM, TIM_CHANNEL_2);
}

void STLL_TIM2_Stop( void ) {
  // Called by the DMA interrupt and from sigfox lib
  LOG_DEBUG_SFXSX1276((">> STLL_TIM2_Stop\r\n"));

  // Clear the update interrupt flag
  __HAL_TIM_CLEAR_FLAG(&ITSDK_SX1276_TIM, TIM_FLAG_UPDATE);
  // Wait for the update interrupt flag
  while(__HAL_TIM_GET_FLAG(&ITSDK_SX1276_TIM, TIM_FLAG_UPDATE) != SET);
  // stop the timer
  HAL_TIM_OC_Stop(&ITSDK_SX1276_TIM, TIM_CHANNEL_1);
  // stop the timer
  HAL_TIM_OC_Stop(&ITSDK_SX1276_TIM, TIM_CHANNEL_2);
  // reset counter value to 0 the timer
  __HAL_TIM_SET_COUNTER(&ITSDK_SX1276_TIM, 0) ;
  // clear counter flag
  __HAL_TIM_CLEAR_FLAG(&ITSDK_SX1276_TIM, TIM_FLAG_CC1);
}

void STLL_TIM2_SetPeriod( uint32_t period ) {
	LOG_DEBUG_SFXSX1276((">> STLL_TIM2_SetPeriod (%d)\r\n",period));
	STLL_TIM2_Init();

	//ITSDK_SX1276_TIM.Instance = TIM2;	// Set by CubeMx config
	ITSDK_SX1276_TIM.Init.Period        = period-1;
	ITSDK_SX1276_TIM.Init.Prescaler     = 0;
	ITSDK_SX1276_TIM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	ITSDK_SX1276_TIM.Init.CounterMode   = TIM_COUNTERMODE_UP;
    HAL_TIM_OC_Init(&ITSDK_SX1276_TIM);
}


uint32_t STLL_TIM2_GetPeriod( void ) {
	LOG_DEBUG_SFXSX1276((">> STLL_TIM2_GetPeriod\r\n"));

	return ITSDK_SX1276_TIM.Init.Period+1;
}


/**
 * Low Power Mode Authorized / prohited switch to low power mode
 */
void STLL_LowPower(STLL_State State)
{
  LOG_DEBUG_SFXSX1276((">> STLL_LowPower(%d)\r\n",State));
  if ( State == STLL_ENABLE) {
	  sx1276_sigfox_state.lowPowerAuthorised = SIGFOX_LPMODE_AUTHORIZED;
	  //LPM_SetStopMode( LPM_LIB_Id, LPM_Enable);
  } else {
	  sx1276_sigfox_state.lowPowerAuthorised = SIGFOX_LPMODE_PROHIBITED;
      // LPM_SetStopMode( LPM_LIB_Id, LPM_Disable);
  }
}

/**
 * Switch clock source...
 * During Sigfox transmission the data tansfert is managed by TIM2 and DMA control
 * The TCXO is driving the communication frequency and we need to switch from HSI to
 * external HSE connected to TCXO.
 */
#warning "This function contains non portable code"
void STLL_SetClockSource( stll_clockType_e clocktype)
{
  LOG_DEBUG_SFXSX1276((">> STLL_SetClockSource\r\n"));

  if ( clocktype == HSI_SOURCE ) {
 	 LOG_DEBUG_SFXSX1276(("   DEFAULT Selected\r\n"));
 	 SystemClock_Config();
 /*
 	 __HAL_RCC_HSI_ENABLE();
	 // Wait till HSI is ready
 	 while( __HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET ) {}
	 // Enable PLL
 	 __HAL_RCC_PLL_ENABLE();
 	 // Wait till PLL is ready
 	 while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET ) {}
	 // Select PLL as system clock source
 	 __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );
 	 // Wait till PLL is used as system clock source
 	 while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK ) {}
*/
 	 __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);

  } else {
	 LOG_DEBUG_SFXSX1276(("   HSE Selected\r\n"));
	 __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);
     // Wait till HSE is ready
     while( __HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET ) {}
     // Select HSE as system clock source
	 __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_HSE );
     // Wait till HSE is used as system clock source
	 while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_HSE ) {}
  }
}


int16_t STLL_SGFX_SX1276_GetSyncRssi(void)
{
  return sx1276_sigfox_state.meas_rssi_dbm;
}


/**
 * Apprently we are waiting end of a timer,
 * not clear why : this sound redundent with the MCU_API_timer_wait_for_end code
 */
STLL_flag STLL_WaitEndOfRxFrame( void )
{
  LOG_DEBUG_SFXSX1276((">> STLL_WaitEndOfRxFrame\r\n"));

  sx1276_sigfox_state.rxPacketReceived = STLL_RESET;
  sx1276_sigfox_state.timerEvent = SIGFOX_EVENT_CLEAR;
  while (sx1276_sigfox_state.timerEvent == SIGFOX_EVENT_CLEAR) {
	  if ( sx1276_sigfox_idle() == SX1276_SIGFOX_ERR_BREAK ) break;
  }
  //SCH_ClrEvt( TIMOUT_EVT );
  //SCH_WaitEvt( TIMOUT_EVT );
  
  return sx1276_sigfox_state.rxPacketReceived;
}


int16_t STLL_RxCarrierSenseGetRssi(void)
{
  return  (-( STLL_Radio_ReadReg( REG_RSSIVALUE ) >> 1 ) -13 + itsdk_config.sdk.sigfox.rssiCal);
}

void STLL_RxCarrierSenseInitStatus( void )
{
  /*Initialises the Flag*/
	sx1276_sigfox_state.rxCarrierSenseFlag =STLL_RESET;
}

STLL_flag STLL_RxCarrierSenseGetStatus( void )
{
  return sx1276_sigfox_state.rxCarrierSenseFlag ;
}


#endif

/******************* (C) COPYRIGHT  STMicroelectronics *****END OF FILE****/
