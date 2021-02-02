/* ==========================================================
 * sigfox_lowlevel.c - Driver for Sigfox SX1276
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 18 may 2019
 *     Author: Paul Pinault aka Disk91
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
 * ----------------------------------------------------------
 *
 * Some peaces of that code directly comes from ST Libraries
 * and identified with << COPYRIGHT(c) 2019 STMicroelectronics >>
 *
 * ==========================================================
 */

/* Includes ------------------------------------------------------------------*/
#include <it_sdk/config.h>
#if ( ITSDK_WITH_SIGFOX_LIB == __ENABLE ) && (ITSDK_SIGFOX_LIB == __SIGFOX_SX1276)

#if ITSDK_PLATFORM != __PLATFORM_STM32L0
#error "sigfox_lowlevel is not compatible with platform other than stm32"
#else
#warning "sigfox_lowlevel file contains non portable code"
#endif
#include <string.h>
#include <it_sdk/configSigfox.h>
#include <drivers/sx1276/sigfox_lowlevel.h>
#include <drivers/sx1276/sx1276.h>
#include <drivers/sx1276/sigfox_sx1276.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/eeprom/sdk_config.h>
#include <it_sdk/eeprom/sdk_state.h>
#include "stm32l0xx.h"
//#include "spi.h"


uint8_t STLL_Radio_ReadReg(uint8_t address) {
  LOG_DEBUG_SFXSX1276((">> STLL_RD %02X",address));
  return  SX1276Read( address);
}

void STLL_Radio_WriteReg( uint8_t address, uint8_t data ) {
  LOG_DEBUG_SFXSX1276((">> STLL_RW %02X,%02X\r\n",address,data));
  SX1276Write( address,  data );
}

void STLL_Radio_ReadFifo(uint8_t size, uint8_t* buffer) {
  SX1276ReadBuffer( 0, buffer, size );
}

void STLL_Radio_WriteFifo(uint8_t size, uint8_t* buffer) {
  SX1276WriteBuffer( 0, buffer, size );
}

void STLL_Radio_Init( void ) {
  LOG_DEBUG_SFXSX1276((">> STLL_Radio_Init\r\n"));
  //RadioEvents_t events = {NULL};
  //SX1276Init( &events );

  SX1276Init( NULL );

}

void STLL_Radio_DeInit( void ) {
  LOG_DEBUG_SFXSX1276((">> STLL_Radio_DeInit\r\n"));
  //RadioEvents_t events = {NULL};
  //SX1276Init( &events );

  SX1276Init( NULL );
  SX1276Write(0x40 , 0x01 );	// set DIO3 from 'buffer empty' to NA to save current
}

static void __irqHandlers_dio0( uint16_t GPIO_Pin) {
	LOG_DEBUG_SFXSX1276((">> __irqHandlers_dio0\r\n"));
	sx1276_sigfox_state.rxPacketReceived = STLL_SET;		// Set a frame have been received
	sx1276_sigfox_state.timerEvent = SIGFOX_EVENT_SET;		// Clear the running timer event
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

  gpio_interruptClear(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN);
  gpio_configure(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN, GPIO_INTERRUPT_RISING );
  gpio_interruptPriority(ITSDK_SX1276_DIO_0_BANK,ITSDK_SX1276_DIO_0_PIN,0,0);
  __sx1276_gpio_irq[0].irq_func = __irqHandlers_dio0;
  __sx1276_gpio_irq[0].pinMask = ITSDK_SX1276_DIO_0_PIN;
  gpio_registerIrqAction(&__sx1276_gpio_irq[0]);
  gpio_interruptEnable(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN);

  gpio_interruptClear(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN);
  gpio_configure(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN, GPIO_INTERRUPT_RISING );
  gpio_interruptPriority(ITSDK_SX1276_DIO_4_BANK,ITSDK_SX1276_DIO_4_PIN,0,0);
  __sx1276_gpio_irq[4].irq_func = __irqHandlers_dio4;
  __sx1276_gpio_irq[4].pinMask = ITSDK_SX1276_DIO_4_PIN;
  gpio_registerIrqAction(&__sx1276_gpio_irq[4]);
  gpio_interruptEnable(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN);

}

void STLL_Radio_IoDeInit( void )
{
  LOG_DEBUG_SFXSX1276((">> STLL_Radio_IoDeInit\r\n"));

  SX1276IoDeInit();
  gpio_removeIrqAction(&__sx1276_gpio_irq[0]);
  gpio_removeIrqAction(&__sx1276_gpio_irq[1]);
  gpio_removeIrqAction(&__sx1276_gpio_irq[2]);
  gpio_removeIrqAction(&__sx1276_gpio_irq[3]);
  gpio_removeIrqAction(&__sx1276_gpio_irq[4]);

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
  LOG_DEBUG_SFXSX1276((">> STLL_RadioPowerSetBoard(%d)\r\n",power));
  SX1276SetRfTxPower( power );
}

/**
 * End of Tx event seems to be executed by a external interruption
 * calling the STLL_SetEndOfTxFrame function.
 * This function is called to wait for the end of the transmission
 * This function is calling the upper layer idle() function to eventually
 * have some action during this wait phase. The processor can goes to idle
 * mode only is the related setting is authorizing it. Basically it should be
 * a bit complicated as during this wait the DMA is transferring orders from the memory to the SPI.
 */
STLL_flag STLL_WaitEndOfTxFrame( void )
{
  LOG_DEBUG_SFXSX1276((">> STLL_WaitEndOfTxFrame\r\n"));

  // Wait that flag EOFTX_EVT is set
  sx1276_sigfox_state.endOfTxEvent = SIGFOX_EVENT_CLEAR;
  while (   sx1276_sigfox_state.endOfTxEvent == SIGFOX_EVENT_CLEAR ) {
      if ( sx1276_sigfox_idle() == SX1276_SIGFOX_ERR_BREAK ) break;
	  #if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
        wdg_refresh();
	  #endif
  }
  LOG_DEBUG_SFXSX1276(("    Wait Done\r\n"));
  
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
 * By default SPI1 is not accessible on the chip pins but you can use alternate pin to get access
 * 	  gpio_configure_ext(__BANK_A, __LP_GPIO_0, GPIO_ALTERNATE_PP_PULLUP,ITSDK_GPIO_SPEED_HIGH,ITSDK_GPIO_ALT_TIMER2_C1);
 *	  gpio_configure_ext(__BANK_A, __LP_GPIO_5, GPIO_ALTERNATE_PP_NOPULL,ITSDK_GPIO_SPEED_HIGH,ITSDK_GPIO_ALT_SPI1_SCLK);
 *	  gpio_configure_ext(__BANK_B, __LP_GPIO_5, GPIO_ALTERNATE_PP_NOPULL,ITSDK_GPIO_SPEED_HIGH,ITSDK_GPIO_ALT_SPI1_MOSI);
 * This is making alternate on accessible pin.
 */
extern DMA_HandleTypeDef ITSDK_SX1276_SPIDMATX;
#define __DMA_HANDLER ITSDK_SX1276_SPIDMATX

extern SPI_HandleTypeDef ITSDK_SX1276_SPI;
#define __SPI_HANDLER ITSDK_SX1276_SPI

void STLL_Transmit_DMA_Start( uint16_t *pDataSource, uint16_t Size)
{
	// Buffer is stored in memory and has a size of 1600 Word = 3200 bytes
	// This is the SpiTxBuffer declared in the Stm32 library. (this is really bad, it should be allocated only
	// for the time of the transmission ... but it is static by-the-way.
	// These 1600 word are corresponding to 0.2s of Sigfox transmission at 100bps
	// So we have half/complete interrupt called on every 0.1s
	LOG_DEBUG_SFXSX1276((">> STLL_Transmit_DMA_Start\r\n"));

    // Reconfigure SPI from scratch for the DMA transfer
    // SPI speed is 16MHz (SPI_BAUDRATEPRESCALER_2) 16B
    bzero(&__SPI_HANDLER,sizeof(SPI_HandleTypeDef));
   	__SPI_HANDLER.Instance = SPI1;
  	__SPI_HANDLER.Init.Mode = SPI_MODE_MASTER;
  	__SPI_HANDLER.Init.Direction = SPI_DIRECTION_2LINES;
  	__SPI_HANDLER.Init.DataSize = SPI_DATASIZE_16BIT;
  	__SPI_HANDLER.Init.CLKPolarity = SPI_POLARITY_LOW;
  	__SPI_HANDLER.Init.CLKPhase = SPI_PHASE_1EDGE;
  	__SPI_HANDLER.Init.NSS = SPI_NSS_SOFT;
  	__SPI_HANDLER.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  	__SPI_HANDLER.Init.FirstBit = SPI_FIRSTBIT_MSB;
  	__SPI_HANDLER.Init.TIMode = SPI_TIMODE_DISABLE;
  	__SPI_HANDLER.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  	__SPI_HANDLER.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&__SPI_HANDLER);
    itsdk_delayMs(1);

    // bzero(&__DMA_HANDLER, sizeof(DMA_HandleTypeDef));
  	// Overide the DMA configuration and link it with SPI_TX
  	// Channel 3 / Request 8 is corresponding to TIM2 CC2 control
  	// 16B
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	HAL_DMA_DeInit(&__DMA_HANDLER);
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	__DMA_HANDLER.Instance = DMA1_Channel3;
	__DMA_HANDLER.Init.Request = DMA_REQUEST_8;
	__DMA_HANDLER.Init.Direction = DMA_MEMORY_TO_PERIPH;
	__DMA_HANDLER.Init.PeriphInc = DMA_PINC_DISABLE;
	__DMA_HANDLER.Init.MemInc = DMA_MINC_ENABLE;
	__DMA_HANDLER.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	__DMA_HANDLER.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	__DMA_HANDLER.Init.Mode = DMA_CIRCULAR;
	__DMA_HANDLER.Init.Priority = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&__DMA_HANDLER);
    __HAL_LINKDMA(&__SPI_HANDLER,hdmatx,__DMA_HANDLER);

    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    /**
     * Start the DMA transmission
     * The buffer contains a full DMA transfer information but to not miss any transmission
     * The HalfTransmit interrupt is used to add new data into the circular buffer. That way
     * we have no risk to loose data.
     * So here the callback used is the Half Tx one.
     */
	if ( spi_transmit_dma_start(
				&ITSDK_SX1276_SPI,
				(uint8_t *)pDataSource,
				Size,
				STLL_onSpiDmaTxComplete,		// Half Tx rised when first half of the buffer has been proceeded to replace this first half
				STLL_onSpiDmaTxComplete			// Complete Tx rised when second half of the buffer has been proceeded to replace this second half
		  ) != __SPI_OK ) {
		  LOG_ERROR_SFXSX1276(("** spi_transmit_dma_start Error \r\n"));
	}
}

/**
 * Switch SPI configuration back to normal mode - 8b with NSS software based.
 */
void STLL_Transmit_DMA_Stop( void )
{
    // Called by an interrupt from the sigfox library
	LOG_DEBUG_SFXSX1276((">> STLL_Transmit_DMA_Stop\r\n"));

	spi_transmit_dma_stop(&__SPI_HANDLER);
	HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);

    // Restore the normal SPI configuration
#if ITSDK_SX1276_SPI == hspi1
	bzero(&__SPI_HANDLER,sizeof(SPI_HandleTypeDef));
	MX_SPI1_Init();
#else
	#error "Please update this part for the use of another spi handler"
#endif
}

/* =========================================================================
 * Timer 2 is directly used by the underlaying library with no
 * details on the API and the use. Sounds like it participate to SPI/DMA
 * communications to control the sx1276 order transmission for RF signal generation
 * ---
 * During transmission the MCU have HSE (from TCXO) as a clock source. The frequency
 * is 32MHz and this HSE is directly route to SYSCLK (no PLL here)
 * ---
 * Timer is generating the NSS signal with a PWM. Period is 12.5uS with a High level
 * during the first 62,5ns of each period.
 * The DMA/SPI transfer will start, for each of the 16b blocks 2,15uS after the NSS
 * signal change each of the SPI transmission will be sent every 12,5uS => SPI transfer
 * is at 80KHz (16b every time)
 * ---
 * We assume at 600b per seconds the TIM2_PERIOD is 6 times lower and the
 * spi frequency is more constrained.
 *
 *                 <---- 2.15uS -->
 *                                 _____________________
 * TIM2_CC2 ______________________|
 *                 <-------------- 12,5uS ---------------->
 *                 > < 62,5ns
 *                  _                                       _
 * TIM2_CC1  ______| |_____________________________________| |_____________
 *
 * SPI MOSI -----------------------< Value 16b >----------------------< Val
 * SPI CLK  _______________________|||||||||||||______________________||||| 16MHz
 *
 * =========================================================================
 */

#define  TIM2_PULSE_WIDTH  ((uint32_t) 2 )		   // 62,5ns (2)
#define  TIM2_PERIOD       ((uint32_t) 400)        // 12,5us @ 32MHz => Good & verified (400)
#define  SPI_LAUNCH_DELAY  ((uint32_t) 69)		   // 2.15us (69)

static TIM_HandleTypeDef __sx1276_htim2 = {0};
#define __TIM_HANDLER __sx1276_htim2

void STLL_TIM2_Init( uint32_t period ) {
	LOG_DEBUG_SFXSX1276((">> STLL_TIM2_Init\r\n"));

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	__HAL_RCC_TIM2_CLK_ENABLE();

	// -1- Timer Output Compare Configuration Structure declaration
	switch (ITSDK_SX1276_TIM_ID) {
	  case 2:
		__TIM_HANDLER.Instance = TIM2;		// Set by CubeMx config
		break;
	  default:
	  	LOG_ERROR_SFXSX1276(("   Invalid TIM Handler\r\n"));
	  	return;
	}
	__TIM_HANDLER.Init.Prescaler     = 0;
	__TIM_HANDLER.Init.CounterMode   = TIM_COUNTERMODE_UP;
	__TIM_HANDLER.Init.Period        = period-1;
	__TIM_HANDLER.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	#ifndef ITSDK_WITHOUT_AUTORELOADPRELOAD		// Catena compatibility
	__TIM_HANDLER.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	#endif
	HAL_TIM_Base_Init(&__TIM_HANDLER);
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&__TIM_HANDLER, &sClockSourceConfig);
	HAL_TIM_PWM_Init(&__TIM_HANDLER);
	HAL_TIM_OC_Init(&__TIM_HANDLER);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&__TIM_HANDLER, &sMasterConfig);

	// ##-2- Configure the Output Compare channels
	// Pilot NSS signal
	sConfigOC.OCMode     = TIM_OCMODE_PWM1;
	sConfigOC.Pulse 	 = TIM2_PULSE_WIDTH;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&__TIM_HANDLER, &sConfigOC, TIM_CHANNEL_1);

	// Pilot DMA transfer - HIGH Level puse of 2.15uS
	sConfigOC.OCMode     = TIM_OCMODE_ACTIVE;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.Pulse      = SPI_LAUNCH_DELAY;
	HAL_TIM_OC_ConfigChannel(&__TIM_HANDLER, &sConfigOC, TIM_CHANNEL_2);

	// Configure the NSS pin as TIM2_CC1 controled output
	gpio_configure_ext(ITSDK_SX1276_NSS_BANK, ITSDK_SX1276_NSS_PIN, GPIO_ALTERNATE_PP_PULLUP,ITSDK_GPIO_SPEED_HIGH,ITSDK_GPIO_ALT_TIMER2_C1);

	// ##-3- Start signals generation
	// Start channel 2 in Output compare mode
	__HAL_TIM_ENABLE_DMA(&__TIM_HANDLER, TIM_DMA_CC2);

}


void STLL_TIM2_Start( void ) {
  // Remove logging because NSS is driven by TIM2 and in relation with SPI.
  // Logging is killing the timings.
  //LOG_DEBUG_SFXSX1276((">> STLL_TIM2_Start\r\n"));
  __HAL_TIM_SET_COUNTER(&__TIM_HANDLER, 0) ;
  HAL_TIM_Base_Start(&__TIM_HANDLER);
  HAL_TIM_OC_Start(&__TIM_HANDLER, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&__TIM_HANDLER, TIM_CHANNEL_2);
}

void STLL_TIM2_Stop( void ) {
	  // Clear the update interrupt flag
	  __HAL_TIM_CLEAR_FLAG(&__TIM_HANDLER, TIM_FLAG_UPDATE);
	  // Wait for the update interrupt flag
	  while(__HAL_TIM_GET_FLAG(&__TIM_HANDLER, TIM_FLAG_UPDATE) != SET);
	  // stop the timer
	  HAL_TIM_OC_Stop(&__TIM_HANDLER, TIM_CHANNEL_1);
	  // stop the timer
	  HAL_TIM_OC_Stop(&__TIM_HANDLER, TIM_CHANNEL_2);
	  // reset counter value to 0 the timer
	  __HAL_TIM_SET_COUNTER(&__TIM_HANDLER, 0) ;
	  // clear counter flag
	  __HAL_TIM_CLEAR_FLAG(&__TIM_HANDLER, TIM_FLAG_CC1);

	  // Restore the NSS pin configuration avec a normal GPIO
	  gpio_configure_ext(ITSDK_SX1276_NSS_BANK, ITSDK_SX1276_NSS_PIN, GPIO_OUTPUT_PULLUP,ITSDK_GPIO_SPEED_HIGH,ITSDK_GPIO_ALT_NONE);
	  gpio_set(ITSDK_SX1276_NSS_BANK,ITSDK_SX1276_NSS_PIN);
}

void STLL_TIM2_SetPeriod( uint32_t period ) {
	LOG_DEBUG_SFXSX1276((">> STLL_TIM2_SetPeriod (%d)\r\n",period));
	STLL_TIM2_Init(period);
}


uint32_t STLL_TIM2_GetPeriod( void ) {
	LOG_DEBUG_SFXSX1276((">> STLL_TIM2_GetPeriod\r\n"));
	return __TIM_HANDLER.Init.Period+1;
}


/**
 * Low Power Mode Authorized / prohited switch to low power mode
 */
void STLL_LowPower(STLL_State State)
{
  LOG_DEBUG_SFXSX1276((">> STLL_LowPower(%s)\r\n",((State==STLL_ENABLE)?"Allowed":"Prohibited")));
  if ( State == STLL_ENABLE) {
	  sx1276_sigfox_state.lowPowerAuthorised = SIGFOX_LPMODE_AUTHORIZED;
  } else {
	  sx1276_sigfox_state.lowPowerAuthorised = SIGFOX_LPMODE_PROHIBITED;
  }
}

/**
 * Switch clock source...
 * During Sigfox transmission the data tansfert is managed by TIM2 and DMA control
 * The TCXO is driving the communication frequency and we need to switch from HSI to
 * external HSE connected to TCXO.
 */

void STLL_SetClockSource( stll_clockType_e clocktype)
{
  LOG_DEBUG_SFXSX1276((">> STLL_SetClockSource: "));
  if ( clocktype == HSI_SOURCE ) {
	  LOG_DEBUG_SFXSX1276((" DEFAULT Selected\r\n"));
  } else {
	  LOG_DEBUG_SFXSX1276((" HSE Selected\r\n"));
  }

  itsdk_enterCriticalSection();
  if ( clocktype == HSI_SOURCE ) {
	  __HAL_RCC_HSI_ENABLE();
	  // Wait till HSI is ready
	  while( __HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET );
	  // Enable PLL
	  __HAL_RCC_PLL_ENABLE();
	  // Wait till PLL is ready
	  while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET ) {}
	  // Select PLL as system clock source
	  __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );
	  // Wait till PLL is used as system clock source
	  while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK ) {}
	  __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
 	 //SystemClock_Config();
 	 //__HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
  } else {
	 __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);
     // Wait till HSE is ready
     while( __HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET);
     // Select HSE as system clock source
	 __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_HSE );
	 // Wait till HSE is used as system clock source
	 while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_HSE);
  }
  itsdk_leaveCriticalSection();
}


int16_t STLL_SGFX_SX1276_GetSyncRssi(void)
{
  LOG_DEBUG_SFXSX1276((">> STLL_SGFX_SX1276_GetSyncRssi\r\n"));
  return sx1276_sigfox_state.meas_rssi_dbm;
}


/**
 * Apparently we are waiting end of a timer,
 * not clear why : this sound redundent with the MCU_API_timer_wait_for_end code
 */
STLL_flag STLL_WaitEndOfRxFrame( void )
{
  LOG_DEBUG_SFXSX1276((">> STLL_WaitEndOfRxFrame\r\n"));

  sx1276_sigfox_state.rxPacketReceived = STLL_RESET;
  sx1276_sigfox_state.timerEvent = SIGFOX_EVENT_CLEAR;
  while (sx1276_sigfox_state.timerEvent == SIGFOX_EVENT_CLEAR) {
      if ( sx1276_sigfox_idle() == SX1276_SIGFOX_ERR_BREAK ) break;
	  #if ITSDK_WITH_WDG != __WDG_NONE && ITSDK_WDG_MS > 0
         wdg_refresh();
	  #endif
  }
  return sx1276_sigfox_state.rxPacketReceived;
}


int16_t STLL_RxCarrierSenseGetRssi(void)
{
  LOG_DEBUG_SFXSX1276((">> STLL_RxCarrierSenseGetRssi\r\n"));
  return  (-( STLL_Radio_ReadReg( REG_RSSIVALUE ) >> 1 ) -13 + itsdk_config.sdk.sigfox.rssiCal);
}

void STLL_RxCarrierSenseInitStatus( void )
{
  LOG_DEBUG_SFXSX1276((">> STLL_RxCarrierSenseInitStatus\r\n"));
  sx1276_sigfox_state.rxCarrierSenseFlag =STLL_RESET;
}

STLL_flag STLL_RxCarrierSenseGetStatus( void )
{
  LOG_DEBUG_SFXSX1276((">> STLL_RxCarrierSenseGetStatus\r\n"));
  return sx1276_sigfox_state.rxCarrierSenseFlag ;
}


#endif

