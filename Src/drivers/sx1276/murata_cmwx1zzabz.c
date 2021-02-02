/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/**
  *******************************************************************************
  * @file    mlm32l07x01.c
  * @author  MCD Application Team
  * @brief   driver LoRa module murata cmwx1zzabz-078
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include <it_sdk/config.h>
#if  ( ( ITSDK_WITH_LORAWAN_LIB == __ENABLE ) && (ITSDK_LORAWAN_LIB == __LORAWAN_SX1276) ) \
   ||( ( ITSDK_WITH_SIGFOX_LIB == __ENABLE ) && (ITSDK_SIGFOX_LIB == __SIGFOX_SX1276) )

/* Includes ------------------------------------------------------------------*/

#include <drivers/sx1276/hw.h>
#include <drivers/lorawan/phy/radio.h>
#include <drivers/sx1276/sx1276.h>
#include <it_sdk/eeprom/sdk_state.h>
#include <it_sdk/eeprom/sdk_config.h>

#include <it_sdk/wrappers.h>

#define IRQ_HIGH_PRIORITY  0

#define TCXO_ON() gpio_set(ITSDK_SX1276_TCXO_VCC_BANK,ITSDK_SX1276_TCXO_VCC_PIN);
#define TCXO_OFF() gpio_reset(ITSDK_SX1276_TCXO_VCC_BANK,ITSDK_SX1276_TCXO_VCC_PIN);


/*!
 * \brief Controls the antena switch if necessary.
 *
 * \remark see errata note
 *
 * \param [IN] opMode Current radio operating mode
 */
static LoRaBoardCallback_t BoardCallbacks = { SX1276SetXO,
                                              SX1276GetWakeTime,
                                              SX1276IoIrqInit,
                                              SX1276SetRfTxPower,
                                              SX1276SetAntSwLowPower,
                                              SX1276SetAntSw};

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276IoInit,
    SX1276IoDeInit,
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,		// Sleep
    SX1276SetStby,		// Standby
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276GetWakeupTime
};



/**
 * Init the SX1276 device and switch it to low power.
 * This allows to have a basic init before executing the full radio init
 */
void SX1276InitLowPower( void ) {

    LOG_INFO_SX1276((">> mSX1276InitLowPower\r\n"));

	gpio_configure(ITSDK_SX1276_TCXO_VCC_BANK, ITSDK_SX1276_TCXO_VCC_PIN, GPIO_OUTPUT_PP );
    TCXO_ON();
    itsdk_delayMs(ITSDK_MURATA_WAKEUP_TIME);
	if (ITSDK_SX1276_DIO_0_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN, GPIO_ANALOG );
	}
	if (ITSDK_SX1276_DIO_1_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_SX1276_DIO_1_BANK, ITSDK_SX1276_DIO_1_PIN, GPIO_ANALOG );
	}
	if (ITSDK_SX1276_DIO_2_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_SX1276_DIO_2_BANK, ITSDK_SX1276_DIO_2_PIN, GPIO_ANALOG );
	}
	if (ITSDK_SX1276_DIO_3_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_SX1276_DIO_3_BANK, ITSDK_SX1276_DIO_3_PIN, GPIO_ANALOG );
	}
	#ifdef RADIO_DIO_4
	  if ( (itsdk_state.activeNetwork & __ACTIV_NETWORK_SIGFOX) > 0 && ITSDK_SX1276_DIO_4_PIN != __LP_GPIO_NONE) {
		gpio_configure(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN, GPIO_ANALOG );
	  }
	#endif
	#ifdef RADIO_DIO_5
		if (ITSDK_SX1276_DIO_5_PIN != __LP_GPIO_NONE ) {
			gpio_configure(ITSDK_SX1276_DIO_5_BANK, ITSDK_SX1276_DIO_5_PIN, GPIO_ANALOG );
		}
	#endif

 	gpio_configure(ITSDK_SX1276_NSS_BANK, ITSDK_SX1276_NSS_PIN, GPIO_OUTPUT_PULLUP );
	gpio_set(ITSDK_SX1276_NSS_BANK, ITSDK_SX1276_NSS_PIN);
	SX1276Reset();
	SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_SLEEP  );	// SLEEP mode is 1uA when Standby Mode is 1,6mA
	SX1276SetAntSwLowPower(true);
	itsdk_delayMs(10);
	TCXO_OFF();
}


uint32_t SX1276GetWakeTime( void )
{
  LOG_INFO_SX1276((">> mSX1276GetWakeTime\r\n"));
  return  0;
}

void SX1276SetXO( uint8_t state )
{
  LOG_INFO_SX1276((">> mSX1276SetXO (%s)\r\n",((state==SET)?"ON":"OFF")));

  if (state == SET )
  {
    TCXO_ON(); 
    itsdk_delayMs(ITSDK_MURATA_TCXO_WARMUP);
  }
  else
  {
	itsdk_delayMs(10);
    TCXO_OFF(); 
  }
}

/**
 * Initial init of the IoS, makes non sense to configure Interrupt mode before initializing the interrupt handlers
 * this is why I set it up ANALOG at this step. it reserve power consumption.
 * Was initially set as  GPIO_INTERRUPT_RISING_PULLDWN in the STM stack and GPIO_INTERRUPT_RISING_PULLUP in the 1.4 SDK version
 * --
 * Ensure the IRQ are configured later by calling SX1276IoIrqInit or equivalent.
 * For sigfox STLL_Radio_IoInit does the job.
 */
void SX1276IoInit( void )
{
  LOG_INFO_SX1276((">> mSX1276IoInit\r\n"));

  SX1276BoardInit( &BoardCallbacks );
  if ( ITSDK_SX1276_DIO_0_PIN != __LP_GPIO_NONE ) {
     gpio_configure(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN, GPIO_ANALOG );
  }
  if ( ITSDK_SX1276_DIO_1_PIN != __LP_GPIO_NONE ) {
     gpio_configure(ITSDK_SX1276_DIO_1_BANK, ITSDK_SX1276_DIO_1_PIN, GPIO_ANALOG );
  }
  if ( ITSDK_SX1276_DIO_2_PIN != __LP_GPIO_NONE ) {
     gpio_configure(ITSDK_SX1276_DIO_2_BANK, ITSDK_SX1276_DIO_2_PIN, GPIO_ANALOG );
  }
  if ( ITSDK_SX1276_DIO_3_PIN != __LP_GPIO_NONE ) {
    gpio_configure(ITSDK_SX1276_DIO_3_BANK, ITSDK_SX1276_DIO_3_PIN, GPIO_ANALOG );
  }

#ifdef RADIO_DIO_4
  if ( (itsdk_state.activeNetwork & __ACTIV_NETWORK_SIGFOX) > 0 && ITSDK_SX1276_DIO_4_PIN != __LP_GPIO_NONE) {
    gpio_configure(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN, GPIO_ANALOG );
  }
#endif
#ifdef RADIO_DIO_5
  if ( ITSDK_SX1276_DIO_5_PIN != __LP_GPIO_NONE ) {
     gpio_configure(ITSDK_SX1276_DIO_5_BANK, ITSDK_SX1276_DIO_5_PIN, GPIO_ANALOG );
  }
#endif
  gpio_configure(ITSDK_SX1276_TCXO_VCC_BANK, ITSDK_SX1276_TCXO_VCC_PIN, GPIO_OUTPUT_PP );

}


gpio_irq_chain_t __sx1276_gpio_irq[6] = { 0 };
void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
	LOG_INFO_SX1276((">> mSX1276IoIrqInit\r\n"));


	if (ITSDK_SX1276_DIO_0_PIN != __LP_GPIO_NONE ) {
	    gpio_interruptClear(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN);
	    gpio_configure(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN, GPIO_INTERRUPT_RISING );
		gpio_interruptPriority(ITSDK_SX1276_DIO_0_BANK,ITSDK_SX1276_DIO_0_PIN,IRQ_HIGH_PRIORITY,0);
		__sx1276_gpio_irq[0].irq_func = (void (*)(uint16_t))irqHandlers[0];
		__sx1276_gpio_irq[0].pinMask = ITSDK_SX1276_DIO_0_PIN;
		gpio_registerIrqAction(&__sx1276_gpio_irq[0]);
		gpio_interruptEnable(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN);
	}

	if (ITSDK_SX1276_DIO_1_PIN != __LP_GPIO_NONE ) {
	    gpio_interruptClear(ITSDK_SX1276_DIO_1_BANK, ITSDK_SX1276_DIO_1_PIN);
	    gpio_configure(ITSDK_SX1276_DIO_1_BANK, ITSDK_SX1276_DIO_1_PIN, GPIO_INTERRUPT_RISING );
  	    gpio_interruptPriority(ITSDK_SX1276_DIO_1_BANK,ITSDK_SX1276_DIO_1_PIN,IRQ_HIGH_PRIORITY,0);
		__sx1276_gpio_irq[1].irq_func = (void (*)(uint16_t))irqHandlers[1];
		__sx1276_gpio_irq[1].pinMask = ITSDK_SX1276_DIO_1_PIN;
		gpio_registerIrqAction(&__sx1276_gpio_irq[1]);
		gpio_interruptEnable(ITSDK_SX1276_DIO_1_BANK, ITSDK_SX1276_DIO_1_PIN);
	}

	if (ITSDK_SX1276_DIO_2_PIN != __LP_GPIO_NONE ) {
	    gpio_interruptClear(ITSDK_SX1276_DIO_2_BANK, ITSDK_SX1276_DIO_2_PIN);
	    gpio_configure(ITSDK_SX1276_DIO_2_BANK, ITSDK_SX1276_DIO_2_PIN, GPIO_INTERRUPT_RISING );
		gpio_interruptPriority(ITSDK_SX1276_DIO_2_BANK,ITSDK_SX1276_DIO_2_PIN,IRQ_HIGH_PRIORITY,0);
		__sx1276_gpio_irq[2].irq_func = (void (*)(uint16_t))irqHandlers[2];
		__sx1276_gpio_irq[2].pinMask = ITSDK_SX1276_DIO_2_PIN;
		gpio_registerIrqAction(&__sx1276_gpio_irq[2]);
		gpio_interruptEnable(ITSDK_SX1276_DIO_2_BANK, ITSDK_SX1276_DIO_2_PIN);
	}

	if (ITSDK_SX1276_DIO_3_PIN != __LP_GPIO_NONE ) {
	    gpio_interruptClear(ITSDK_SX1276_DIO_3_BANK, ITSDK_SX1276_DIO_3_PIN);
	    gpio_configure(ITSDK_SX1276_DIO_3_BANK, ITSDK_SX1276_DIO_3_PIN, GPIO_INTERRUPT_RISING );
		gpio_interruptPriority(ITSDK_SX1276_DIO_3_BANK,ITSDK_SX1276_DIO_3_PIN,IRQ_HIGH_PRIORITY,0);
		__sx1276_gpio_irq[3].irq_func = (void (*)(uint16_t))irqHandlers[3];
		__sx1276_gpio_irq[3].pinMask = ITSDK_SX1276_DIO_3_PIN;
		gpio_registerIrqAction(&__sx1276_gpio_irq[3]);
		gpio_interruptEnable(ITSDK_SX1276_DIO_3_BANK, ITSDK_SX1276_DIO_3_PIN);
	}

#ifdef RADIO_DIO_4
    if ( (itsdk_state.activeNetwork & __ACTIV_NETWORK_SIGFOX) > 0 && ITSDK_SX1276_DIO_4_PIN != __LP_GPIO_NONE) {
		gpio_configure(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN, GPIO_INTERRUPT_RISING );
		// The irq handler is configured by STLL_Radio_IoInit in sigfox_lowlevel
    }
#endif
#ifdef RADIO_DIO_5
    if ( ITSDK_SX1276_DIO_5_PIN != __LP_GPIO_NONE ) {
        gpio_configure(ITSDK_SX1276_DIO_5_BANK, ITSDK_SX1276_DIO_5_PIN, GPIO_INTERRUPT_RISING );
    }
#endif

}



void SX1276IoDeInit( void )
{

  LOG_INFO_SX1276((">> mSX1276IoDeInit\r\n"));
	if (ITSDK_SX1276_DIO_0_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN, GPIO_ANALOG );
	    gpio_interruptClear(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN);
	}
	if (ITSDK_SX1276_DIO_1_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_SX1276_DIO_1_BANK, ITSDK_SX1276_DIO_1_PIN, GPIO_ANALOG );
	    gpio_interruptClear(ITSDK_SX1276_DIO_1_BANK, ITSDK_SX1276_DIO_1_PIN);
	}
	if (ITSDK_SX1276_DIO_2_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_SX1276_DIO_2_BANK, ITSDK_SX1276_DIO_2_PIN, GPIO_ANALOG );
	    gpio_interruptClear(ITSDK_SX1276_DIO_2_BANK, ITSDK_SX1276_DIO_2_PIN);
	}
	if (ITSDK_SX1276_DIO_3_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_SX1276_DIO_3_BANK, ITSDK_SX1276_DIO_3_PIN, GPIO_ANALOG );
	    gpio_interruptClear(ITSDK_SX1276_DIO_3_BANK, ITSDK_SX1276_DIO_3_PIN);
	}
  
#ifdef RADIO_DIO_4
  if ( (itsdk_state.activeNetwork & __ACTIV_NETWORK_SIGFOX) > 0 && ITSDK_SX1276_DIO_4_PIN != __LP_GPIO_NONE) {
    gpio_configure(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN, GPIO_ANALOG );
    gpio_interruptClear(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN);

  }
#endif
#ifdef RADIO_DIO_5
	if (ITSDK_SX1276_DIO_5_PIN != __LP_GPIO_NONE ) {
		gpio_configure(ITSDK_SX1276_DIO_5_BANK, ITSDK_SX1276_DIO_5_PIN, GPIO_ANALOG );
	    gpio_interruptClear(ITSDK_SX1276_DIO_5_BANK, ITSDK_SX1276_DIO_5_PIN);
	}
#endif
}


void SX1276SetRfTxPower( int8_t power )
{
	LOG_INFO_SX1276((">> mSX1276SetRfTxPower (%d)\r\n",power));

	#ifdef ITSDK_RADIO_POWER_OFFSET
	// Sigfox power modification can't be applied at low level
	// because the library seems to change sx setting or rf route on the flow
	if ( itsdk_config.sdk.activeNetwork == __ACTIV_NETWORK_LORAWAN ) {
      power += ITSDK_RADIO_POWER_OFFSET;
  	  LOG_INFO_SX1276(("   changed for (%d)\r\n",power));
	}
	#endif

    uint8_t paConfig = SX1276Read( REG_PACONFIG );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;
    uint8_t paDac = SX1276Read( REG_PADAC );

    if ( power > 14 ) {
    	paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_PABOOST;
        if( power > 17 ) {
        	// 18 .. 20 dBm
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
            if( power > 20 ) power = 20;
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        } else  {
        	// 15 .. 17 dBm
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
        SX1276Write( REG_OCP, 0x32 ); // 18 = 0x12 = 150mA max
    } else {
    	// -1 .. 14dBm
    	paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_RFO;
        if ( power < -1 ) power = -1;
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

/**
 * Switch PA Low power (true) or Full power (false)
 */
void SX1276SetAntSwLowPower( bool status )
{
	LOG_INFO_SX1276((">> mSX1276SetAntSwLowPower (%s)\r\n",((status)?"Off":"Ready")));

    if( status == false )
    {
    	// FP
    	gpio_configure(ITSDK_MURATA_ANTSW_RX_BANK, ITSDK_MURATA_ANTSW_RX_PIN, GPIO_OUTPUT_PP );
    	gpio_reset(ITSDK_MURATA_ANTSW_RX_BANK,ITSDK_MURATA_ANTSW_RX_PIN);
    	gpio_configure(ITSDK_MURATA_ANTSW_TXBOOST_BANK, ITSDK_MURATA_ANTSW_TXBOOST_PIN, GPIO_OUTPUT_PP );
    	gpio_reset(ITSDK_MURATA_ANTSW_TXBOOST_BANK,ITSDK_MURATA_ANTSW_TXBOOST_PIN);
    	gpio_configure(ITSDK_MURATA_ANTSW_TXRFO_BANK, ITSDK_MURATA_ANTSW_TXRFO_PIN, GPIO_OUTPUT_PP );
    	gpio_reset(ITSDK_MURATA_ANTSW_TXRFO_BANK,ITSDK_MURATA_ANTSW_TXRFO_PIN);
    }
    else
    {
    	// LP
    	gpio_configure(ITSDK_MURATA_ANTSW_RX_BANK, ITSDK_MURATA_ANTSW_RX_PIN, GPIO_ANALOG );
    	gpio_reset(ITSDK_MURATA_ANTSW_RX_BANK,ITSDK_MURATA_ANTSW_RX_PIN);
    	gpio_configure(ITSDK_MURATA_ANTSW_TXBOOST_BANK, ITSDK_MURATA_ANTSW_TXBOOST_PIN, GPIO_ANALOG );
    	gpio_reset(ITSDK_MURATA_ANTSW_TXBOOST_BANK,ITSDK_MURATA_ANTSW_TXBOOST_PIN);
    	gpio_configure(ITSDK_MURATA_ANTSW_TXRFO_BANK, ITSDK_MURATA_ANTSW_TXRFO_PIN, GPIO_ANALOG );
    	gpio_reset(ITSDK_MURATA_ANTSW_TXRFO_BANK,ITSDK_MURATA_ANTSW_TXRFO_PIN);
    }
}

void SX1276SetAntSw( uint8_t opMode )
{
	LOG_INFO_SX1276((">> mSX1276SetAntSw (%s)\r\n",((opMode==RFLR_OPMODE_TRANSMITTER)?"Tx":"Rx")));

    uint8_t paConfig =  SX1276Read( REG_PACONFIG );
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
      if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST ) {
    	LOG_INFO_SX1276(("   paboost route\r\n"));
    	gpio_set(ITSDK_MURATA_ANTSW_TXBOOST_BANK,ITSDK_MURATA_ANTSW_TXBOOST_PIN);
      } else {
      	LOG_INFO_SX1276(("   rfo route\r\n"));
        gpio_set(ITSDK_MURATA_ANTSW_TXRFO_BANK,ITSDK_MURATA_ANTSW_TXRFO_PIN);
      }
      SX1276.RxTx = 1;
      break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
     LOG_INFO_SX1276(("   Rx Mode\r\n"));
     SX1276.RxTx = 0;
     gpio_set(ITSDK_MURATA_ANTSW_RX_BANK,ITSDK_MURATA_ANTSW_RX_PIN);
     break;
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
	LOG_INFO_SX1276((">> mSX1276CheckRfFrequency\r\n"));

    // Implement check. Currently all frequencies are supported
    return true;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#endif
