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
#if ITSDK_WITH_LORAWAN_LIB == __ENABLE && ITSDK_LORAWAN_LIB == __LORAWAN_SX1276

/* Includes ------------------------------------------------------------------*/

#include <drivers/sx1276/hw.h>
#include <drivers/lorawan/phy/radio.h>
#include <drivers/sx1276/sx1276.h>

#include <it_sdk/wrappers.h>

#define BOARD_WAKEUP_TIME  5
#define IRQ_HIGH_PRIORITY  0

#define TCXO_ON() gpio_set(ITSDK_SX1276_TCXO_VCC_BANK,ITSDK_SX1276_TCXO_VCC_PIN);
					//HW_GPIO_Write( ITSDK_SX1276_TCXO_VCC_BANK, ITSDK_SX1276_TCXO_VCC_PIN, 1)

#define TCXO_OFF()  gpio_reset(ITSDK_SX1276_TCXO_VCC_BANK,ITSDK_SX1276_TCXO_VCC_PIN);
					//HW_GPIO_Write( ITSDK_SX1276_TCXO_VCC_BANK, ITSDK_SX1276_TCXO_VCC_PIN, 0)

static void SX1276AntSwInit( void );

static void SX1276AntSwDeInit( void );

void SX1276SetXO( uint8_t state );

uint32_t SX1276GetWakeTime( void );

void SX1276IoIrqInit( DioIrqHandler **irqHandlers );

uint8_t SX1276GetPaSelect( uint8_t power );

void SX1276SetAntSwLowPower( bool status );

void SX1276SetAntSw( uint8_t opMode );

bool SX1276CheckRfFrequency( uint32_t frequency );
void SX1276IoInit( void );
void SX1276IoDeInit( void );
void SX1276SetRfTxPower( int8_t power );

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

uint32_t SX1276GetWakeTime( void )
{
  //LOG_INFO_SX1276((">> SX1276GetWakeTime\r\n"));
  return  BOARD_WAKEUP_TIME;
}

void SX1276SetXO( uint8_t state )
{
  //LOG_INFO_SX1276((">> SX1276SetXO\r\n"));

  if (state == SET )
  {
    TCXO_ON(); 
    itsdk_delayMs(BOARD_WAKEUP_TIME);
    //DelayMs( BOARD_WAKEUP_TIME ); //start up time of TCXO
  }
  else
  {
    TCXO_OFF(); 
  }
}
void SX1276IoInit( void )
{
  LOG_INFO_SX1276((">> SX1276IoInit\r\n"));

  //GPIO_InitTypeDef initStruct={0};
  
  SX1276BoardInit( &BoardCallbacks );
  
  gpio_configure(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN, GPIO_INTERRUPT_RISING_PULLDWN );
  gpio_configure(ITSDK_SX1276_DIO_1_BANK, ITSDK_SX1276_DIO_1_PIN, GPIO_INTERRUPT_RISING_PULLDWN );
  gpio_configure(ITSDK_SX1276_DIO_2_BANK, ITSDK_SX1276_DIO_2_PIN, GPIO_INTERRUPT_RISING_PULLDWN );
  gpio_configure(ITSDK_SX1276_DIO_3_BANK, ITSDK_SX1276_DIO_3_PIN, GPIO_INTERRUPT_RISING_PULLDWN );

//  initStruct.Mode =GPIO_MODE_IT_RISING;
//  initStruct.Pull = GPIO_PULLDOWN;
//  initStruct.Speed = GPIO_SPEED_HIGH;

//  HW_GPIO_Init( RADIO_DIO_0_PORT, RADIO_DIO_0_PIN, &initStruct );
//  HW_GPIO_Init( RADIO_DIO_1_PORT, RADIO_DIO_1_PIN, &initStruct );
//  HW_GPIO_Init( RADIO_DIO_2_PORT, RADIO_DIO_2_PIN, &initStruct );
//  HW_GPIO_Init( RADIO_DIO_3_PORT, RADIO_DIO_3_PIN, &initStruct );
#ifdef RADIO_DIO_4
//  HW_GPIO_Init( RADIO_DIO_4_PORT, RADIO_DIO_4_PIN, &initStruct );
  gpio_configure(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN, GPIO_INTERRUPT_RISING_PULLDWN );
#endif
#ifdef RADIO_DIO_5
//  HW_GPIO_Init( RADIO_DIO_5_PORT, RADIO_DIO_5_PIN, &initStruct );
  gpio_configure(ITSDK_SX1276_DIO_5_BANK, ITSDK_SX1276_DIO_5_PIN, GPIO_INTERRUPT_RISING_PULLDWN );
#endif
//  initStruct.Mode =GPIO_MODE_OUTPUT_PP;
//  initStruct.Pull = GPIO_NOPULL;
//  HW_GPIO_Init( RADIO_TCXO_VCC_PORT, RADIO_TCXO_VCC_PIN, &initStruct );
  gpio_configure(ITSDK_SX1276_TCXO_VCC_BANK, ITSDK_SX1276_TCXO_VCC_PIN, GPIO_OUTPUT_PP );
}

gpio_irq_chain_t __sx1276_gpio_irq[4] = { 0 };
void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
	LOG_INFO_SX1276((">> SX1276IoIrqInit\r\n"));

    gpio_interruptPriority(ITSDK_SX1276_DIO_0_BANK,ITSDK_SX1276_DIO_0_PIN,IRQ_HIGH_PRIORITY,0);
    gpio_interruptEnable(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN);
    __sx1276_gpio_irq[0].irq_func = (void (*)(uint16_t))irqHandlers[0];
    __sx1276_gpio_irq[0].pinMask = ITSDK_SX1276_DIO_0_PIN;
    gpio_registerIrqAction(&__sx1276_gpio_irq[0]);

    gpio_interruptPriority(ITSDK_SX1276_DIO_1_BANK,ITSDK_SX1276_DIO_1_PIN,IRQ_HIGH_PRIORITY,0);
    gpio_interruptEnable(ITSDK_SX1276_DIO_1_BANK, ITSDK_SX1276_DIO_1_PIN);
    __sx1276_gpio_irq[1].irq_func = (void (*)(uint16_t))irqHandlers[1];
    __sx1276_gpio_irq[1].pinMask = ITSDK_SX1276_DIO_1_PIN;
    gpio_registerIrqAction(&__sx1276_gpio_irq[1]);

    gpio_interruptPriority(ITSDK_SX1276_DIO_2_BANK,ITSDK_SX1276_DIO_2_PIN,IRQ_HIGH_PRIORITY,0);
    gpio_interruptEnable(ITSDK_SX1276_DIO_2_BANK, ITSDK_SX1276_DIO_2_PIN);
    __sx1276_gpio_irq[2].irq_func = (void (*)(uint16_t))irqHandlers[2];
    __sx1276_gpio_irq[2].pinMask = ITSDK_SX1276_DIO_2_PIN;
    gpio_registerIrqAction(&__sx1276_gpio_irq[2]);

    gpio_interruptPriority(ITSDK_SX1276_DIO_3_BANK,ITSDK_SX1276_DIO_3_PIN,IRQ_HIGH_PRIORITY,0);
    gpio_interruptEnable(ITSDK_SX1276_DIO_3_BANK, ITSDK_SX1276_DIO_3_PIN);
    __sx1276_gpio_irq[3].irq_func = (void (*)(uint16_t))irqHandlers[3];
    __sx1276_gpio_irq[3].pinMask = ITSDK_SX1276_DIO_3_PIN;
    gpio_registerIrqAction(&__sx1276_gpio_irq[3]);

}

void SX1276IoDeInit( void )
{
  LOG_INFO_SX1276((">> SX1276IoDeInit\r\n"));

  gpio_configure(ITSDK_SX1276_DIO_0_BANK, ITSDK_SX1276_DIO_0_PIN, GPIO_INTERRUPT_RISING_PULLDWN );
  gpio_configure(ITSDK_SX1276_DIO_1_BANK, ITSDK_SX1276_DIO_1_PIN, GPIO_INTERRUPT_RISING_PULLDWN );
  gpio_configure(ITSDK_SX1276_DIO_2_BANK, ITSDK_SX1276_DIO_2_PIN, GPIO_INTERRUPT_RISING_PULLDWN );
  gpio_configure(ITSDK_SX1276_DIO_3_BANK, ITSDK_SX1276_DIO_3_PIN, GPIO_INTERRUPT_RISING_PULLDWN );


//  GPIO_InitTypeDef initStruct={0};
//
//  initStruct.Mode = GPIO_MODE_IT_RISING ; //GPIO_MODE_ANALOG;
//  initStruct.Pull = GPIO_PULLDOWN;
//
//  HW_GPIO_Init( RADIO_DIO_0_PORT, RADIO_DIO_0_PIN, &initStruct );
//  HW_GPIO_Init( RADIO_DIO_1_PORT, RADIO_DIO_1_PIN, &initStruct );
//  HW_GPIO_Init( RADIO_DIO_2_PORT, RADIO_DIO_2_PIN, &initStruct );
//  HW_GPIO_Init( RADIO_DIO_3_PORT, RADIO_DIO_3_PIN, &initStruct );
  
#ifdef RADIO_DIO_4
  HW_GPIO_Init( RADIO_DIO_4_PORT, RADIO_DIO_4_PIN, &initStruct );
  gpio_configure(ITSDK_SX1276_DIO_4_BANK, ITSDK_SX1276_DIO_4_PIN, GPIO_INTERRUPT_RISING_PULLDWN );
#endif
#ifdef RADIO_DIO_5
  HW_GPIO_Init( RADIO_DIO_5_PORT, RADIO_DIO_5_PIN, &initStruct );
  gpio_configure(ITSDK_SX1276_DIO_5_BANK, ITSDK_SX1276_DIO_5_PIN, GPIO_INTERRUPT_RISING_PULLDWN );
#endif
}

void SX1276SetRfTxPower( int8_t power )
{
	LOG_INFO_SX1276((">> SX1276SetRfTxPower (%d)\r\n",power));

    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( power );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

uint8_t SX1276GetPaSelect( uint8_t power )
{
	//LOG_INFO_SX1276((">> SX1276GetPaSelect\r\n"));

    if (power >14)
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276SetAntSwLowPower( bool status )
{
	//LOG_INFO_SX1276((">> SX1276SetAntSwLowPower\r\n"));

    if( status == false )
    {
      SX1276AntSwInit( );
    }
    else 
    {
      SX1276AntSwDeInit( );
    }
}

static void SX1276AntSwInit( void )
{
	LOG_INFO_SX1276((">> SX1276AntSwInit\r\n"));

//  GPIO_InitTypeDef initStruct={0};

  gpio_configure(ITSDK_MURATA_ANTSW_RX_BANK, ITSDK_MURATE_ANTSW_RX_PIN, GPIO_OUTPUT_PP );
  gpio_reset(ITSDK_MURATA_ANTSW_RX_BANK,ITSDK_MURATE_ANTSW_RX_PIN);
  gpio_configure(ITSDK_MURATA_ANTSW_TXBOOST_BANK, ITSDK_MURATE_ANTSW_TXBOOST_PIN, GPIO_OUTPUT_PP );
  gpio_reset(ITSDK_MURATA_ANTSW_TXBOOST_BANK,ITSDK_MURATE_ANTSW_TXBOOST_PIN);
  gpio_configure(ITSDK_MURATA_ANTSW_TXRFO_BANK, ITSDK_MURATE_ANTSW_TXRFO_PIN, GPIO_OUTPUT_PP );
  gpio_reset(ITSDK_MURATA_ANTSW_TXRFO_BANK,ITSDK_MURATE_ANTSW_TXRFO_PIN);

//
//  initStruct.Mode =GPIO_MODE_OUTPUT_PP;
//  initStruct.Pull = GPIO_NOPULL;
//  initStruct.Speed = GPIO_SPEED_HIGH;
//
//  HW_GPIO_Init( RADIO_ANT_SWITCH_PORT_RX, RADIO_ANT_SWITCH_PIN_RX, &initStruct  );
//  HW_GPIO_Write( RADIO_ANT_SWITCH_PORT_RX, RADIO_ANT_SWITCH_PIN_RX, 0);
//
//  HW_GPIO_Init( RADIO_ANT_SWITCH_PORT_TX_BOOST, RADIO_ANT_SWITCH_PIN_TX_BOOST, &initStruct  );
//  HW_GPIO_Write( RADIO_ANT_SWITCH_PORT_TX_BOOST, RADIO_ANT_SWITCH_PIN_TX_BOOST, 0);
//
//  HW_GPIO_Init( RADIO_ANT_SWITCH_PORT_TX_RFO, RADIO_ANT_SWITCH_PIN_TX_RFO, &initStruct  );
//  HW_GPIO_Write( RADIO_ANT_SWITCH_PORT_TX_RFO, RADIO_ANT_SWITCH_PIN_TX_RFO, 0);
}

static void SX1276AntSwDeInit( void )
{
	//LOG_INFO_SX1276((">> SX1276AntSwDeInit\r\n"));

  gpio_configure(ITSDK_MURATA_ANTSW_RX_BANK, ITSDK_MURATE_ANTSW_RX_PIN, GPIO_ANALOG );
  gpio_reset(ITSDK_MURATA_ANTSW_RX_BANK,ITSDK_MURATE_ANTSW_RX_PIN);
  gpio_configure(ITSDK_MURATA_ANTSW_TXBOOST_BANK, ITSDK_MURATE_ANTSW_TXBOOST_PIN, GPIO_ANALOG );
  gpio_reset(ITSDK_MURATA_ANTSW_TXBOOST_BANK,ITSDK_MURATE_ANTSW_TXBOOST_PIN);
  gpio_configure(ITSDK_MURATA_ANTSW_TXRFO_BANK, ITSDK_MURATE_ANTSW_TXRFO_PIN, GPIO_ANALOG );
  gpio_reset(ITSDK_MURATA_ANTSW_TXRFO_BANK,ITSDK_MURATE_ANTSW_TXRFO_PIN);

//  GPIO_InitTypeDef initStruct={0};
//
//  initStruct.Mode = GPIO_MODE_ANALOG ;
//
//  initStruct.Pull = GPIO_NOPULL;
//  initStruct.Speed = GPIO_SPEED_HIGH;
//
//  HW_GPIO_Init( RADIO_ANT_SWITCH_PORT_RX, RADIO_ANT_SWITCH_PIN_RX, &initStruct  );
//  HW_GPIO_Write( RADIO_ANT_SWITCH_PORT_RX, RADIO_ANT_SWITCH_PIN_RX, 0);
//
//  HW_GPIO_Init( RADIO_ANT_SWITCH_PORT_TX_BOOST, RADIO_ANT_SWITCH_PIN_TX_BOOST, &initStruct  );
//  HW_GPIO_Write( RADIO_ANT_SWITCH_PORT_TX_BOOST, RADIO_ANT_SWITCH_PIN_TX_BOOST, 0);
//
//  HW_GPIO_Init( RADIO_ANT_SWITCH_PORT_TX_RFO, RADIO_ANT_SWITCH_PIN_TX_RFO, &initStruct  );
//  HW_GPIO_Write( RADIO_ANT_SWITCH_PORT_TX_RFO, RADIO_ANT_SWITCH_PIN_TX_RFO, 0);
}

void SX1276SetAntSw( uint8_t opMode )
{
	//LOG_INFO_SX1276((">> SX1276SetAntSw (%d)\r\n",opMode));

    uint8_t paConfig =  SX1276Read( REG_PACONFIG );
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
      if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
      {
    	gpio_set(ITSDK_MURATA_ANTSW_TXBOOST_BANK,ITSDK_MURATE_ANTSW_TXBOOST_PIN);
//        HW_GPIO_Write( RADIO_ANT_SWITCH_PORT_TX_BOOST, RADIO_ANT_SWITCH_PIN_TX_BOOST, 1 );
      }
      else
      {
//        HW_GPIO_Write( RADIO_ANT_SWITCH_PORT_TX_RFO, RADIO_ANT_SWITCH_PIN_TX_RFO, 1 );
        gpio_set(ITSDK_MURATA_ANTSW_TXRFO_BANK,ITSDK_MURATE_ANTSW_TXRFO_PIN);
      }
      SX1276.RxTx = 1;
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
     SX1276.RxTx = 0;
     gpio_set(ITSDK_MURATA_ANTSW_RX_BANK,ITSDK_MURATE_ANTSW_RX_PIN);
//     HW_GPIO_Write( RADIO_ANT_SWITCH_PORT_RX, RADIO_ANT_SWITCH_PIN_RX, 1 );
     break;
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
	//LOG_INFO_SX1276((">> SX1276CheckRfFrequency\r\n"));

    // Implement check. Currently all frequencies are supported
    return true;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#endif
