
/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Header for driver hw msp module

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/**
  ******************************************************************************
  * @file    hw_msp.h
  * @author  MCD Application Team
  * @brief   Header for driver hw msp module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __HW_MSP_H__
#define __HW_MSP_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

//#define VDDA_VREFINT_CAL         ((uint32_t) 3000)
//#define BAT_CR2032               ((uint32_t) 3000)
//#define VDD_BAT                  BAT_CR2032
//#define VDD_MIN                  1800

/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/

/*!
 * \brief GPIOs Macro
 */
/*
#define RCC_GPIO_CLK_ENABLE( __GPIO_PORT__ )              \
do {                                                    \
    switch( __GPIO_PORT__)                                \
    {                                                     \
      case GPIOA_BASE: __HAL_RCC_GPIOA_CLK_ENABLE(); break;    \
      case GPIOB_BASE: __HAL_RCC_GPIOB_CLK_ENABLE(); break;    \
      case GPIOC_BASE: __HAL_RCC_GPIOC_CLK_ENABLE(); break;    \
      case GPIOD_BASE: __HAL_RCC_GPIOD_CLK_ENABLE(); break;    \
      case GPIOH_BASE: default:  __HAL_RCC_GPIOH_CLK_ENABLE(); \
    }                                                    \
  } while(0)  

#define RCC_GPIO_CLK_DISABLE( __GPIO_PORT__ )              \
do {                                                    \
    switch( __GPIO_PORT__)                                \
    {                                                     \
      case GPIOA_BASE: __HAL_RCC_GPIOA_CLK_DISABLE(); break;    \
      case GPIOB_BASE: __HAL_RCC_GPIOB_CLK_DISABLE(); break;    \
      case GPIOC_BASE: __HAL_RCC_GPIOC_CLK_DISABLE(); break;    \
      case GPIOD_BASE: __HAL_RCC_GPIOD_CLK_DISABLE(); break;    \
      case GPIOH_BASE: default:  __HAL_RCC_GPIOH_CLK_ENABLE(); \
    }                                                    \
  } while(0) 
*/
/* Exported functions ------------------------------------------------------- */ 

/*!
 * \brief Get the current temperature
 *
 * \retval value  temperature in degreeCelcius( q7.8 )
 */
uint16_t HW_GetTemperatureLevel( void );
/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level ( 0: very low, 254: fully charged )
 */
uint8_t HW_GetBatteryLevel( void );
/*!
 * \brief Initializes the boards peripherals.
 */
void HW_Init( void );

  /*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
  
void HW_DeInit( void );
/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t HW_GetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID 
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void HW_GetUniqueId( uint8_t *id );

  /*!
 * \brief Initializes the HW and enters stope mode
 */
void HW_EnterStopMode( void);

/*!
 * \brief Exits stop mode and Initializes the HW
 */
void HW_ExitStopMode( void);

/**
  * @brief Enters Low Power Sleep Mode
  * @note ARM exists the function when waking up
  * @param none
  * @retval none
  */
void HW_EnterSleepMode( void);

typedef enum
  {
    e_LOW_POWER_RTC = (1<<0),
    e_LOW_POWER_GPS = (1<<1),
    e_LOW_POWER_UART = (1<<2), /* can be used to forbid stop mode in case of uart Xfer*/
  } e_LOW_POWER_State_Id_t;

/* ADC */

/*!
 * \brief Initializes the ADC input
 *
 * \param [IN] scl  ADC input pin name to be used
 */
void HW_AdcInit(  void );

/*!
 * \brief DeInitializes the ADC 
 *
 * \param [IN] none
 */
void HW_AdcDeInit( void );

/*!
 * \brief Read the analogue voltage value
 *
 * \param [IN] Channel to read
 * \retval value    Analogue pin value
 */
uint16_t HW_AdcReadChannel( uint32_t Channel);

/*!
 * \brief Configures the sytem Clock at start-up
 *
 * \param none
 * \retval none
 */
void SystemClock_Config( void );

/**
  * @brief  Configure all GPIO's to Analog input to reduce the power consumption
  * @param  None
  * @retval None
  */
void HW_GpioInit(void);
  

#ifdef __cplusplus
}
#endif

#endif /* __HW_MSP_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
