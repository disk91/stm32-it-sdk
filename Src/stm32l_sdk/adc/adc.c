/* ==========================================================
 * adc.c - Common feature with ADC
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 18 sept. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 Disk91
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
 * To get ADC we need the HAL driver so we need to select an ADC
 * in cubeMX (like temperature) then we need to remove adc.c/h and
 * remove references in main.c
 *
 * ==========================================================
 */
#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
#include <it_sdk/itsdk.h>
#include <it_sdk/eeprom/sdk_state.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/debug.h>
#include <it_sdk/time/time.h>
#include "stm32l0xx_hal.h"

#if ( ITSDK_WITH_ADC & __ADC_ENABLED ) > 0
ADC_HandleTypeDef hadc;

// check the device datasheet for precised address
#if ITSDK_DEVICE == __DEVICE_STM32L011D4
#define CAL2_TEMP			110									// not sure if 110..
#define CAL2_VALUE          ((uint16_t*)((uint32_t)0x1FF8007E))
#define CAL1_TEMP			30
#define CAL1_VALUE          ((uint16_t*)((uint32_t)0x1FF8007A))
#define VREFINT_CAL       ((uint16_t*) ((uint32_t) 0x1FF80078))
#elif ITSDK_DEVICE == __DEVICE_STM32L031K6 || ITSDK_DEVICE == __DEVICE_STM32L053R8 || ITSDK_DEVICE == __DEVICE_STM32L072XX || ITSDK_DEVICE == __DEVICE_STM32L052T8
#define CAL2_TEMP			130 // 110 according to certain sources but 130 from Datasheet
#define CAL2_VALUE          ((uint16_t*)((uint32_t)0x1FF8007E))
#define CAL1_TEMP			30
#define CAL1_VALUE          ((uint16_t*)((uint32_t)0x1FF8007A))
#define VREFINT_CAL         ((uint16_t*) ((uint32_t) 0x1FF80078))
#else
#warning DEVICE IS NOT DEFINED FOR CALIBRATION
#define CAL2_TEMP			130
#define CAL2_VALUE          ((uint16_t*)((uint32_t)0x1FF8007E))
#define CAL1_TEMP			30
#define CAL1_VALUE          ((uint16_t*)((uint32_t)0x1FFF7A2C))
#define VREFINT_CAL       ((uint16_t*) ((uint32_t) 0x1FF80078))
#endif

#define VDD_CALIB 			((uint16_t) (3000))					// Temperature calibration VREF
#define VDD_APPLI 			((uint16_t) (ITSDK_VDD_MV))			// VDD voltage value


#if ITSDK_ADC_OPTIMIZE_SIZE == __ENABLE

/**
 * Read adc
 */
uint32_t __getAdcValue(uint32_t channel, uint8_t oversampling) {

  uint32_t data;
  uint32_t i;

  __disable_irq();

  // ADC RESET
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;		// enable ADC clock
  __NOP();
  __NOP();
  RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
  __NOP();
  __NOP();
  RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;
  __NOP();
  __NOP();


  // Enable some basic parts
  ADC1->IER = 0;						// do not allow any interrupts
  ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;		// select HSI16 clock

  ADC1->CR |= ADC_CR_ADVREGEN;			// enable ADC voltage regulator, probably not required, because this is automatically activated
  ADC->CCR |= ADC_CCR_VREFEN; 			// Wake-up the VREFINT
  ADC->CCR |= ADC_CCR_TSEN; 			// Wake-up the temperature sensor
  __NOP();
  __NOP();


  // CALIBRATION
  if ((ADC1->CR & ADC_CR_ADEN) != 0) {		// clear ADEN flag if required
    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);
  }
  ADC1->CR |= ADC_CR_ADCAL; 				// start calibration
  while ((ADC1->ISR & ADC_ISR_EOCAL) == 0);	// wait for calibration finished
  ADC1->ISR |= ADC_ISR_EOCAL; 				// clear the status flag, by writing 1 to it
  __NOP();									// not sure why, but some nop's are required here, at least 4 of them
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();


  // ENABLE ADC
  ADC1->ISR |= ADC_ISR_ADRDY; 					// clear ready flag
  ADC1->CR |= ADC_CR_ADEN; 						// enable ADC
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);		// wait for ADC


  // CONFIGURE ADC
  ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;				// software enabled conversion start
  ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;				// right alignment
  ADC1->CFGR1 &= ~ADC_CFGR1_RES;				// 12 bit resolution
  ADC1->CHSELR = channel & ADC_CHANNEL_MASK; 	// Select channel (1 << channel number)
  ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;
  	  	  	  	  	  	  	  	  	  	  	  	// Select a sampling mode of 111 (very slow)

  // DO MUTIPLE READ & AVERAGE
  data = 0;
  for( i = 0; i < oversampling ; i++ )
  {
    ADC1->CR |= ADC_CR_ADSTART; 				// start the ADC conversion
    while ((ADC1->ISR & ADC_ISR_EOC) == 0); 	// wait end of conversion
    data += ADC1->DR;							// get ADC result and clear the ISR_EOC flag
  }
  data = data / oversampling;

  // DISABLE ADC
  // at this point the end of sampling and end of sequence bits are also set in ISR registr
  if ( (ADC1->CR & ADC_CR_ADEN) != 0 )
  {
    ADC1->CR |= ADC_CR_ADDIS; 					// disable ADC... maybe better execute a reset
    while ((ADC1->CR & ADC_CR_ADEN) != 0); 		// wait for ADC disable, ADEN is also cleared
  }

  // DISABLE OTHER PARTS, INCLUDING CLOCK
  ADC->CCR &= ~ADC_CCR_VREFEN; 					// disable VREFINT
  ADC->CCR &= ~ADC_CCR_TSEN; 					// disable temperature sensor
  ADC1->CR &= ~ADC_CR_ADVREGEN;					// disable ADC voltage regulator
  RCC->APB2ENR &= ~RCC_APB2ENR_ADCEN;			// disable ADC clock

  __enable_irq();
  return data;
}

#else

uint32_t __getAdcValue(uint32_t channel, uint8_t oversampling) {
	  __HAL_RCC_ADC1_CLK_ENABLE();

	  __HAL_RCC_ADC1_FORCE_RESET();			// without reset the values were not correct
	  __NOP();
	  __NOP();
	  __HAL_RCC_ADC1_RELEASE_RESET();

	  ADC_ChannelConfTypeDef sConfig;

	  // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	  hadc.Instance = ADC1;
	  hadc.Init.OversamplingMode = DISABLE;
	  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	  hadc.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
	  hadc.Init.ScanConvMode = DISABLE;
	  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc.Init.ContinuousConvMode = DISABLE;
	  hadc.Init.DiscontinuousConvMode = DISABLE;
	  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc.Init.DMAContinuousRequests = DISABLE;
	  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	  hadc.Init.LowPowerAutoWait = DISABLE;
	  hadc.Init.LowPowerFrequencyMode = DISABLE;
	  hadc.Init.LowPowerAutoPowerOff = DISABLE;
	  if (HAL_ADC_Init(&hadc) != HAL_OK) {
		  ITSDK_ERROR_REPORT(ITSDK_ERROR_ADC_INIT_FAILED,0);
	  }

	  if ( HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK) {
		  ITSDK_ERROR_REPORT(ITSDK_ERROR_ADC_CALIBRATION_FAILED,0);
	  }

	  // Configure for the selected ADC regular channel to be converted.
	  sConfig.Channel = channel;
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		  ITSDK_ERROR_REPORT(ITSDK_ERROR_ADC_CONFCHANNEL_FAILED,0);
	  }

	  uint32_t v = 0;
	  for( int i = 0; i < oversampling ; i++ ) {
		  HAL_ADC_Start(&hadc);
		  if (HAL_ADC_PollForConversion(&hadc, 100) != HAL_OK) {
		  		  HAL_ADC_Stop(&hadc);
		  		  __HAL_RCC_ADC1_CLK_DISABLE();
		  		  return ADC_CONVERSION_ERROR;
		  }
		  v += HAL_ADC_GetValue(&hadc);
	  }
	  v = v / oversampling;

	  HAL_ADC_Stop(&hadc);
	  __HAL_RCC_ADC1_CLK_DISABLE();
	  return v;
}


#endif

/**
 * Return temperature from Adc the temp is in centi-degrés Celcius
 * Ensure to read Temperature at least 8ms after wake up ...
 * Time to get accurate getVdd response
 */
int16_t adc_getTemperature() {

	uint16_t vdd = adc_getVdd();
	uint32_t v = __getAdcValue(ADC_CHANNEL_TEMPSENSOR,ITSDK_ADC_OVERSAMPLING);

	// adapt the calibration values to the current VDD reference
	uint16_t cal1_vdd = (*CAL1_VALUE * VDD_CALIB) / vdd;
	uint16_t cal2_vdd = (*CAL2_VALUE * VDD_CALIB) / vdd;

	// convert in 0.01�C according to the calibration ref
	int32_t temperature = 100 * (CAL2_TEMP - CAL1_TEMP)*(v - cal1_vdd);
	temperature /= (cal2_vdd - cal1_vdd);
    temperature = temperature + (100*CAL1_TEMP);
    return (int16_t)temperature;

}

/**
 * Return VDD in mV ( internal VDD )
 * Be Careful -> right after wakeup from STOP the
 * value can be invalid (200mv error). The solution is to
 * sleep a bit (8ms recommanded) before sampling Vdd
 */
uint16_t adc_getVdd() {
	// The value measured is not good until we wait about 8ms after MCU wakeup from stop
	uint64_t t = ( itsdk_time_get_us() - itsdk_state.lastWakeUpTimeUs) / 1000;
	if ( t < 8 ) {
		itsdk_delayMs(8 - t);
	}
	return adc_getValue(0);
}


/**
 * Return VBAT in mV - external VDD when a VBAT pin has been configured with a voltage divider by 2
 * Assuming VBAT have a /2 in front of the ADC
 */
uint16_t adc_getVBat() {
#if ITSDK_VBAT_ADC_PIN >= 0
	return adc_getValue(ITSDK_VBAT_ADC_PIN)*2;
#else
	return adc_getVdd();
#endif
}



/**
 * Return ADC Value for an external PIN or internal
 * Get the pin number (hardware one)...
 * Pin 0 = internal VDD
 */
uint16_t adc_getValue(uint32_t pin) {

	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_TypeDef  	* GPIO_TypeDefStruct = GPIOA;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	uint32_t channel = 0;
#if ITSDK_DEVICE == __DEVICE_STM32L011D4
	switch (pin) {
	case 7:		// PA4
		GPIO_InitStruct.Pin = 4;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_4;
		break;
	case 0:
		channel = ADC_CHSELR_CHSEL17;
		break;
	default:
  	    ITSDK_ERROR_REPORT(ITSDK_ERROR_ADC_INVALID_PIN,(uint16_t)pin);
	}
#elif ITSDK_DEVICE == __DEVICE_STM32L053R8
	switch (pin) {
	case 14:		// PA0
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_0;
		break;
	case 15:		// PA1
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_1;
		break;
	case 27:		// PB1
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_TypeDefStruct = GPIOB;
		channel = ADC_CHANNEL_9;
		break;
	case 0:
		channel = ADC_CHANNEL_VREFINT;
		break;
	default:
  	    ITSDK_ERROR_REPORT(ITSDK_ERROR_ADC_INVALID_PIN,(uint16_t)pin);
	}
#elif  ITSDK_DEVICE == __DEVICE_STM32L072XX
	// For the BGA device I consider the pin number as Line||Column 65 => line 6 Column 5
	switch (pin) {
	case 0:
		channel = ADC_CHANNEL_VREFINT; 	// VDD
		break;
	case 55:
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_0;	// PA0
		break;
	case 54:
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_1;	// PA1
		break;
	case 66:
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_2;	// PA2
		break;
	case 77:
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_3;	// PA3
		break;
	case 65:
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_4; 	// PA4
		break;
	case 76:
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_5;	// PA5
		break;
	case 75:
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_6;	// PA6
		break;
	case 64:
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_7;	// PA7
		break;
	case 74:
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_TypeDefStruct = GPIOB;
		channel = ADC_CHANNEL_8;	// PB0
		break;
	case 43:
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_TypeDefStruct = GPIOB;
		channel = ADC_CHANNEL_9;	// PB1
		break;
	case 35:
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_TypeDefStruct = GPIOC;
		channel = ADC_CHANNEL_10;	// PC0
		break;
	case 34:
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_TypeDefStruct = GPIOC;
		channel = ADC_CHANNEL_11;	// PC1
		break;
	case 57:
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_TypeDefStruct = GPIOC;
		channel = ADC_CHANNEL_12;	// PC2
		break;

	default:
  	    ITSDK_ERROR_REPORT(ITSDK_ERROR_ADC_INVALID_PIN,(uint16_t)pin);
	}
#elif  ITSDK_DEVICE == __DEVICE_STM32L052T8
	// For the BGA device I consider the pin number as Line||Column 65 => line 6 Column 5
	switch (pin) {
	case 0:
		channel = ADC_CHANNEL_VREFINT; 	// VDD
		break;
	case 44:
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_0;	// PA0
		break;
	case 66:
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_1;	// PA1
		break;
	case 55:
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_2;	// PA2
		break;
	case 65:
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_3;	// PA3
		break;
	case 54:
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_4; 	// PA4
		break;
	case 64:
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_5;	// PA5
		break;
	case 53:
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_6;	// PA6
		break;
	case 63:
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_7;	// PA7
		break;
	case 43:
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_TypeDefStruct = GPIOB;
		channel = ADC_CHANNEL_8;	// PB0
		break;
	case 33:
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_TypeDefStruct = GPIOB;
		channel = ADC_CHANNEL_9;	// PB1
		break;
	default:
  	    ITSDK_ERROR_REPORT(ITSDK_ERROR_ADC_INVALID_PIN,(uint16_t)pin);
	}
#else
	#error DEVICE NOT DEFINED
#endif
	if(pin!= 0) {
		HAL_GPIO_Init(GPIO_TypeDefStruct, &GPIO_InitStruct);
	}

	uint32_t v = __getAdcValue(channel,ITSDK_ADC_OVERSAMPLING);
	if (pin == 0) {
		if ( v == 0 ) return 0; // securing
   	    int32_t vdd = ((int32_t)(*VREFINT_CAL) * VDD_CALIB) / v;
	    return (uint16_t)vdd;
	} else {
		int32_t vdd = ((uint32_t)adc_getVdd() * v )/4096;
	    return (uint16_t)vdd;
	}

}


#else
int16_t adc_getTemperature() {
	return 0;
}
#endif



#endif
