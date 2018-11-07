/* ==========================================================
 * adc.c - Common feature with ADC
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 18 sept. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018  IngeniousThings and Disk91
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
#if ITSDK_PLATFORM == __PLATFORM_STM32L0x1
#include <it_sdk/itsdk.h>
#include <it_sdk/debug.h>
#include "stm32l0xx_hal.h"

#define ADC_OPTIMIZED_CODE_FOR_SIZE	1

#if ( ITSDK_WITH_ADC & __ADC_ENABLED ) > 0
ADC_HandleTypeDef hadc;

// check the device datasheet for precised address
#if ITSDK_DEVICE == __DEVICE_STM32L011D4
#define CAL2_TEMP			110									// not sure if 110..
#define CAL2_VALUE          ((uint16_t*)((uint32_t)0x1FF8007E))
#define CAL1_TEMP			30
#define CAL1_VALUE          ((uint16_t*)((uint32_t)0x1FF8007A))
#elif ITSDK_DEVICE == __DEVICE_STM32L053R8
#define CAL2_TEMP			130
#define CAL2_VALUE          ((uint16_t*)((uint32_t)0x1FF8007E))
#define CAL1_TEMP			30
#define CAL1_VALUE          ((uint16_t*)((uint32_t)0x1FF8007A))
#else
#warning DEVICE IS NOT DEFINED FOR CALIBRATION
#define CAL2_TEMP			130
#define CAL2_VALUE          ((uint16_t*)((uint32_t)0x1FF8007E))
#define CAL1_TEMP			30
#define CAL1_VALUE          ((uint16_t*)((uint32_t)0x1FFF7A2C))
#endif

#define VDD_CALIB 			((uint16_t) (3000))
#define VDD_APPLI 			((uint16_t) (ITSDK_VDD_MV))


#ifdef ADC_OPTIMIZED_CODE_FOR_SIZE

/**
 * Read adc
 */
uint16_t readADC(uint32_t ch)
{
  uint32_t data;
  uint32_t i;

  __disable_irq();

  /* ADC RESET */

  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;	/* enable ADC clock */
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */
  RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */
  RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;
  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */


  /* Enable some basic parts */

  ADC1->IER = 0;						/* do not allow any interrupts */
  ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;	/* select HSI16 clock */

  ADC1->CR |= ADC_CR_ADVREGEN;				/* enable ADC voltage regulator, probably not required, because this is automatically activated */
  ADC->CCR |= ADC_CCR_VREFEN; 			/* Wake-up the VREFINT */
  ADC->CCR |= ADC_CCR_TSEN; 			/* Wake-up the temperature sensor */

  __NOP();								/* let us wait for some time */
  __NOP();								/* let us wait for some time */

  /* CALIBRATION */

  if ((ADC1->CR & ADC_CR_ADEN) != 0) /* clear ADEN flag if required */
  {
    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);
  }
  ADC1->CR |= ADC_CR_ADCAL; 				/* start calibration */
  while ((ADC1->ISR & ADC_ISR_EOCAL) == 0) 	/* wait for clibration finished */
  {
  }
  ADC1->ISR |= ADC_ISR_EOCAL; 			/* clear the status flag, by writing 1 to it */
  __NOP();								/* not sure why, but some nop's are required here, at least 4 of them */
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();

  /* ENABLE ADC */

  ADC1->ISR |= ADC_ISR_ADRDY; 			/* clear ready flag */
  ADC1->CR |= ADC_CR_ADEN; 			/* enable ADC */
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* wait for ADC */
  {
  }

  //printBits(5, ADC1->ISR );
  //printBits(6, ADC1->CR );

  /* CONFIGURE ADC */

  ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;	/* software enabled conversion start */
  ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;		/* right alignment */
  ADC1->CFGR1 &= ~ADC_CFGR1_RES;		/* 12 bit resolution */
  ADC1->CHSELR = ch; 					/* Select channel (1 << channel number)*/
  ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* Select a sampling mode of 111 (very slow)*/

  /* DO CONVERSION */

  data = 0;
  for( i = 0; i < 8; i++ )
  {

    ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) /* wait end of conversion */
    {
    }
    data += ADC1->DR;						/* get ADC result and clear the ISR_EOC flag */
  }
  data >>= 3;

  /* DISABLE ADC */

  /* at this point the end of sampling and end of sequence bits are also set in ISR registr */
  if ( (ADC1->CR & ADC_CR_ADEN) != 0 )
  {
    ADC1->CR |= ADC_CR_ADDIS; 			/* disable ADC... maybe better execute a reset */
    while ((ADC1->CR & ADC_CR_ADEN) != 0) 	/* wait for ADC disable, ADEN is also cleared */
    {
    }
  }

  /* DISABLE OTHER PARTS, INCLUDING CLOCK */

  ADC->CCR &= ~ADC_CCR_VREFEN; 		/* disable VREFINT */
  ADC->CCR &= ~ADC_CCR_TSEN; 			/* disable temperature sensor */
  ADC1->CR &= ~ADC_CR_ADVREGEN;		/* disable ADC voltage regulator */
  RCC->APB2ENR &= ~RCC_APB2ENR_ADCEN;	/* disable ADC clock */

  __enable_irq();
  return data;
}

#else

uint32_t __getAdcValue(uint32_t channel) {
	  ADC_ChannelConfTypeDef sConfig;

	  // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	  hadc.Instance = ADC1;
	  hadc.Init.OversamplingMode = DISABLE;
	  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	  hadc.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc.Init.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
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
	  hadc.Init.LowPowerAutoPowerOff = ENABLE;
	  if (HAL_ADC_Init(&hadc) != HAL_OK) {
		  _ERROR_HANDLER((__FILE__, __LINE__));
	  }

	  if ( HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK) {
		  _ERROR_HANDLER((__FILE__, __LINE__));
	  }

	  // Configure for the selected ADC regular channel to be converted.
	  sConfig.Channel = channel;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		  _ERROR_HANDLER((__FILE__, __LINE__));
	  }

	  HAL_ADC_Start(&hadc);
	  if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
		  uint32_t v = HAL_ADC_GetValue(&hadc);
		  HAL_ADC_Stop(&hadc);
		  return (uint32_t)v;
      }
	  HAL_ADC_Stop(&hadc);
	  return ADC_TEMPERATURE_ERROR;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

 // GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
    __HAL_RCC_ADC1_CLK_ENABLE();

/*
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
*/
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
    __HAL_RCC_ADC1_CLK_DISABLE();

/*
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
    */
  }
}

#endif

/**
 * Return temperature from Adc the temp is in centi°C
 */
int16_t adc_getTemperature() {

#ifdef ADC_OPTIMIZED_CODE_FOR_SIZE
	uint32_t v = readADC(ADC_CHSELR_CHSEL18);
#else
	uint32_t v = __getAdcValue(ADC_CHANNEL_TEMPSENSOR);
#endif

	int32_t temperature = 100 * (CAL2_TEMP - CAL1_TEMP)*(((v * VDD_CALIB) / VDD_APPLI) - *CAL1_VALUE);
	temperature /= (*CAL2_VALUE - *CAL1_VALUE);
    temperature = temperature + (100*CAL1_TEMP);
    return (int16_t)temperature;


	  /*
	  // Configure for the selected ADC regular channel to be converted.
	  sConfig.Channel = ADC_CHANNEL_VREFINT;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }
	  */
	  /*
	  sConfig.Channel = ADC_CHANNEL_4;
	   sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	   {
	     _Error_Handler(__FILE__, __LINE__);
	   }
	   */

	  /*Configure GPIO pins : PA4 PA10 */
	  /*
	  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  */
}

/**
 * Return VDD in mV
 */
uint16_t adc_getVdd() {
/*
#ifdef ADC_OPTIMIZED_CODE_FOR_SIZE
	uint32_t v = readADC(ADC_CHSELR_CHSEL17);
#else
	uint32_t v = __getAdcValue(ADC_CHANNEL_VREFINT);
#endif

	int32_t vdd = 1200 * 4096 / (((v * VDD_CALIB) / VDD_APPLI));
    return (uint16_t)vdd;
    */
	return adc_getValue(0);

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
		 _ERROR_HANDLER((__FILE__, __LINE__));
		 while(1);
	}
#elif ITSDK_DEVICE == __DEVICE_STM32L053R8
	switch (pin) {
	case 14:		// PA0
		GPIO_InitStruct.Pin = 0;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_0;
		break;
	case 15:		// PA1
		GPIO_InitStruct.Pin = 1;
		GPIO_TypeDefStruct = GPIOA;
		channel = ADC_CHANNEL_1;
		break;
	case 0:
		channel = ADC_CHSELR_CHSEL17;
		break;
	default:
		 _ERROR_HANDLER((__FILE__, __LINE__));
		 while(1);
	}
#else
	#error DEVICE NOT DEFINED
#endif
	if(pin!= 0) {
		HAL_GPIO_Init(GPIO_TypeDefStruct, &GPIO_InitStruct);
	}

#ifdef ADC_OPTIMIZED_CODE_FOR_SIZE
	uint32_t v = readADC(channel);
#else
	uint32_t v = __getAdcValue(channel);
#endif

	if (pin == 0) {
		if ( v == 0 ) v=1; // securing
		int32_t vdd = 1200 * 4096 / v; //(((v * VDD_CALIB) / VDD_APPLI));
	    return (uint16_t)vdd;
	} else {
		int32_t vdd = (adc_getVdd() * v )/4096;
	    return (uint16_t)vdd;
	}

}


#else
int16_t adc_getTemperature() {
	return 0;
}
#endif



#endif
