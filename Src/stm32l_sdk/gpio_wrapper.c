/* ==========================================================
 * uart_wrapper.c - wrapper function for uarts
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 12 sept. 2018
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
 * Serial1 is LPUART
 * Serial2 is USART2
 * Debuh is non affected
 *
 * ==========================================================
 */
#include <string.h>
#include <stdbool.h>
#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0

#include <it_sdk/itsdk.h>
#include <it_sdk/wrappers.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/logger/logger.h>
#include "stm32l0xx_hal.h"


/**
 * Convert the bankId used as a generic Id to the right GPIO structure
 */
GPIO_TypeDef * getPortFromBankId(uint8_t bankId) {
	switch ( bankId ) {
	case __BANK_A: return GPIOA;
	case __BANK_B: return GPIOB;
	case __BANK_C: return GPIOC;
#if ITSDK_DEVICE == __DEVICE_STM32L072XX || ITSDK_DEVICE == __DEVICE_STM32L053R8
	case __BANK_D: return GPIOD;
#endif
#if ITSDK_DEVICE == __DEVICE_STM32L072XX
	case __BANK_E: return GPIOE;
#endif
#if ITSDK_DEVICE == __DEVICE_STM32L072XX
	case __BANK_H: return GPIOH;
#endif
	default:
		ITSDK_ERROR_REPORT(ITSDK_ERROR_GPIO_UNSUPPORTED_BANK,(uint16_t)bankId);
	}
	return NULL;
}

/**
 * Convert the pin vector (every pin is corresponding to a single bit) to a pin number.
 * Internal
 */
uint8_t getPinNumFromPinVector(uint16_t pinId) {
	uint8_t pinPos=0;
	if ( ( pinId & 0xFF00 ) != 0) { pinPos |= 0x8; }
	if ( ( pinId & 0xF0F0 ) != 0) { pinPos |= 0x4; }
	if ( ( pinId & 0xCCCC ) != 0) { pinPos |= 0x2; }
	if ( ( pinId & 0xAAAA ) != 0) { pinPos |= 0x1; }
	return pinPos;
}

/**
 * Convert a GPIO bank/pin into the corresponding ExtI line
 */
IRQn_Type getIrqFromBankPin(uint8_t bankId, uint16_t id) {

	uint8_t pinPos = getPinNumFromPinVector(id);
	if ( pinPos <= 1 ) {
		return EXTI0_1_IRQn;
	} else if ( pinPos <= 3 ) {
		return EXTI2_3_IRQn;
	} else {
		return EXTI4_15_IRQn;
	}
}


void gpio_configure(uint8_t bank, uint16_t id, itsdk_gpio_type_t type ) {
	gpio_configure_ext(bank, id, type, ITSDK_GPIO_SPEED_LOW, ITSDK_GPIO_ALT_NONE );
}


void gpio_configure_ext(uint8_t bank, uint16_t id, itsdk_gpio_type_t type, itsdk_gpio_speed_t speed, itsdk_gpio_alternate_t alternate ) {

	GPIO_InitTypeDef GPIO_InitStruct;

	switch ( bank ) {
	case __BANK_A:
		  __GPIOA_CLK_ENABLE();
		  break;
	case __BANK_B:
		  __GPIOB_CLK_ENABLE();
		  break;
	case __BANK_C:
		  __GPIOC_CLK_ENABLE();
		  break;
    #if ITSDK_DEVICE == __DEVICE_STM32L072XX || ITSDK_DEVICE == __DEVICE_STM32L053R8
	case __BANK_D:
		  __GPIOD_CLK_ENABLE();
		  break;
	#endif
	#if ITSDK_DEVICE == __DEVICE_STM32L072XX
	case __BANK_E:
		  __GPIOE_CLK_ENABLE();
		  break;
    #endif
	#if ITSDK_DEVICE == __DEVICE_STM32L072XX
	case __BANK_H:
		  __GPIOH_CLK_ENABLE();
		  break;
	#endif
	}

	GPIO_InitStruct.Pin = id;
	switch ( speed ) {
	case ITSDK_GPIO_SPEED_LOW:
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		break;
	case ITSDK_GPIO_SPEED_HIGH:
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		break;

	}

	switch (type) {

	case GPIO_OUTPUT_PP:
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		break;

	case GPIO_OUTPUT_PULLUP:
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		break;

	case GPIO_OUTPUT_PULLDOWN:
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		break;

	case GPIO_OUTPUT_OD:
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		break;

	case GPIO_INPUT:
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		break;

	case GPIO_INPUT_PULLUP:
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		break;

	case GPIO_INPUT_PULLDOWN:
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		break;

	case GPIO_INTERRUPT_RISING:
	    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
		break;

	case GPIO_INTERRUPT_RISING_PULLDWN:
	    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		break;

	case GPIO_INTERRUPT_RISING_PULLUP:
	    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
		break;

	case GPIO_INTERRUPT_FALLING:
	    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
		break;

	case GPIO_INTERRUPT_FALLING_PULLUP:
	    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
		break;

	case GPIO_INTERRUPT_FALLING_PULLDWN:
	    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		break;

	case GPIO_INTERRUPT_ANY:
	    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
		break;

	case GPIO_ANALOG:
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
		break;

	case GPIO_OFF:
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
		break;

	case GPIO_ALTERNATE_PP_NOPULL:
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    break;

	case GPIO_ALTERNATE_PP_PULLUP:
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
	    break;

	case GPIO_ALTERNATE_PP_PULLDOWN:
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	    break;

	case GPIO_ALTERNATE_OPENDRAIN:
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    break;

	}
	int err=0;
	switch (type) {
	case GPIO_ALTERNATE_PP_NOPULL:
	case GPIO_ALTERNATE_PP_PULLUP:
	case GPIO_ALTERNATE_PP_PULLDOWN:
	case GPIO_ALTERNATE_OPENDRAIN:
		switch (alternate) {
		case ITSDK_GPIO_ALT_TIMER2_TR:
		#if ITSDK_DEVICE == __DEVICE_STM32L072XX ||  ITSDK_DEVICE == __DEVICE_STM32L052T8
			if ( bank == __BANK_A && id == __LP_GPIO_15 ) GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
			else if ( bank == __BANK_A && id == __LP_GPIO_5 ) GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
			else if ( bank == __BANK_A && id == __LP_GPIO_0 ) GPIO_InitStruct.Alternate = GPIO_AF5_TIM2;
			else err=1;
		#else
		  #warning This device will not accept alternate GPIO configuration: code is missing
		#endif
			break;
		case ITSDK_GPIO_ALT_TIMER2_C1:
		#if ITSDK_DEVICE == __DEVICE_STM32L072XX ||  ITSDK_DEVICE == __DEVICE_STM32L052T8
			if ( bank == __BANK_A && id == __LP_GPIO_15 ) GPIO_InitStruct.Alternate = GPIO_AF5_TIM2;
			else if ( bank == __BANK_A && id == __LP_GPIO_5 ) GPIO_InitStruct.Alternate = GPIO_AF5_TIM2;
			else if ( bank == __BANK_A && id == __LP_GPIO_0 ) GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
			else err=1;
		#endif
			break;
		case ITSDK_GPIO_ALT_SPI1_SCLK:
			#if ITSDK_DEVICE == __DEVICE_STM32L072XX
				if ( bank == __BANK_A && id == __LP_GPIO_5 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else err=1;
			#elif ITSDK_DEVICE == __DEVICE_STM32L052T8
				if ( bank == __BANK_A && id == __LP_GPIO_5 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else if ( bank == __BANK_B && id == __LP_GPIO_3 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else err=1;
			#endif
			break;
		case ITSDK_GPIO_ALT_SPI1_MOSI:
			#if ITSDK_DEVICE == __DEVICE_STM32L072XX
				if ( bank == __BANK_B && id == __LP_GPIO_5 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else err=1;
			#elif ITSDK_DEVICE == __DEVICE_STM32L052T8
				if ( bank == __BANK_A && id == __LP_GPIO_7 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else if ( bank == __BANK_A && id == __LP_GPIO_12 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else if ( bank == __BANK_B && id == __LP_GPIO_5 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else err=1;
			#endif
			break;
		case ITSDK_GPIO_ALT_SPI1_MISO:
			#if ITSDK_DEVICE == __DEVICE_STM32L072XX
				err=1;
			#elif ITSDK_DEVICE == __DEVICE_STM32L052T8
				if ( bank == __BANK_A && id == __LP_GPIO_11 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else if( bank == __BANK_A && id == __LP_GPIO_6 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else if( bank == __BANK_B && id == __LP_GPIO_4 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else err=1;
			#endif
			break;
		case ITSDK_GPIO_ALT_SPI1_NSS:
			#if ITSDK_DEVICE == __DEVICE_STM32L072XX
				err=1;
			#elif ITSDK_DEVICE == __DEVICE_STM32L052T8
				if ( bank == __BANK_A && id == __LP_GPIO_4 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else if( bank == __BANK_A && id == __LP_GPIO_15 ) GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
				else err=1;
			#endif
			break;
		default:
		case ITSDK_GPIO_ALT_NONE:
			break;
		}
		if (err>0) {
			log_error("Gpio - invalid alternate\r\n");
		}
		break;
	default:
		break;
	}


	HAL_GPIO_Init(getPortFromBankId(bank), &GPIO_InitStruct);

}

void gpio_set(uint8_t bank, uint16_t id) {
	HAL_GPIO_WritePin(getPortFromBankId(bank), id,GPIO_PIN_SET);
}

void gpio_reset(uint8_t bank, uint16_t id) {
	HAL_GPIO_WritePin(getPortFromBankId(bank), id,GPIO_PIN_RESET);
}

void gpio_change(uint8_t bank, uint16_t id, uint8_t val) {
	HAL_GPIO_WritePin(getPortFromBankId(bank), id, ((val==__GPIO_VAL_SET)?GPIO_PIN_SET:GPIO_PIN_RESET));
}

void gpio_toggle(uint8_t bank, uint16_t id) {
	HAL_GPIO_TogglePin(getPortFromBankId(bank), id);
}

uint8_t gpio_read(uint8_t bank, uint16_t id) {
	return HAL_GPIO_ReadPin(getPortFromBankId(bank), id);
}


void gpio_interruptEnable(uint8_t bank, uint16_t id) {
	HAL_NVIC_EnableIRQ(getIrqFromBankPin(bank,id));
}

void gpio_interruptDisable(uint8_t bank, uint16_t id) {
	HAL_NVIC_DisableIRQ(getIrqFromBankPin(bank,id));
}

void gpio_interruptDisableAll() {
	  HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
	  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
}

void gpio_interruptPriority(uint8_t bank, uint16_t id, uint8_t nPreemption, uint8_t nSubpriority) {
	HAL_NVIC_SetPriority(getIrqFromBankPin(bank,id), nPreemption, nSubpriority);
}

void gpio_interruptClear(uint8_t bank, uint16_t id) {
	__HAL_GPIO_EXTI_CLEAR_IT(id);
}


/**
 * RCT Interrupt handler allowing to chain different function
 * The callback function will be fired when pinMask is matching or
 * equal to 0.
 */
gpio_irq_chain_t __gpio_irq_chain = { NULL, 0, NULL };
gpio_irq_chain_t * __gpio_irq_wakeup = NULL;
#if !defined ITSDK_WITH_GPIO_HANDLER || ITSDK_WITH_GPIO_HANDLER == __ENABLE
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
#else
void gpio_Callback(uint16_t GPIO_Pin)
#endif
{

	// When the __gpio_irq_wakeup handler is set this handler is called
	// Because we do not want the normal handler to be called until the
	// MCU is correctly configured when waking up from deep-sleep
	if (__gpio_irq_wakeup != NULL ) {
		void (*p)(uint16_t p) = __gpio_irq_wakeup->irq_func;
		if ( p != NULL ) {
			p(GPIO_Pin);
			return;
		}
	}
	// Normal non wake-up situation.
	gpio_irq_chain_t * c = &__gpio_irq_chain;
	while ( c != NULL ) {
		void (*p)(uint16_t p) = c->irq_func;
		if ( p != NULL && (c->pinMask==0 || ((c->pinMask & GPIO_Pin) > 0) ) ) {
			p(GPIO_Pin);
		}
		c = c->next;
	}
	#if !defined ITSDK_WITH_GPIO_HANDLER || ITSDK_WITH_GPIO_HANDLER == __ENABLE
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
	#endif
}

/**
 * Add an action on wake-up.
 * This action replace temporaly the exiting actions
 */
void gpio_registerWakeUpAction(gpio_irq_chain_t * chain) {
	__gpio_irq_wakeup = chain;
}

/**
 * Remove the action on wake-up.
 * This action restore the previously defined gpio actions
 */
void gpio_removeWakeUpAction() {
	__gpio_irq_wakeup = NULL;
}

/**
 * Add an action to the chain, the action **must be** static
 */
void gpio_registerIrqAction(gpio_irq_chain_t * chain) {
	gpio_irq_chain_t * c = &__gpio_irq_chain;
	while ( c->next != NULL && c->irq_func != chain->irq_func ) {
	  c = c->next;
	}
	if ( c->irq_func != chain->irq_func ) {
		// the Action is not already existing
		c->next=chain;
		chain->next = NULL;
	}
}

/**
 * Remove an action to the chain, the action **must be** static
 */
void gpio_removeIrqAction(gpio_irq_chain_t * chain) {
	gpio_irq_chain_t * c = &__gpio_irq_chain;
	while ( c != NULL && c->next != chain ) {
	  c = c->next;
	}
	if ( c != NULL ) {
		c->next = c->next->next;
	}
}

/**
 * Search for an existing action
 */
bool gpio_existAction(gpio_irq_chain_t * chain) {
	gpio_irq_chain_t * c = &__gpio_irq_chain;
	while ( c != NULL && c->next != chain ) {
	  c = c->next;
	}
	if ( c != NULL ) {
		return true;
	}
	return false;
}


#endif
