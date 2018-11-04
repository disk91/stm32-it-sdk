/* ==========================================================
 * uart_wrapper.c - wrapper function for uarts
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 12 sept. 2018
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
 * Serial1 is LPUART
 * Serial2 is USART2
 * Debuh is non affected
 *
 * ==========================================================
 */
#include <string.h>
#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0x1 || ITSDK_PLATFORM == __PLATFORM_STM32L0x3

#include <it_sdk/wrappers.h>
#include "stm32l0xx_hal.h"


/**
 * Convert the bankId used as a generic Id to the right GPIO structure
 */
GPIO_TypeDef * getPortFromBankId(uint8_t bankId) {
	switch ( bankId ) {
	case __BANK_A: return GPIOA;
	case __BANK_B: return GPIOB;
	case __BANK_C: return GPIOC;
	case __BANK_D: return GPIOD;
	case __BANK_H: return GPIOH;
	default:
		_Error_Handler(__FILE__, __LINE__);
	}
	return NULL;
}


/**
 * Convert a GPIO bank/pin into the corresponding ExtI line
 */
IRQn_Type getIrqFromBankPin(uint8_t bankId, uint16_t id) {

	if ( id <= 1 ) {
		return EXTI0_1_IRQn;
	} else if ( id <= 3 ) {
		return EXTI2_3_IRQn;
	} else {
		return EXTI4_15_IRQn;
	}
}


void gpio_set(uint8_t bank, uint16_t id) {
	HAL_GPIO_WritePin(getPortFromBankId(bank), id,GPIO_PIN_SET);
}

void gpio_reset(uint8_t bank, uint16_t id) {
	HAL_GPIO_WritePin(getPortFromBankId(bank), id,GPIO_PIN_RESET);
}

void gpio_change(uint8_t bank, uint16_t id, uint8_t val) {
	HAL_GPIO_WritePin(getPortFromBankId(bank), id, val);
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

void gpio_interruptPriority(uint8_t bank, uint16_t id, uint8_t nPreemption, uint8_t nSubpriority) {
	HAL_NVIC_SetPriority(getIrqFromBankPin(bank,id), nPreemption, nSubpriority);
}



#endif
