/* ==========================================================
 * wrappers.h - 
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
 * 
 *
 * ==========================================================
 */

#ifndef STM32L_SDK_WRAPPERS_H_
#define STM32L_SDK_WRAPPERS_H_

#include <stdbool.h>

// Serial wrappers
void serial1_flush();
void serial2_flush();
void debug_flush();

void serial1_print(char * msg);
void serial2_print(char * msg);
void debug_print(char * msg);
void logfile_print(char * msg);

void serial1_println(char * msg);
void serial2_println(char * msg);
void debug_println(char * msg);
void logfile_println(char * msg);

// watchdog
void wdg_setupWithMaxMs(uint32_t ms);
void wdg_refresh();

// eeprom
bool _eeprom_write(uint8_t bank, uint32_t offset, void * data, int len);
bool _eeprom_read(uint8_t bank, uint32_t offset, void * data, int len);

// adc
#define ADC_TEMPERATURE_ERROR		-500
int16_t adc_getTemperature();
uint16_t adc_getVdd();
uint16_t adc_getValue(uint32_t pin);

// gpio
typedef enum {
	GPIO_OUTPUT_PP = 0,
	GPIO_OUTPUT_PULLUP,
	GPIO_OUTPUT_PULLDOWN,
	GPIO_OUTPUT_OD,
	GPIO_INPUT,
	GPIO_INPUT_PULLUP,
	GPIO_INPUT_PULLDOWN,
	GPIO_INTERRUPT_RISING,
	GPIO_INTERRUPT_RISING_PULLDWN,
	GPIO_INTERRUPT_FALLING,
	GPIO_INTERRUPT_FALLING_PULLUP,
	GPIO_INTERRUPT_ANY,
	GPIO_ANALOG,
	GPIO_OFF
} itsdk_gpio_type_t;

typedef struct s_gpio_irq_chain {
	void (*irq_func)(uint16_t GPIO_Pin);
	struct s_gpio_irq_chain * next;
} gpio_irq_chain_t;

void gpio_configure(uint8_t bank, uint16_t id, itsdk_gpio_type_t type );
void gpio_set(uint8_t bank, uint16_t id);
void gpio_reset(uint8_t bank, uint16_t id);
void gpio_change(uint8_t bank, uint16_t id, uint8_t val);
void gpio_toggle(uint8_t bank, uint16_t id);
uint8_t gpio_read(uint8_t bank, uint16_t id);
void gpio_interruptEnable(uint8_t bank, uint16_t id);
void gpio_interruptDisable(uint8_t bank, uint16_t id);
void gpio_interruptPriority(uint8_t bank, uint16_t id, uint8_t nPreemption, uint8_t nSubpriority);
void gpio_interruptClear(uint8_t bank, uint16_t id);
void gpio_registerIrqAction(gpio_irq_chain_t * chain);
void gpio_removeIrqAction(gpio_irq_chain_t * chain);
bool gpio_existAction(gpio_irq_chain_t * chain);

// misc_wrapper
void itsdk_reset();
void itsdk_delayMs(uint32_t ms);

// Reset Cause
typedef enum {
	RESET_CAUSE_BOR = 0,		// under voltage			0
	RESET_CAUSE_RESET_PIN,		// hardware reset pin		1
	RESET_CAUSE_POWER_ON,		// power on					2
	RESET_CAUSE_SOFTWARE,		// software reset			3
	RESET_CAUSE_IWDG,			// Independent Watchdog		4
	RESET_CAUSE_WWDG,			// Window Watchdog			5
	RESET_CAUSE_LOWPOWER,		// Low-Power reset Flag		6

	RESET_CAUSE_UNKNONW
} itsdk_reset_cause_t;

void itsdk_cleanResetCause();
itsdk_reset_cause_t itsdk_getResetCause();


// spi
typedef enum
{
  SPI_OK       = 0x00U,
  SPI_ERROR    = 0x01U,
  SPI_BUSY     = 0x02U,
  SPI_TIMEOUT  = 0x03U
} _SPI_Status;

_SPI_Status spi_rwRegister(
		ITSDK_SPI_HANDLER_TYPE * spi,
		uint8_t	* toTransmit,
		uint8_t * toReceive,
		uint8_t   sizeToTransmit
);

_SPI_Status spi_readRegister(
		ITSDK_SPI_HANDLER_TYPE * spi,
		uint8_t	* toTransmit,
		uint8_t * toReceive,
		uint8_t   sizeToTransmit
);

_SPI_Status spi_write_byte(
		ITSDK_SPI_HANDLER_TYPE * spi,
		uint8_t Value
);

void spi_wait4TransactionEnd(
		ITSDK_SPI_HANDLER_TYPE * spi
);

void spi_reset(
		ITSDK_SPI_HANDLER_TYPE * spi
);


#endif /* STM32L_SDK_WRAPPERS_H_ */
