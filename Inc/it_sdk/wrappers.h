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

#include <stdbool.h>
#include <it_sdk/itsdk.h>
#include <it_sdk/config.h>

#ifndef STM32L_SDK_WRAPPERS_H_
#define STM32L_SDK_WRAPPERS_H_

// ================================================
// Serial wrappers
void serial1_init();									// Init serial regarding the global configuration - later will be more dynamic
void serial1_connect();									// Makes the serial UART configuration
void serial1_disconnect();								// Makes the serial UART de-configuration - pins back to analog
void serial1_flush();									// Terminate pending communication over serial line
void serial1_print(char * msg);							// Print over the serial communication
void serial1_println(char * msg);
void serial1_write(uint8_t * bytes,uint16_t len);		// Write binary over the serial communication

void serial2_init();
void serial2_connect();
void serial2_disconnect();
void serial2_flush();
void serial2_print(char * msg);
void serial2_println(char * msg);
void serial2_write(uint8_t * bytes,uint16_t len);


typedef enum {
	DEBUG_PRINT_DEBUG = 0,
	DEBUG_PRINT_INFO,
	DEBUG_PRINT_WARNING,
	DEBUG_PRINT_ERROR,
	DEBUG_PRINT_ANY
} debug_print_type_e;
void debug_flush();
void debug_print(debug_print_type_e lvl, char * msg);
void logfile_print(char * msg);
void logfile_println(char * msg);

typedef enum {
	SERIAL_READ_SUCCESS=0,		// 1 char has been read, none is pending
	SERIAL_READ_PENDING_CHAR,	// 1 char has been read, some other are pending
	SERIAL_READ_NOCHAR,			// No char read, No pending
	SERIAL_READ_FAILED			// Error during reading
} serial_read_response_e;
serial_read_response_e serial1_read(char * c);
serial_read_response_e serial2_read(char * c);

typedef enum {
	SERIAL_SPEED_1200 = 0,
	SERIAL_SPEED_2400,
	SERIAL_SPEED_4800,
	SERIAL_SPEED_9600,
	SERIAL_SPEED_19200,
	SERIAL_SPEED_38400,
	SERIAL_SPEED_57600,
	SERIAL_SPEED_115200
} serial_baudrate_e;

itsdk_bool_e serial1_changeBaudRate(serial_baudrate_e bd);
itsdk_bool_e serial2_changeBaudRate(serial_baudrate_e bd);



// ================================================
// watchdog
void wdg_setupWithMaxMs(uint32_t ms);
void wdg_refresh();

// ================================================
// eeprom
bool _eeprom_write(uint8_t bank, uint32_t offset, void * data, int len);
bool _eeprom_read(uint8_t bank, uint32_t offset, void * data, int len);

// ================================================
// adc
#define ADC_TEMPERATURE_ERROR		-500
#define ADC_CONVERSION_ERROR	0xFFFFFFFF
int16_t adc_getTemperature();
uint16_t adc_getVdd();
uint16_t adc_getValue(uint32_t pin);
uint16_t adc_getVBat();

// ================================================
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
	GPIO_INTERRUPT_RISING_PULLUP,
	GPIO_INTERRUPT_FALLING,
	GPIO_INTERRUPT_FALLING_PULLUP,
	GPIO_INTERRUPT_FALLING_PULLDWN,
	GPIO_INTERRUPT_ANY,
	GPIO_ANALOG,
	GPIO_OFF,
	GPIO_ALTERNATE_PP_NOPULL,
	GPIO_ALTERNATE_PP_PULLUP,
	GPIO_ALTERNATE_PP_PULLDOWN,
	GPIO_ALTERNATE_OPENDRAIN
} itsdk_gpio_type_t;

typedef enum {
	ITSDK_GPIO_SPEED_LOW = 0,
	ITSDK_GPIO_SPEED_HIGH
} itsdk_gpio_speed_t;

typedef enum {
	ITSDK_GPIO_ALT_NONE = 0,
	ITSDK_GPIO_ALT_TIMER2_TR,
	ITSDK_GPIO_ALT_TIMER2_C1,
	ITSDK_GPIO_ALT_SPI1_SCLK,
	ITSDK_GPIO_ALT_SPI1_MOSI,
	ITSDK_GPIO_ALT_SPI1_MISO,
	ITSDK_GPIO_ALT_SPI1_NSS

} itsdk_gpio_alternate_t;

typedef struct s_gpio_irq_chain {
	void (*irq_func)(uint16_t GPIO_Pin);
	uint16_t pinMask;
	struct s_gpio_irq_chain * next;
} gpio_irq_chain_t;

void gpio_configure(uint8_t bank, uint16_t id, itsdk_gpio_type_t type );
void gpio_configure_ext(uint8_t bank, uint16_t id, itsdk_gpio_type_t type, itsdk_gpio_speed_t speed, itsdk_gpio_alternate_t alternate );
void gpio_set(uint8_t bank, uint16_t id);
void gpio_reset(uint8_t bank, uint16_t id);
void gpio_change(uint8_t bank, uint16_t id, uint8_t val);
void gpio_toggle(uint8_t bank, uint16_t id);
uint8_t gpio_read(uint8_t bank, uint16_t id);
void gpio_interruptEnable(uint8_t bank, uint16_t id);
void gpio_interruptDisable(uint8_t bank, uint16_t id);
void gpio_interruptDisableAll( void );
void gpio_interruptPriority(uint8_t bank, uint16_t id, uint8_t nPreemption, uint8_t nSubpriority);
void gpio_interruptClear(uint8_t bank, uint16_t id);
void gpio_registerIrqAction(gpio_irq_chain_t * chain);
void gpio_removeIrqAction(gpio_irq_chain_t * chain);
bool gpio_existAction(gpio_irq_chain_t * chain);
void gpio_registerWakeUpAction(gpio_irq_chain_t * chain);
void gpio_removeWakeUpAction();

#if ITSDK_WITH_GPIO_HANDLER == __DISABLE
void gpio_Callback(uint16_t GPIO_Pin);
#endif

// ================================================
// spi
#if ITSDK_WITH_SPI == __SPI_ENABLED
typedef enum
{
  __SPI_OK       = 0x00U,
  __SPI_ERROR    = 0x01U,
  __SPI_BUSY     = 0x02U,
  __SPI_TIMEOUT  = 0x03U
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

_SPI_Status spi_transmit_dma_start(
		SPI_HandleTypeDef * spi,
		uint8_t * 			pData,
		uint16_t  			size,
		void (* pCallbackHC)( void ),		// Half Transfer Complete callback
		void (* pCallbackTC)( void )		// TransferComplete callback
);

_SPI_Status spi_transmit_dma_stop(
		ITSDK_SPI_HANDLER_TYPE * spi
);

void spi_wait4TransactionEnd(
		ITSDK_SPI_HANDLER_TYPE * spi
);

void spi_reset(
		ITSDK_SPI_HANDLER_TYPE * spi
);
#endif
// ================================================
// I2C
#if ITSDK_WITH_I2C == __I2C_ENABLED
typedef enum
{
  __I2C_OK       = 0x00U,
  __I2C_ERROR    = 0x01U,
  __I2C_BUSY     = 0x02U,
  __I2C_TIMEOUT  = 0x03U
} _I2C_Status;

_I2C_Status i2c_memWrite(
		ITSDK_I2C_HANDLER_TYPE * i2c,				// i2c handler
		uint16_t  devAdr,							// Device Address => 7 bits non shifted
		uint16_t  memAdr,							// Memory address to access
		uint16_t  memAdrSize,						// 8 for 8b, 16 for 16 bits ...
		uint8_t * values,							// Data to be written
		uint16_t  size								// Size of the data to be written
);

_I2C_Status i2c_write(
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,
		uint8_t * values,
		uint16_t  size
);

_I2C_Status i2c_write8BRegister(
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,			// Non shifted device address
		uint16_t  regAdr,			// Register address (8b or 16b�
		uint8_t   value,			// 8B value to be written
		uint16_t  regSize			// Register address size 1B or 2B
);

_I2C_Status i2c_write16BRegister(		// 16B Word => LSB first on I2C
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,			// Non shifted device address
		uint16_t  regAdr,			// Register address (8b or 16b�
		uint16_t  value,			// 16B value to be written
		uint16_t  regSize			// Register address size 1B or 2B
);

_I2C_Status i2c_memRead(
		ITSDK_I2C_HANDLER_TYPE * i2c,				// i2c handler
		uint16_t  devAdr,							// Device Address => 7 bits non shifted
		uint16_t  memAdr,							// Memory address to access
		uint16_t  memAdrSize,						// 8 for 8b, 16 for 16 bits ...
		uint8_t * values,							// Data to be written
		uint16_t  size								// Size of the data to be written
);

_I2C_Status i2c_read(
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,
		uint8_t * values,
		uint16_t  size
);

_I2C_Status i2c_read8BRegister(
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,			// Non shifted device address
		uint16_t  regAdr,			// Register address (8b or 16b�
		uint8_t * value,			// 8B value to be read
		uint16_t  regSize			// Register address size 1B or 2B
);

_I2C_Status i2c_read16BRegister(    // 16B Word => LSB first on I2C
		ITSDK_I2C_HANDLER_TYPE * i2c,
		uint16_t  devAdr,			// Non shifted device address
		uint16_t  regAdr,			// Register address (8b or 16b�
		uint16_t * value,			// 8B value to be read
		uint16_t  regSize			// Register address size 1B or 2B
);

void i2c_reset(
		ITSDK_I2C_HANDLER_TYPE * i2c
);
#endif

// ================================================
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

// ================================================
// misc_wrapper
void itsdk_reset();
void itsdk_delayMs(uint32_t ms);

uint32_t itsdk_getIrqMask();
void itsdk_setIrqMask(uint32_t mask);
void itsdk_enterCriticalSection();
void itsdk_leaveCriticalSection();
void itsdk_disableIrq();
void itsdk_enableIrq();

uint32_t itsdk_getRandomSeed();								// get a random seed value - can be the same for one given object
uint8_t itsdk_randomBit();									// get a pseudo or random bit value
void itsdk_getUniqId(uint8_t * id, int8_t size);			// fill id table with an object ID having the given size


#endif /* STM32L_SDK_WRAPPERS_H_ */
