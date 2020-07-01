/* ==========================================================
 * misc_wrapper.c - 
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 15 sept. 2018
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
 * Wrapper for different usage
 *
 * ==========================================================
 */

#include <it_sdk/config.h>
#if ITSDK_PLATFORM == __PLATFORM_STM32L0

#include <it_sdk/wrappers.h>
#include "stm32l0xx_hal.h"

/**
 * Reset the device
 */
void itsdk_reset() {
	while(1) NVIC_SystemReset();
}

/**
 * Reset Cause
 */
itsdk_reset_cause_t itsdk_getResetCause() {
	if ( RCC->CSR & RCC_CSR_LPWRRSTF ) return RESET_CAUSE_LOWPOWER;
	if ( RCC->CSR & RCC_CSR_WWDGRSTF ) return RESET_CAUSE_WWDG;
	if ( RCC->CSR & RCC_CSR_IWDGRSTF ) return RESET_CAUSE_IWDG;
	if ( RCC->CSR & RCC_CSR_SFTRSTF ) return RESET_CAUSE_SOFTWARE;
	if ( RCC->CSR & RCC_CSR_PORRSTF ) return RESET_CAUSE_POWER_ON;
	if ( RCC->CSR & RCC_CSR_PINRSTF ) return RESET_CAUSE_RESET_PIN;
	if ( RCC->CSR & RCC_CSR_OBLRSTF ) return RESET_CAUSE_LOWPOWER;
	else return RESET_CAUSE_UNKNONW;
}

void itsdk_cleanResetCause() {
	RCC->CSR |= RCC_CSR_RMVF;
}

/**
 * Delay in ms
 */
void itsdk_delayMs(uint32_t ms) {
	HAL_Delay(ms);
}

/**
 * Get the IRQ Mask
 */
uint32_t itsdk_getIrqMask() {
	return __get_PRIMASK();
}

/**
 * Set / Restore the IRQ Mask
 */
void itsdk_setIrqMask(uint32_t mask) {
	__set_PRIMASK(mask);
}
/**
 * Enter a critical section / disable interrupt
 */
static uint32_t __interrupt_mask;
void itsdk_enterCriticalSection() {
	__interrupt_mask = itsdk_getIrqMask();
	//__disable_irq();
	__set_PRIMASK(1);	// allows to capture but not execute the interruption appearing during the critical section execution
}

/**
 * Restore the initial irq mask
 * to leave a critical secqtion
 */
void itsdk_leaveCriticalSection() {
	itsdk_setIrqMask(__interrupt_mask);
}

/**
 * Disable IRQ
 */
void itsdk_disableIrq() {
	__disable_irq();
}

/**
 * Enable IRQ
 */
void itsdk_enableIrq() {
	__enable_irq();
}


/**
 * Generate a seed. This seed is different for any of the objects
 *
 */
#if ITSDK_PLATFORM == __PLATFORM_STM32L0
	#define  STM32_ID1    ( 0x1FF80050 )
	#define  STM32_ID2    ( 0x1FF80054 )
	#define  STM32_ID3    ( 0x1FF80064 )
#else
    #error "You need to define the MCU ID for this platform"
#endif
uint32_t itsdk_getRandomSeed() {
	return ( ( *( uint32_t* )STM32_ID1 ) ^ ( *( uint32_t* )STM32_ID2 ) ^ ( *( uint32_t* )STM32_ID3 ) );
}

/**
 * Generate a uniq ID based on the object ID. The id struct is
 * initialized based on this. This size of the id table is given
 * as a parameter. size is in Byte
 */
void itsdk_getUniqId(uint8_t * id, int8_t size){

	uint32_t i = (( *( uint32_t* )STM32_ID1 ) << 16) + (( *( uint32_t* )STM32_ID2 )  << 8) + (*( uint32_t* )STM32_ID3 );
	uint8_t l=0;
	uint32_t s=i;
	while ( l < size ) {
		if ( (l & 0x3) == 0 ) {
			switch ( (l >> 2) & 3 ) {
				case 0:	s = i ^ STM32_ID1; break;
				case 1: s = i ^ STM32_ID2; break;
				case 2: s = i ^ STM32_ID3; break;
				default:
				case 3: s = i; break;
			}
		}
		id[l] = ( s >> (8*(l&3))) & 0xFF;
		l++;
	}

}

/**
 * generate a single random bit.
 */
#if ( ITSDK_WITH_ADC & __ADC_ENABLED ) > 0
uint32_t __getAdcValue(uint32_t channel,uint8_t oversampling);
uint8_t itsdk_randomBit(){
	uint32_t t = __getAdcValue(ADC_CHANNEL_TEMPSENSOR,1);
	uint32_t v = __getAdcValue(ADC_CHANNEL_VREFINT,1);
	return (t ^ v) & 1 ;
}
#else
static uint32_t __r = 0;
uint8_t itsdk_randomBit(){
	__r+=itsdk_getRandomSeed();
	__r+=281624173;
	__r*=415727;
	return  (( __r & 0x00010000 )>0)?1:0;
}
#endif
#endif
