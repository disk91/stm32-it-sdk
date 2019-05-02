/* ==========================================================
 * st_lib_api.c - S2LP (STm SubGhz transceiver) API interface
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 04 nov. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 and Disk91
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
 * Some peaces of that code directly comes from ST Libraries
 * and identified with << COPYRIGHT(c) 2018 STMicroelectronics >>
 *
 * ==========================================================
 */

/* Includes ------------------------------------------------------------------*/
#include <it_sdk/config.h>
#if ITSDK_WITH_SIGFOX_LIB > 0 && ITSDK_SIGFOX_LIB == __SIGFOX_S2LP

#include <it_sdk/itsdk.h>
#include <it_sdk/logger/logger.h>
#include <it_sdk/logger/error.h>
#include <it_sdk/time/timer.h>
#include <it_sdk/lowpower/lowpower.h>
#include <stm32l_sdk/rtc/rtc.h>
#include <drivers/sigfox/sigfox_types.h>
#include <drivers/sigfox/sigfox_api.h>
#include <drivers/s2lp/st_lib_api.h>
#include <drivers/s2lp/st_mcu_api.h>
#include <drivers/s2lp/st_rf_api.h>
#include <drivers/s2lp/s2lp.h>
#include <drivers/s2lp/s2lp_spi.h>
#include <drivers/s2lp/sigfox_retriever.h>
#include <drivers/s2lp/sigfox_helper.h>
#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640
	#include <drivers/eeprom/m95640/m95640.h>
#endif
#include <it_sdk/wrappers.h>
#include <string.h>

#include <it_sdk/encrypt/tiny-AES-c/aes.h>




// Reset the system clock to default
void ST_MCU_API_SetSysClock(void) {
	LOG_DEBUG_S2LP((">> ST_MCU_API_SetSysClock\r\n"));
	SystemClock_Config();
}



/*
 *  Delay in Ms
 */
static void priv_ST_MCU_API_delay(uint32_t delay_ms) {
	LOG_DEBUG_S2LP((">> priv_ST_MCU_API_delay\r\n"));
	itsdk_delayMs(delay_ms);
}

/**
 * Static memory allocation
 */
sfx_u8 MCU_API_malloc(sfx_u16 size, sfx_u8 **returned_pointer)
{
  static sfx_u8 mem[ITSDK_SIGFOX_MEM_SIZE];
  
  LOG_DEBUG_S2LP(("Sigfox lib mem req: %dB\r\n",size));
  if(size>ITSDK_SIGFOX_MEM_SIZE) {
	  LOG_ERROR_S2LP(("Requesting more memory than maximum allowable\r\n"));
	  return MCU_ERR_API_MALLOC;
  } else {
    (*returned_pointer)=mem;
  }
  return SFX_ERR_NONE;
}

/**
 * Fake memory deallocation
 */
sfx_u8 MCU_API_free(sfx_u8 *ptr)
{
  LOG_DEBUG_S2LP((">> MCU_API_free\r\n"));
  return SFX_ERR_NONE;
}

/*
* @brief  get voltage temperature.
* @param  voltage_idle : Idle voltage in mV
* @param  voltage_tx   : Volatge in mV
* @param  temperature  : temperature
* @retval None
*/

uint16_t	__s2lp_voltageInTx = 0;
uint8_t		__s2lp_voltageInTxPending = 0;
sfx_u8 MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle,
				       sfx_u16 *voltage_tx,
				       sfx_s16 *temperature)
{

	LOG_DEBUG_S2LP((">> MCU_API_get_voltage_temperature\r\n"));

  // get the idle voltage of the complete device
  // get the temperature of the device
  // if those values are not available : set it to 0x0000
  // return the voltage_idle in 1/1000 volt on 16bits and 1/10 degrees for the temperature */
  (*voltage_idle)=adc_getVdd();
  (*voltage_tx)=(__s2lp_voltageInTx!=0)?__s2lp_voltageInTx:adc_getVdd();
  (*temperature)=adc_getTemperature()/10;
  

  return SFX_ERR_NONE;
}

/*
* @brief  Set System clock.
* @param  None
* @retval None
*/
sfx_u8 MCU_API_delay(sfx_delay_t delay_type)
{
  LOG_DEBUG_S2LP((">> MCU_API_delay %d\r\n",delay_type));
  switch(delay_type)
  {
  case SFX_DLY_INTER_FRAME_TRX:
  case SFX_DLY_INTER_FRAME_TX:
  case SFX_DLY_CS_SLEEP:
    /* ramping should be considered in the ramp up/down:
    since we have 72 samples in the ramp 
    (18ms for each ramp, we need to compensate 36 ms)
    Moreover we have also 6ms of silence (2 before and 4 after packet)
    */
    priv_ST_MCU_API_delay(500-2*ST_RF_API_get_ramp_duration());
    break;
  case SFX_DLY_OOB_ACK:
    priv_ST_MCU_API_delay(2000-2*ST_RF_API_get_ramp_duration());
    break;
  }

  return SFX_ERR_NONE;
}

/*
* @brief  Performs AES_128 encryption.
* @param  encrypted_data : Pointer to encrypted data    
* @param  data_to_encrypt   : pointer to the data to encrypt
* @param  aes_block_len  : block length
* @param  key : Key 
* @param  use_key   : Use key 
* @retval None
*/
sfx_u8 MCU_API_aes_128_cbc_encrypt(sfx_u8 *encrypted_data,
				   sfx_u8 *data_to_encrypt,
				   sfx_u8 aes_block_len,
				   sfx_u8 key[16],
				   sfx_credentials_use_key_t use_key)
{
  LOG_DEBUG_S2LP((">> MCU_API_aes_128_cbc_encrypt\r\n"));
  /* Let the retriever encrypts the requested buffer using the ID_KEY_RETRIEVER function.
  The retriever knows the KEY of this node. */

//  log_info("data_to_encrypt : [ ");
//  for ( int i = 0 ; i < aes_block_len ; i++ ) log_info("%02X",data_to_encrypt[i]);
//  log_info(" ]\r\n");

//  log_info("with key : [ ");
//  for ( int i = 0 ; i < 16 ; i++ ) log_info("%02X",(use_key==CREDENTIALS_KEY_IN_ARGUMENT)?key[i]:0);
//  log_info(" ]\r\n");

#if ( ITSDK_SIGFOX_ENCRYPTION & __SIGFOX_ENCRYPT_SIGFOX ) > 0
  if (use_key==CREDENTIALS_KEY_IN_ARGUMENT) {
	    // Due to a bug in the ST Library in enc_utils_encrypt
	    // the result of the AES computation is wrong when the given key
	    // is passed as a parameter. The use of another AES library solve
	    // this issue until ST fix it.
	    // We have this situation when the sigfox encryption is activated
		struct AES_ctx ctx;
		memcpy(encrypted_data,data_to_encrypt,16);
		bzero(ctx.Iv,16);
		tiny_AES_init_ctx(&ctx,key);
		tiny_AES_CBC_encrypt_buffer(&ctx, encrypted_data, 16);
  } else
#endif
	  enc_utils_encrypt(encrypted_data, data_to_encrypt, aes_block_len, key, use_key);

//  log_info("encrypted_data : [ ");
//  for ( int i = 0 ; i < aes_block_len ; i++ ) log_info("%02X",encrypted_data[i]);
//  log_info(" ]\r\n");
  
  // This is for being ready to measure the voltage during transmission
  // as this function is called before the first transmission..
  __s2lp_voltageInTxPending=70;

  return SFX_ERR_NONE;
}


/**
 * Read the Sigfox NVM block. It contains SeqId and other data updated by
 * the sigfox lib. This block is a structure defined as an enum... (wahou!)
 * with the block size as the last value of the enum.
 */
sfx_u8 MCU_API_get_nv_mem(sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE])
{		
	LOG_DEBUG_S2LP((">> MCU_API_get_nv_mem\r\n"));
#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640

	eeprom_m95640_read(
			&ITSDK_DRIVERS_M95640_SPI,
			ITSDK_SIGFOX_NVM_BASEADDR,
			SFX_NVMEM_BLOCK_SIZE,
			(uint8_t *)read_data
	);

	return 0;	// success

#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
	#error "__SFX_NVM_LOCALEPROM - not yet implemented"
#endif

}

/**
 * This function is used to write back the sigfox block once modified.
 * see the read function for getting more about it.
 */
sfx_u8 MCU_API_set_nv_mem(sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE])
{

#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640

	LOG_DEBUG_S2LP((">> MCU_API_set_nv_mem :"));
	for (int i=0; i< SFX_NVMEM_BLOCK_SIZE ; i++) {
		LOG_DEBUG_S2LP(("0x%X, ",data_to_write[i]));
	}
	LOG_DEBUG_S2LP(("\r\n"));

	// Update the SeqId in the structure (last sent seqId)
	_s2lp_sigfox_config->seqId = (uint16_t)data_to_write[SFX_NVMEM_SEQ_NUM] + (((uint16_t)data_to_write[SFX_NVMEM_SEQ_NUM+1]) << 8);

	eeprom_m95640_write(
			&ITSDK_DRIVERS_M95640_SPI,
			ITSDK_SIGFOX_NVM_BASEADDR,
			SFX_NVMEM_BLOCK_SIZE,
			(uint8_t *)data_to_write
	);

	return 0;	// success

#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
	#error "__SFX_NVM_LOCALEPROM - not yet implemented"
#endif

}







/*
* @brief  Reports test results.
* @param  status : status 
* @param  rssi   : RSSI 
* @retval sfx_u8 : status
*/
sfx_u8 MCU_API_report_test_result(sfx_bool status, sfx_s16 rssi)
{    
	LOG_DEBUG_S2LP((">> MCU_API_report_test_result\r\n"));
  //ST_MANUF_report_CB(status, rssi);
  /* use this function to : print output result : status and rssi on uart if you have one or any link is available on device
   or use a gpio to indicate at least the status
   or to send a message over the air using any link to report the status with rssi
   you could also use the RF part in specific modulation (ook ask or gfsk or else to return status and rssi */
  return SFX_ERR_NONE;
}

/*
* @brief  Gets version.
* @param  version : pointer to the buffer 
* @param  size    : Size 
* @retval sfx_u8  : status
*/
sfx_u8 MCU_API_get_version(sfx_u8 **version, sfx_u8 *size)
{  
  LOG_DEBUG_S2LP((">> MCU_API_get_version\r\n"));
  static uint8_t _libVersion[] = MCU_API_VER;
  (*size) = sizeof(_libVersion);
  (*version) = (sfx_u8*)_libVersion;
  
  return SFX_ERR_NONE;
}


/*
* @brief  Gets device id and payload encryption flag.
* @param  dev_id : pointer to the device id
* @param  payload_encryption_enabled    : Flag 
* @retval sfx_u8  : status
*/
sfx_u8 MCU_API_get_device_id_and_payload_encryption_flag(
  sfx_u8 dev_id[ID_LENGTH],
  sfx_bool *payload_encryption_enabled
){
	LOG_DEBUG_S2LP((">> MCU_API_get_device_id_and_payload_encryption_flag\r\n"));
#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640

	// from Sigfox Retriever library
	enc_utils_get_id(dev_id);
   (*payload_encryption_enabled) = _s2lp_sigfox_config->payload_encryption;

//	log_info("Read ID from Eeprom via retriever (mcu_api.c): [ ");
//	for (int i=0; i< ID_LENGTH ; i++) {
//		log_info("%X ",dev_id[i]);
//	}
//	log_info("]\r\n");
	return SFX_ERR_NONE;

#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
	#error "__SFX_NVM_LOCALEPROM - not yet implemented"
#endif

}

/*
* @brief  Gets initial pac.
* @param  initial_pac : pointer to the initial pac
* @retval sfx_u8  : status
*/
sfx_u8 MCU_API_get_initial_pac(sfx_u8 initial_pac[PAC_LENGTH])
{
	LOG_DEBUG_S2LP((">> MCU_API_get_initial_pac\r\n"));
#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640

	// from Sigfox Retriever library
	enc_utils_get_initial_pac(initial_pac);

	log_info("Read PAC from Eeprom via retriever (mcu_api.c):");
	for (int i=0; i< PAC_LENGTH ; i++) {
		log_info("0x%X, ",initial_pac[i]);
	}
	log_info("\r\n");
	return SFX_ERR_NONE;

#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
	#error "__SFX_NVM_LOCALEPROM - not yet implemented"
#endif

}

/**
 * Change the low_power flag configuration
 */
void ST_MCU_API_LowPower(sfx_u8 low_power_flag) {
	LOG_DEBUG_S2LP((">> ST_MCU_API_LowPower\r\n"));
	_s2lp_sigfox_config->low_power_flag=low_power_flag;
}



/*
* @brief  MCU API shutdown.
* @param  value : TRUE/FALSE
* @retval none
*/
void ST_MCU_API_Shutdown(sfx_u8 value)
{
  LOG_DEBUG_S2LP((">> ST_MCU_API_Shutdown %d\r\n",value));
  if(value==ST_TRUE) {
	 s2lp_shutdown();
#if defined(FOR_ARIB) || defined(FOR_ALL)
#error "Not yet supported Tcxo"
    RadioTcxoOff();
#endif
  } else {
#if defined(FOR_ARIB) || defined(FOR_ALL)
#error "Not yet supported Tcxo"
     RadioTcxoInit();
     RadioTcxoOn();
#endif
     s2lp_wakeup();
  }
  // It seems like a 10ms delay is a good idea to have the library
  // correctly working
  priv_ST_MCU_API_delay(10);
}

/*
* @brief  MCU API SPI RAW.
* @param  n_bytes : number of bytes
* @param  in_buffer : input buffer
* @param  out_buffer : output buffer
* @param  can_return_bef_tx : return buffer
* @retval none
*/
void ST_MCU_API_SpiRaw(uint8_t n_bytes,
                       uint8_t* in_buffer, 
                       uint8_t* out_buffer, 
                       uint8_t can_return_bef_tx)
{
	//LOG_DEBUG_S2LP((">> ST_MCU_API_SpiRaw %d 0x%X\r\n",n_bytes,in_buffer[0]));
	s2lp_spi_accessRaw(
			&ITSDK_S2LP_SPI,
			in_buffer,
			out_buffer,
			n_bytes
	);
	// Hack to get the Reception RSSI and have the same value
	// as the one returned to sigfox
	if ( in_buffer[1] == 0xA2 ) {
		_s2lp_sigfox_config->lastReadRssi = out_buffer[2];
	}
}


/*
* @brief  Sets encryption payload.
* @param  ePayload : encryption payload
* @retval none
*/
void ST_MCU_API_SetEncryptionPayload(uint8_t ePayload)
{
	LOG_DEBUG_S2LP((">> ST_MCU_API_SetEncryptionPayload\r\n"));
	_s2lp_sigfox_config->payload_encryption = ePayload;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




/********************************************************************************
 * ST Key Retriever interface
 * Assuming the Flash is not supported.
 */

void EepromRead(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer) {
	LOG_DEBUG_S2LP((">> EepromRead\r\n"));
	#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640

	eeprom_m95640_read(
			&ITSDK_DRIVERS_M95640_SPI,
			nAddress,
			cNbBytes,
			pcBuffer
	);

//	log_info("Eeprom read 0x%d, %dB\r\n",nAddress,cNbBytes);
//	log_info("  >");
//	for (int i = 0 ; i < cNbBytes ; i++ ) {
//		log_info("0x%X, ",pcBuffer[i]);
//	}
//	log_info("\r\n");

	#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
	#else
		#error "Unsupported NVM Source"
	#endif
}


NVM_RW_RESULTS NVM_ReadBoardData(NVM_BoardDataType *data)
{
	LOG_DEBUG_S2LP((">> NVM_ReadBoardData\r\n"));
    return NVM_READ_ERROR;
}

NVM_RW_RESULTS NVM_Read(uint32_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  NVM_RW_RESULTS tRet = NVM_RW_OK;
  LOG_DEBUG_S2LP((">> NVM_Read\r\n"));
  EepromRead(nAddress, cNbBytes, pcBuffer);

  return tRet;
}


/* ****************************************************************************************************
 * GPIO / INTERRUPT FUNCTIONS
 * ****************************************************************************************************
 */

/**
 * Interrupt Handler
 */
void __GPIO_IRQHandler(uint16_t GPIO_Pin);
gpio_irq_chain_t __sfx_gpio_irq = {
		__GPIO_IRQHandler,
		0,
		NULL
};
volatile uint8_t __pendingIrqDelayed=0;
void __GPIO_IRQHandler(uint16_t GPIO_Pin) {

  if(GPIO_Pin==ITSDK_S2LP_GPIO3_PIN) {
	  if (ST_RF_API_Get_Continuous_TX_Flag()==0) {
		// Most of the Irq are delayed during Tx
	   	__pendingIrqDelayed=1;
	  } else {
	    ST_RF_API_S2LP_IRQ_CB(); // If the CBPSK is implemented trigger TX State Machine
	  }
  }

}

/**
 * Configure one of the GPIO's fro managing interrupts
 * new_state true : enbale the gpio interrupt
 * 			 false : disable interrupt
 * trigger   true : interrupt on signal rising
 *           false : interrupt on signal falling
 * pin is the S2LP GPIO port 0,1,2,3
 */
void ST_MCU_API_GpioIRQ(sfx_u8 pin, sfx_u8 new_state, sfx_u8 trigger)
{
  LOG_DEBUG_S2LP((">> ST_MCU_API_GpioIRQ %d %d %d \r\n",pin,new_state,trigger));

  uint16_t gpioPin;
  uint8_t  gpioBank;

  switch(pin)
  {
  case 0:
    gpioPin = ITSDK_S2LP_GPIO0_PIN;
    gpioBank = ITSDK_S2LP_GPIO0_BANK;
    break;
  case 1:
    gpioPin = ITSDK_S2LP_GPIO1_PIN;
    gpioBank = ITSDK_S2LP_GPIO1_BANK;
    break;
  case 2:
    gpioPin = ITSDK_S2LP_GPIO2_PIN;
    gpioBank = ITSDK_S2LP_GPIO2_BANK;
    break;
  case 3:
    gpioPin = ITSDK_S2LP_GPIO3_PIN;
    gpioBank = ITSDK_S2LP_GPIO3_BANK;
    break;
  default:
	  return;
  }
  gpio_interruptClear(gpioBank, gpioPin);

  if(new_state) {
    gpio_registerIrqAction(&__sfx_gpio_irq);	// Install the action as the irq can be activated outside this code
	if ( trigger ) {
		gpio_configure(gpioBank,gpioPin, GPIO_INTERRUPT_RISING_PULLDWN );
	} else {
		gpio_configure(gpioBank,gpioPin, GPIO_INTERRUPT_FALLING_PULLUP );
	}
	gpio_interruptPriority(gpioBank,gpioPin,IRQ_PRIORITY, 0x00);
	gpio_interruptEnable(gpioBank,gpioPin);
  } else {
	gpio_removeIrqAction(&__sfx_gpio_irq);
	gpio_configure(gpioBank,gpioPin, GPIO_INPUT_PULLDOWN );
	gpio_interruptDisable(gpioBank,gpioPin);
  }
}

/**
 * This function is called with a loop until the interrupt is detected
 * Eventually we can have a switch to low_power mode during this period
 * or just return.
 * This function is also called during the downlink processing, waiting for
 * the end of the timer.
 * The interrupt flag can be fired externally and need to be proceed here
 */
void ST_MCU_API_WaitForInterrupt(void)
{
	LOG_DEBUG_S2LP(("+"));
	if(__pendingIrqDelayed) {
	   ST_RF_API_S2LP_IRQ_CB();
	   __pendingIrqDelayed=0;
	} else {
	  // special code for measuring the voltage during transmission
	  // it is measured during the first frame (n=1) this is
	  // initialized from the MCU_API_aes_128_cbc_encrypt called before
	  // the transmission start.
	  // we are waiting for 50 loops here for being in the middle of the transmission
	  if ( __s2lp_voltageInTxPending > 0 ) {
		  __s2lp_voltageInTxPending--;
		  if (__s2lp_voltageInTxPending==0) {
			  __s2lp_voltageInTx=adc_getVdd();
			  LOG_DEBUG_S2LP(("_"));
		  }
	  }
	  // other actions
	  #if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0
		 lowPower_switch();
	  #endif
	  itsdk_stimer_run();
	  #if ITSDK_WDG_MS > 0
	  	wdg_refresh();
	  #endif
	}
}


/* ****************************************************************************************************
 * TIMER FUNCTIONS BASED ON RTC
 * ****************************************************************************************************
 */

/**
 * Timer end handler
 */
void __TIMER_Handler(uint32_t v) {
	LOG_DEBUG_S2LP((">> __TIMER_Handler (END)\r\n"));
	ST_RF_API_Timer_CB(TIMER_STOP);
}


/**
 * Start a Timer with the given duration in S
 * Long timer... use the RTC
 * Rq: the var name is in S but the comment was in MS ... YOLO ST !!
 *     S is the right one
 */
sfx_u8 MCU_API_timer_start(sfx_u32 time_duration_in_s)
{
	LOG_DEBUG_S2LP((">> MCU_API_timer_start %d\r\n",time_duration_in_s));

	ST_RF_API_Timer_CB(TIMER_START);
	if (   itsdk_stimer_register(
				(time_duration_in_s)*1000-500,
				__TIMER_Handler,
				0,
				TIMER_ACCEPT_LOWPOWER
			) != TIMER_INIT_SUCCESS ) {
		ITSDK_ERROR_REPORT(ITSDK_ERROR_STIMER_INIT_FLD,0);
	}
	return SFX_ERR_NONE;
}


/**
 *  Stop a running timer
 */
sfx_u8 MCU_API_timer_stop(void)
{
	LOG_DEBUG_S2LP((">> MCU_API_timer_stop \r\n"));

	itsdk_stimer_stop(
			__TIMER_Handler,
			0
	);
	return SFX_ERR_NONE;
}

/**
 * This function is called for waiting the end of the running timer
 * during this time we call the WaitForInterrupt function to switch
 * the device in low power mode.
 */
sfx_u8 MCU_API_timer_wait_for_end(void)
{
	LOG_DEBUG_S2LP((">> MCU_API_timer_wait_for_end \r\n"));

	while ( itsdk_stimer_isRunning(
				__TIMER_Handler,
				0
			)
		  ) {
		ST_MCU_API_WaitForInterrupt();
	}

	return SFX_ERR_NONE;
}


/**
 * Timer2 end handler
 */
void __TIMER2_Handler(uint32_t v) {
	LOG_DEBUG_S2LP((">> __TIMER2_Handler (END)\r\n"));
	ST_RF_API_Timer_Channel_Clear_CB();
}

/**
 * Start a timer for carrier sense in MS
 * Use the RTC Timer if available
 * The duration are 5000ms and 8000ms
 */
sfx_u8 MCU_API_timer_start_carrier_sense(sfx_u16 time_duration_in_ms)
{
	LOG_DEBUG_S2LP((">> MCU_API_timer_start_carrier_sense %d \r\n",time_duration_in_ms));

	ST_RF_API_Timer_CB(TIMER_START);
	if (   itsdk_stimer_register(
			time_duration_in_ms,
			__TIMER2_Handler,
			0,
			TIMER_ACCEPT_LOWPOWER
		) != TIMER_INIT_SUCCESS ) {
			ITSDK_ERROR_REPORT(ITSDK_ERROR_STIMER_INIT_FLD,0);
		}
	return SFX_ERR_NONE;
}

/*
* @brief  Timer stop carrier sense.
* @param  None
* @retval sfx_u8 : status
*/
sfx_u8 MCU_API_timer_stop_carrier_sense(void)
{
	LOG_DEBUG_S2LP((">> MCU_API_timer_stop_carrier_sense \r\n"));

	itsdk_stimer_stop(
			__TIMER2_Handler,
			0
	);
	return SFX_ERR_NONE;
}

/* **********************************************************************************************************
 * Eprom not documented extension
 * Not to be used in normal operations
 * **********************************************************************************************************
 */

/**
 * extract the key from the eeprom
 */
void enc_utils_retrieve_key(uint8_t * key) {

	uint8_t	raw[32];
	LOG_DEBUG_S2LP((">> enc_utils_retrieve_key \r\n"));

#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640

	eeprom_m95640_read(
			&ITSDK_DRIVERS_M95640_SPI,
			ITSDK_SIGFOX_NVM_IDBASEADDR,
			32,
			raw
	);

	for ( int i = 0 ; i < 16 ; i++ ) {
		key[i]=raw[16+i];
	}

	// Clean the stack
	for ( int i = 0 ; i < 32 ; i++ ) {
		raw[i] = 0;
	}

#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
	#error "__SFX_NVM_LOCALEPROM - not yet implemented"
#else
	#error "NVM Storage - not yet implemented"
#endif

}

#endif // ITSDK_WITH_SIGFOX_LIB > 0



