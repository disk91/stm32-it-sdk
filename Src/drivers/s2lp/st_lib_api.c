/* ==========================================================
 * st_lib_api.c - S2LP (STm SubGhz transceiver) API interface
 * Project : IngeniousThings SDK
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
#include <it_sdk/logger/logger.h>
#include <it_sdk/lowpower/lowpower.h>
#include <stm32l_sdk/rtc/rtc.h>
#include <drivers/sigfox/sigfox_types.h>
#include <drivers/sigfox/sigfox_api.h>
#include <drivers/s2lp/st_lib_api.h>
#include <drivers/s2lp/st_mcu_api.h>
#include <drivers/s2lp/st_rf_api.h>
#include <drivers/s2lp/s2lp.h>
#include <drivers/s2lp/s2lp_spi.h>
#include <drivers/sigfox/sigfox_retriever.h>
#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640
	#include <drivers/eeprom/m95640/m95640.h>
#endif
#include <it_sdk/wrappers.h>
#include <string.h>


s2lp_config_t *	_s2lp_sigfox_config;
void itsdk_sigfox_configInit(s2lp_config_t * cnf) {
	_s2lp_sigfox_config = cnf;
}



// Reset the system clock to default
void ST_MCU_API_SetSysClock(void) {
	log_info(">> ST_MCU_API_SetSysClock\r\n");
	SystemClock_Config();
}




/*
* @brief  Do the Timer calibration.
* @param  None
* @retval None
*/
//void ST_MCU_API_TimerCalibration(uint16_t duration_ms)
//{
//  TIM_HandleTypeDef  Tim2_Handler={.Instance=TIM2};
//  uint16_t c;
//  Configure_RTC_Clock();
//  notify_end=1;
//  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RtcHandler, RTC_FLAG_WUTF);
//  __HAL_RTC_CLEAR_FLAG(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);
//
//  next_rtc_wakeup=0;
//
//  RadioTimersTimConfig(&Tim2_Handler,16000-1,65535-1);
//  __HAL_TIM_DISABLE_IT(&Tim2_Handler, TIM_IT_UPDATE);
//  HAL_NVIC_DisableIRQ(TIM2_IRQn);
//  HAL_TIM_Base_Start(&Tim2_Handler);
//
//  HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandler,rtc_presc*duration_ms/1000,RTC_WAKEUPCLOCK_RTCCLK_DIV16);
//  while(!rtc_irq);
//  c=Tim2_Handler.Instance->CNT;
//  rtc_irq=0;
//  HAL_TIM_Base_Stop(&Tim2_Handler);
//
//  rtc_presc=duration_ms*rtc_presc/c;
//}




/*
* @brief  Timer Interrupt handler.
* @param  None
* @retval None
*/
//void TIM2_IRQHandler(void)
//{
//  if(__HAL_TIM_GET_IT_SOURCE(&Tim2_Handler, TIM_IT_UPDATE) !=RESET)
//  {
//    ST_RF_API_Timer_Channel_Clear_CB();
//
//    __HAL_TIM_CLEAR_IT(&Tim2_Handler, TIM_IT_UPDATE);
//    RadioTimersState(&Tim2_Handler, DISABLE);
//  }
//}

/*
 *  Delay in Ms
 */
static void priv_ST_MCU_API_delay(uint32_t delay_ms) {
	//log_info(">> priv_ST_MCU_API_delay\r\n");
	itsdk_delayMs(delay_ms);
}

/**
 * Static memory allocation
 */
sfx_u8 MCU_API_malloc(sfx_u16 size, sfx_u8 **returned_pointer)
{
  static sfx_u8 mem[ITSDK_SIGFOX_MEM_SIZE];
  
  log_error("Sigfox lib mem req: %dB\r\n",size);
  if(size>ITSDK_SIGFOX_MEM_SIZE) {
	  log_error("Requesting more memory than maximum allowable\r\n");
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
  return SFX_ERR_NONE;
}

/*
* @brief  get voltage temperature.
* @param  voltage_idle : Idle voltage
* @param  voltage_tx   : Volatge temperature
* @param  temperature  : temperature
* @retval None
*/
sfx_u8 MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle,
				       sfx_u16 *voltage_tx,
				       sfx_s16 *temperature)
{
	log_info(">> MCU_API_get_voltage_temperature\r\n");

  /* get the idle voltage of the complete device
  get the temperature of the device
  if those values are not available : set it to 0x0000
  return the voltage_idle in 1/10 volt on 16bits and 1/10 degrees for the temperature */
  (*voltage_idle)=adc_getVdd()/100;
  (*voltage_tx)=adc_getVdd()/100;
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
	log_info(">> MCU_API_delay %d\r\n",delay_type);
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
  log_info(">> MCU_API_aes_128_cbc_encrypt\r\n");
  /* Let the retriever encrypts the requested buffer using the ID_KEY_RETRIEVER function.
  The retriever knows the KEY of this node. */

  log_info("data_to_encrypt : [ ");
  for ( int i = 0 ; i < aes_block_len ; i++ ) log_info("%02X",data_to_encrypt[i]);
  log_info(" ]\r\n");

  enc_utils_encrypt(encrypted_data, data_to_encrypt, aes_block_len, key, use_key);
  log_info("encrypted_data : [ ");
  for ( int i = 0 ; i < aes_block_len ; i++ ) log_info("%02X",encrypted_data[i]);
  log_info(" ]\r\n");
  
  

  return SFX_ERR_NONE;
}


/**
 * Read the Sigfox NVM block. It contains SeqId and other data updated by
 * the sigfox lib. This block is a structure defined as an enum... (wahou!)
 * with the block size as the last value of the enum.
 */
sfx_u8 MCU_API_get_nv_mem(sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE])
{		
	log_info(">> MCU_API_get_nv_mem\r\n");
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

	log_info("Write into Eeprom (mcu_api.c):");
	for (int i=0; i< SFX_NVMEM_BLOCK_SIZE ; i++) {
		log_info("0x%X, ",data_to_write[i]);
	}
	log_info("\r\n");

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
	log_info(">> MCU_API_report_test_result\r\n");
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
	log_info(">> MCU_API_get_version\r\n");
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
sfx_u8 MCU_API_get_device_id_and_payload_encryption_flag(\
  sfx_u8 dev_id[ID_LENGTH],
  sfx_bool *payload_encryption_enabled)
{
	log_info(">> MCU_API_get_device_id_and_payload_encryption_flag\r\n");
#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640

	// from Sigfox Retriever library
	enc_utils_get_id(dev_id);
   (*payload_encryption_enabled) = _s2lp_sigfox_config->payload_encryption;

	log_info("Read ID from Eeprom via retriever (mcu_api.c):");
	for (int i=0; i< ID_LENGTH ; i++) {
		log_info("0x%X, ",dev_id[i]);
	}
	log_info("\r\n");
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
	log_info(">> MCU_API_get_initial_pac\r\n");
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
	log_info(">> ST_MCU_API_LowPower\r\n");
	_s2lp_sigfox_config->low_power_flag=low_power_flag;
}



/*
* @brief  MCU API shutdown.
* @param  value : TRUE/FALSE
* @retval none
*/
void ST_MCU_API_Shutdown(sfx_u8 value)
{
	log_info(">> ST_MCU_API_Shutdown %d\r\n",value);
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
	//log_info(">> ST_MCU_API_SpiRaw %d 0x%X\r\n",n_bytes,in_buffer[0]);
	s2lp_spi_accessRaw(
			&ITSDK_S2LP_SPI,
			in_buffer,
			out_buffer,
			n_bytes
	);

}


/*
* @brief  Sets encryption payload.
* @param  ePayload : encryption payload
* @retval none
*/
void ST_MCU_API_SetEncryptionPayload(uint8_t ePayload)
{
	log_info(">> ST_MCU_API_SetEncryptionPayload\r\n");
	_s2lp_sigfox_config->payload_encryption = ePayload;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




/********************************************************************************
 * ST Key Retriever interface
 * Assuming the Flash is not supported.
 */

void EepromRead(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer) {
	log_info(">> EepromRead\r\n");
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
	log_info(">> NVM_ReadBoardData\r\n");
    return NVM_READ_ERROR;
}

NVM_RW_RESULTS NVM_Read(uint32_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  NVM_RW_RESULTS tRet = NVM_RW_OK;
  log_info(">> NVM_Read\r\n");
  EepromRead(nAddress, cNbBytes, pcBuffer);

  return tRet;
}


/* ****************************************************************************************************
 * GPIO / INTERRUPT FUNCTIONS BASED ON RTC
 * ****************************************************************************************************
 */

/**
 * Interrupt Handler
 */
void __GPIO_IRQHandler(uint16_t GPIO_Pin);
gpio_irq_chain_t __sfx_gpio_irq = {
		__GPIO_IRQHandler,
		NULL
};
volatile uint8_t __pendingIrqDelayed=0;
void __GPIO_IRQHandler(uint16_t GPIO_Pin) {

  if(GPIO_Pin==ITSDK_S2LP_GPIO3_PIN) {
	  //log_info(">> GpioIRQ\r\n");
	  if (ST_RF_API_Get_Continuous_TX_Flag()==0) {
		// Most of the Irq are delayed during Tx
		  log_info(":");
	   	__pendingIrqDelayed=1;
	  } else {
	    ST_RF_API_S2LP_IRQ_CB(); // If the CBPSK is implemented trigger TX State Machine
	  }
  } else {
	  log_info(">> GpioIRQ Other\r\n");
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
  log_info(">> ST_MCU_API_GpioIRQ\r\n");
  gpio_registerIrqAction(&__sfx_gpio_irq);	// Install the action as the irq can be activated outside this code

  uint16_t gpioPin;
  uint8_t  gpioBank;

  log_info("    S2LP Configuring Gpio %d as %d,%d\r\n",pin,new_state,trigger);

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
	if ( trigger ) {
		gpio_configure(gpioBank,gpioPin, GPIO_INTERRUPT_RISING );
	} else {
		gpio_configure(gpioBank,gpioPin, GPIO_INTERRUPT_FALLING );
	}
	gpio_interruptPriority(gpioBank,gpioPin,IRQ_PRIORITY, 0x00);
	gpio_interruptEnable(gpioBank,gpioPin);
  } else {
	gpio_interruptDisable(gpioBank,gpioPin);
	gpio_configure(gpioBank,gpioPin, GPIO_INPUT );
  }
}

/**
 * This function is called with a loop until the interrupt is detected
 * Eventually we can have a switch to low_power mode during this period
 * or just return.
 * The interrupt flag can be fired externally and need to be proceed here
 */
void ST_MCU_API_WaitForInterrupt(void)
{
	//log_info(">> ST_MCU_API_WaitForInterrupt\r\n");
	log_info("!");
	if(__pendingIrqDelayed) {
	   ST_RF_API_S2LP_IRQ_CB();
	   __pendingIrqDelayed=0;
	} else {
	  #if ( ITSDK_LOWPOWER_MOD & __LOWPWR_MODE_WAKE_GPIO ) > 0
		 lowPower_switch();
	  #endif
// watchdog refresh as we prefer to reboot if longuer than WDG if IRQ fireing issue
//	  #if ITSDK_WDG_MS > 0
//		wdg_refresh();
//	  #endif

	}

//
//	// Low power mode is manage outside of the driver
//	// (by the way, there is no reason to have a drive to decide a such action
//
//	// @TODO
//	// The ST_RF_API_S2LP_IRQ_CB(); should be called from the IRQ handler
//	// The S2LP Irq should wake up the MCU
//	log_info("Request Entering Low Power Mode\r\n");
//
//
//#ifndef DEBUG
//  if(low_power && (!carrier_sense_tim_started || rtc_in_use_for_cs))
//  {
//    setGpioLowPower();
//
//    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
//
//    ST_MCU_API_SetSysClock();
//
//    setGpioRestore();
//  }
//#endif
//
//
//  Traitement en cas d'irq car dans un cas particulier on veux que l'irq soit traitée plus tard...
//  if(s2lp_irq_raised)
//  {
//    ST_RF_API_S2LP_IRQ_CB();
//    s2lp_irq_raised=0;
//  }
}


/* ****************************************************************************************************
 * TIMER FUNCTIONS BASED ON RTC
 * ****************************************************************************************************
 */

/**
 * RTC Interrupt Handler
 */
void __RTC_IRQHandler(RTC_HandleTypeDef *h);
uint32_t __st_lib_rtc_loops_pending = 0;
rtc_irq_chain_t __sfx_rtc_irq = {
		__RTC_IRQHandler,
		NULL
};
void __RTC_IRQHandler(RTC_HandleTypeDef *h) {

  rtc_disableWakeUp();
  if (__st_lib_rtc_loops_pending == 0 ) {
	  // Global timer is finished
	  ST_RF_API_Timer_CB(TIMER_STOP);
	  log_info("Finished RTC\r\n");
  } else {
	  // We need to run the timer up to the next 100ms slot
	  __st_lib_rtc_loops_pending--;
	  rtc_runRtcUntilTicks(rtc_getTicksFromDuration(1000));
  }
//
//  if (__st_lib_rtc_loops_cs_pending == 0 ) {
//	  // Global timer is finished
//	  ST_RF_API_Timer_Channel_Clear_CB();
//	  __st_lib_rtc_loops_cs_pending=-1;
//	  log_info("Finished CS\r\n");
//  } else {
//	  // We need to run the timer up to the next 100ms slot
//	  __st_lib_rtc_loops_cs_pending--;
//  }
//
//  if (__st_lib_rtc_loops_pending >= 0 || __st_lib_rtc_loops_cs_pending >= 0 ) {
//	  rtc_runRtcUntilTicks(__RTC_TICKS_PER_LOOP);
//  }

/*
  HAL_RTCEx_DeactivateWakeUpTimer(h);

  if(next_rtc_wakeup==0)
  {
    rtc_irq=1;
    rtc_in_use=0;
    if(notify_end)
    {
      if(rtc_in_use_for_cs)
      {
	rtc_in_use_for_cs=0;
	ST_RF_API_Timer_Channel_Clear_CB();
      }
      else
      {
	ST_RF_API_Timer_CB(TIMER_STOP);
      }
    }
  }
  else
  {
    MCU_API_timer_start(next_rtc_wakeup);
    n_intermediate_tim_irq++;
  }
  */
}


/**
 * Start a Timer with the given duration in S
 * Long timer... use the RTC
 * Rq: the var name is in S but the comment was in MS ... YOLO ST !!
 *     S is the right one
 */
sfx_u8 MCU_API_timer_start(sfx_u32 time_duration_in_s)
{
	log_info(">> MCU_API_timer_start %d\r\n",time_duration_in_s);

	ST_RF_API_Timer_CB(TIMER_START); // Notify the STM Layer

	// Register the interrupt handler
	rtc_registerIrqAction(&__sfx_rtc_irq);

	// Determine the number of cycles
	uint32_t totalTicks = rtc_getTicksFromDuration(time_duration_in_s*1000);
	__st_lib_rtc_loops_pending = ( totalTicks / rtc_getTicksFromDuration(1000) );

	// Determine the subdivision time to add
	totalTicks -=  (__st_lib_rtc_loops_pending * rtc_getTicksFromDuration(1000) );

	// Start timer - assuming as > 1s no risk to have __st_lib_rtc_loops_pending = 0
	// Run for the sub cycle time part
	if ( totalTicks == 0 ) {
		__st_lib_rtc_loops_pending--;
		totalTicks = rtc_getTicksFromDuration(1000);
	}
	rtc_runRtcUntilTicks(totalTicks);


	return SFX_ERR_NONE;
}


/**
 *  Stop a running timer
 */
sfx_u8 MCU_API_timer_stop(void)
{
	log_info(">> MCU_API_timer_stop \r\n");
	rtc_disableWakeUp();
	__st_lib_rtc_loops_pending=0;
	rtc_removeIrqAction(&__sfx_rtc_irq);
	return SFX_ERR_NONE;
}

/**
 * Not really clear, it seems this function is
 * called during a timer, potentially it is active scrutation
 * for timer end... intially it calls a function to go low power
 * ... so let see later how to works.
 */
sfx_u8 MCU_API_timer_wait_for_end(void)
{
	log_info(">> MCU_API_timer_wait_for_end \r\n");
	return SFX_ERR_NONE;


//  while(!rtc_irq)/*(!(next_rtc_wakeup==0 || rtc_irq==1)) */
//  {
//    ST_MCU_API_WaitForInterrupt();
//  }
//  rtc_irq=0;
//  PRINTF("MCU_API_timer_wait_for_end OUT\n\r");
//  return SFX_ERR_NONE;
}






/* ****************************************************************************************************
 * TIMER FUNCTIONS BASED ON RTC
 * ****************************************************************************************************
 */


void __RTC_IRQHandler2(RTC_HandleTypeDef *h);
rtc_irq_chain_t __sfx_rtc_irq2 = {
		__RTC_IRQHandler2,
		NULL
};
void __RTC_IRQHandler2(RTC_HandleTypeDef *h) {

  rtc_disableWakeUp();
  if (__st_lib_rtc_loops_pending == 0 ) {
	  // Global timer is finished
	  ST_RF_API_Timer_Channel_Clear_CB();
	  log_info("Finished RTC\r\n");
  } else {
	  // We need to run the timer up to the next 100ms slot
	  __st_lib_rtc_loops_pending--;
	  rtc_runRtcUntilTicks(rtc_getTicksFromDuration(1000));
  }

}

/**
 * Start a timer for carrier sense in MS
 * Use the RTC Timer if available
 */
sfx_u8 MCU_API_timer_start_carrier_sense(sfx_u16 time_duration_in_ms)
{
	log_info(">> MCU_API_timer_stop_carrier_sense %d \r\n",time_duration_in_ms);

	if (! rtc_existAction(&__sfx_rtc_irq) ) {

		// RTC timer is available
		// Register the interrupt handler
		rtc_registerIrqAction(&__sfx_rtc_irq2);

		// Determine the number of cycles
		uint32_t totalTicks = rtc_getTicksFromDuration(time_duration_in_ms);
		__st_lib_rtc_loops_pending = ( totalTicks / rtc_getTicksFromDuration(1000) );

		// Determine the subdivision time to add
		totalTicks -=  (__st_lib_rtc_loops_pending * rtc_getTicksFromDuration(1000) );

		// Start timer - assuming as > 1s no risk to have __st_lib_rtc_loops_pending = 0
		// Run for the sub cycle time part
		if ( totalTicks == 0 ) {
			__st_lib_rtc_loops_pending--;
			totalTicks = rtc_getTicksFromDuration(1000);
		}
		rtc_runRtcUntilTicks(totalTicks);
	} else {
		log_warn(">> MCU_API_timer_stop_carrier_sense - RTC in use \r\n");
		itsdk_delayMs(time_duration_in_ms);
		ST_RF_API_Timer_Channel_Clear_CB();
	}


//  uint32_t rtc_wup_tick, next_rtc_wakeup_tick;
//
//  log_info(">> MCU_API_timer_stop_carrier_sense %d \r\n",time_duration_in_ms);
//
//  PRINTF("MCU_API_timer_start_carrier_sense IN (rtc_in_use=%d)\n\r",rtc_in_use);
//
//  carrier_sense_tim_started=1;
//
//  if(rtc_in_use)
//  {
//    uint32_t n = ((uint32_t)time_duration_in_ms*16000);
//    uint16_t a,b;
//    RadioTimersFindFactors(n,&a,&b);
//    RadioTimersTimConfig(&Tim2_Handler,a-1,b-1);
//    RadioTimersState(&Tim2_Handler, ENABLE);
//  }
//  else
//  {
//    Configure_RTC_Clock();
//    notify_end = 1;
//    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RtcHandler, RTC_FLAG_WUTF);
//    __HAL_RTC_CLEAR_FLAG(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);
//    n_intermediate_tim_irq=0;
//    rtc_in_use=1;
//    rtc_in_use_for_cs=1;
//    /*rtc_wup_tick = time_duration_in_ms/1000*rtc_presc; */
//    rtc_wup_tick = (time_duration_in_ms*rtc_presc)/1000;
//    if(rtc_wup_tick>65535) /* Mapped register is 16bit */
//    {
//      next_rtc_wakeup_tick=rtc_wup_tick-65535;
//      rtc_wup_tick=65535;
//    }
//    else
//    {
//      next_rtc_wakeup_tick=0;
//    }
//
//    /*next_rtc_wakeup = next_rtc_wakeup_tick/rtc_presc*1000; */
//    next_rtc_wakeup = (next_rtc_wakeup_tick*1000)/rtc_presc;
//    HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandler, rtc_wup_tick, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
//  }
//
//  PRINTF("MCU_API_timer_start_carrier_sense OUT\n\r");
//
  return SFX_ERR_NONE;
}

/*
* @brief  Timer stop carrier sense.
* @param  None
* @retval sfx_u8 : status
*/
sfx_u8 MCU_API_timer_stop_carrier_sense(void)
{
	log_info(">> MCU_API_timer_stop_carrier_sense \r\n");
	rtc_disableWakeUp();
	__st_lib_rtc_loops_pending=0;
	rtc_removeIrqAction(&__sfx_rtc_irq2);
	return SFX_ERR_NONE;

//	log_info(">> MCU_API_timer_stop_carrier_sense \r\n");
//
//  if(rtc_in_use_for_cs)
//  {
//    HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandler);
//    rtc_in_use=0;
//    rtc_in_use_for_cs=0;
//  }
//  else
//  {
//    RadioTimersState(&Tim2_Handler, DISABLE);
//  }
//  carrier_sense_tim_started=0;
//
//  PRINTF("MCU_API_timer_stop_carrier_sense OUT\n\r");
//
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

/**
 * Some hack of the ST IDRetriever library to extract and set a better protection to
 * the Sigfox secret key stored in clear in the RAM after
 * Basically the strcuture created have the following format
 *
 * struct s_internal {
 *	uint8_t 	unk0[4];			// 0		- 00 00 00 07
 *	uint8_t 	unk1;				// 4		- 0x07
 *	uint8_t		unk2[3];			// 5		- 00 00 00
 *	uint8_t		pac[8];				// 8 		- 36 61 3F 89 44 0B D6 47
 *	uint32_t	deviceID;			// 16		-
 *	uint8_t		unk3[16];			// 20		- 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 *	uint8_t		private_key[16];	// 36       - XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX
 *  ...
 * }
 */

uint8_t * __keyPtr = 0;
uint32_t enc_search4key(int32_t deviceId, uint8_t * pac) {
	 uint8_t * ramPtr = (uint8_t *)0x20000010;	// start at 16 as we search for deviceID

	 int i;
	 for( i = 16 ; i < ITSDK_RAM_SIZE ; i++ ) {
		 // search for deviceId
		 if ( *((uint32_t *)ramPtr) == deviceId ) {
			 // search for pac at -8 bytes
			 int j = 0;
			 while ( j < 8 ) {
				 if ( *(ramPtr-8+j) != pac[j] ) break;
				 j++;
			 }
			 // found !
			 if ( j == 8 ) break;
		 }
		 ramPtr+=4;	// stm32 only support 32b aligned memory access apparently
	 }
	 __keyPtr = ( i < ITSDK_RAM_SIZE )?ramPtr+20:(uint8_t)0;
	 return (uint32_t)__keyPtr;
}

 bool enc_retreive_key(int32_t deviceId, uint8_t * pac, uint8_t * key) {
	 if ( __keyPtr == 0 ) enc_search4key(deviceId, pac);
	 if ( __keyPtr != 0 ) {
		 memcpy(key,__keyPtr,16);
		 return true;
	 }
	 return false;
 }

 /**
  * Protect the private key from being retreived from the memory in clear
  */
 void enc_protect_key() {
	 if ( __keyPtr == 0 ) return;
	 uint32_t key = ITSDK_SIGFOX_PROTECT_KEY;
	 uint32_t * pk = (uint32_t *)__keyPtr;

	 for ( int i = 0  ; i < 4 ; i++,pk++ ) *pk ^= key;
 }

 void enc_unprotect_key() {
	 enc_protect_key();
 }

