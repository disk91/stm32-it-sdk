/**
* @file    mcu_api.c
* @author  STM32ODE Team, NOIDA
* @version 1.0.0
* @date    24-September-2018
* @brief   This is a ST-SigFox demo that shows how to use the sigfox protocol to 
*          send a message to the base stations each time the push button 
*          is pressed.The data sent is a number representing the number of times 
*	   the button has been pressed from the boot.        
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <it_sdk/config.h>
#include <it_sdk/logger/logger.h>
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

//#include "radio_shield_Config.h"
//#include "sigfox_retriever.h"
//#include "MCU_Interface.h"
//#include "nvm_api.h"
//#include "radio_gpio.h"
//#include "radio_spi.h"
//#include "radio_timers.h"
//#include "cube_hal.h"



/*
static RTC_HandleTypeDef RtcHandler={.Instance=RTC};
static volatile uint8_t rtc_irq=0, rtc_in_use=0, notify_end=0, rtc_in_use_for_cs=0;
static volatile uint8_t low_power=1,carrier_sense_tim_started=0;
static volatile uint32_t next_rtc_wakeup=0,n_intermediate_tim_irq=0;
static volatile int16_t rtc_presc=2375;
static volatile uint8_t s2lp_irq_raised=0;
static TIM_HandleTypeDef  Tim2_Handler={.Instance=TIM2};
static uint8_t _encryptedPayload = 0;
static const uint8_t _libVersion[] = MCU_API_VER;

// this callback should be implemented by the main
void Appli_Exti_CB(uint16_t GPIO_Pin);
void ST_MANUF_report_CB(uint8_t status, int32_t rssi);
*/




sigfox_configuration_t	itsdk_sigfox_config;
void itsdk_sigfox_configInit() {
	itsdk_sigfox_config.low_power_flag = ITSDK_SIGFOX_LOWPOWER;
	itsdk_sigfox_config.payload_encryption = ITSDK_SIGFOX_ENCRYPTED;
}



// Reset the system clock to default
void ST_MCU_API_SetSysClock(void) {
	SystemClock_Config();
}

/*
void setGpioLowPower(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
  
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_0) & (~GPIO_PIN_13);   // IRQ
#if defined(FOR_ALL) || defined(FOR_ARIB)
  GPIO_InitStructure.Pin &= (~GPIO_PIN_7);   // TCXO ENABLE
#endif
  
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_8) & (~GPIO_PIN_1) & (~GPIO_PIN_9);// SDN , CS S2-LP and CS E2PROM
  
#if defined(USE_STM32L0XX_NUCLEO) || defined(USE_STM32F0XX_NUCLEO) || defined(USE_STM32F4XX_NUCLEO)
  GPIO_InitStructure.Pin &= (~GPIO_PIN_2) & (~GPIO_PIN_3);
#endif
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_4);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //  __GPIOD_CLK_DISABLE();__GPIOB_CLK_DISABLE();
}
*/

/*
* @brief  Restore GPIOs.
* @param  None
* @retval None
*/
/*static */
//void setGpioRestore(void)
//{
//  STM32_GPIO_CLK_ENABLE();
//  __GPIOA_CLK_ENABLE();__GPIOB_CLK_ENABLE();  __GPIOC_CLK_ENABLE();__GPIOD_CLK_ENABLE();
//
//  GPIO_InitTypeDef GPIO_InitStructure;
//  GPIO_InitStructure.Mode =    GPIO_MODE_AF_PP;
//  GPIO_InitStructure.Pull      = GPIO_PULLUP;
//  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
//  GPIO_InitStructure.Alternate= EEPROM_SPI_PERIPH_MISO_AF;
//  GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_6;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//  GPIO_InitStructure.Mode =    GPIO_MODE_AF_PP;
//  GPIO_InitStructure.Pull      = GPIO_PULLUP;
//  GPIO_InitStructure.Alternate= EEPROM_SPI_PERIPH_MISO_AF;
//  GPIO_InitStructure.Pin = GPIO_PIN_3;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
//}

/*
* @brief  Wait for interrupt.
* @param  None
* @retval None
*/
void ST_MCU_API_WaitForInterrupt(void)
{
	// Low power mode is manage outside of the driver
	// (by the way, there is no reason to have a drive to decide a such action

	// @TODO
	// The ST_RF_API_S2LP_IRQ_CB(); should be called from the IRQ handler
	// The S2LP Irq should wake up the MCU
	log_info("Request Entering Low Power Mode\r\n");


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
//  if(s2lp_irq_raised)
//  {
//    ST_RF_API_S2LP_IRQ_CB();
//    s2lp_irq_raised=0;
//  }
}

/*
* @brief  Configure RTC clock.
* @param  None
* @retval None
*/
//static void Configure_RTC_Clock(void)
//{
//  RCC_OscInitTypeDef        RCC_OscInitStruct;
//  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
//
//#ifndef USE_STM32L0XX_NUCLEO
//  __HAL_RCC_PWR_CLK_ENABLE();
//#endif
//  HAL_PWR_EnableBkUpAccess();
//
//  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
//  HAL_RCC_OscConfig(&RCC_OscInitStruct);
//
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
//  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
//  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
//
//  /* Enable RTC Clock */
//  __HAL_RCC_RTC_ENABLE();
//
//  HAL_NVIC_SetPriority(STM32_RTC_IRQn, 0x01, 0);
//  HAL_NVIC_EnableIRQ(STM32_RTC_IRQn);
//}

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
* @brief  EXTI callback.
* @param  None
* @retval None
*/
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  if(GPIO_Pin==GPIO_PIN_0)
//  {
//    if (ST_RF_API_Get_Continuous_TX_Flag()==0) /*It Works as for the previous version (SDK 1.3.0) */
//      s2lp_irq_raised=1;
//    else
//      ST_RF_API_S2LP_IRQ_CB(); /*If the CBPSK is implemented trigger TX State Machine */
//
//    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
//  }
//  else
//  {
//    Appli_Exti_CB(GPIO_Pin);
//  }
//}

/*
* @brief  RTC interrupt handler.
* @param  None
* @retval None
*/
//void STM32_RTC_IRQHandler(void)
//{
//  Configure_RTC_Clock();
//
//  PRINTF("*** RTC_IRQHandler IN\n\r");
//
//  HAL_RTCEx_WakeUpTimerIRQHandler(&RtcHandler);
//  HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandler);
//
//  if(next_rtc_wakeup==0)
//  {
//    rtc_irq=1;
//    rtc_in_use=0;
//    if(notify_end)
//    {
//      if(rtc_in_use_for_cs)
//      {
//	rtc_in_use_for_cs=0;
//	ST_RF_API_Timer_Channel_Clear_CB();
//      }
//      else
//      {
//	ST_RF_API_Timer_CB(TIMER_STOP);
//      }
//    }
//  }
//  else
//  {
//    MCU_API_timer_start(next_rtc_wakeup);
//    n_intermediate_tim_irq++;
//  }
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
static void priv_ST_MCU_API_delay(uint32_t delay_ms)
{
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
  /* Let the retriever encrypts the requested buffer using the ID_KEY_RETRIEVER function.
  The retriever knows the KEY of this node. */
  
  enc_utils_encrypt(encrypted_data, data_to_encrypt, aes_block_len, key, use_key);
  
  
  return SFX_ERR_NONE;
}

/*
* @brief  Get NVM memory.
* @param  read_data : Pointer to the buffer    
* @retval None
*/
sfx_u8 MCU_API_get_nv_mem(sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE])
{		

#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640

	eeprom_m95640_read(
			&ITSDK_DRIVERS_M95640_SPI,
			ITSDK_SIGFOX_NVM_BASEADDR,
			SFX_NVMEM_BLOCK_SIZE,
			(uint8_t *)read_data
	);

	log_info("Read from Eeprom (mcu_api.c):");
	for (int i=0; i< SFX_NVMEM_BLOCK_SIZE ; i++) {
		log_info("0x%X, ",read_data[i]);
	}
	log_info("\r\n");
	return 0;	// success

#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
	#error "__SFX_NVM_LOCALEPROM - not yet implemented"
#endif

}

/*
* @brief  Set NVM memory.
* @param  data_to_write : Pointer to the buffer to write   
* @retval None
*/
//sfx_u8 MCU_API_set_nv_mem(sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE])
//{
//
//#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640
//
//	log_info("Write into Eeprom (mcu_api.c):");
//	for (int i=0; i< SFX_NVMEM_BLOCK_SIZE ; i++) {
//		log_info("0x%X, ",data_to_write[i]);
//	}
//	log_info("\r\n");
//
//	eeprom_m95640_write(
//			&ITSDK_DRIVERS_M95640_SPI,
//			ITSDK_SIGFOX_NVM_BASEADDR,
//			SFX_NVMEM_BLOCK_SIZE,
//			(uint8_t *)data_to_write
//	);
//
//	return 0;	// success
//
//#elif ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_LOCALEPROM
//	#error "__SFX_NVM_LOCALEPROM - not yet implemented"
//#endif
//
//}

/*
* @brief  Timer start carrier sense.
* @param  time_duration_in_ms :TIme duation in ms  
* @retval None
*/
//sfx_u8 MCU_API_timer_start_carrier_sense(sfx_u16 time_duration_in_ms)
//{
//  uint32_t rtc_wup_tick, next_rtc_wakeup_tick;
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
//  return SFX_ERR_NONE;
//}

/*
* @brief  Timer start.
* @param  time_duration_in_ms :TIme duation in ms  
* @retval None
*/
//sfx_u8 MCU_API_timer_start(sfx_u32 time_duration_in_s)
//{
//
//  ST_RF_API_Timer_CB(TIMER_START); /* To notify the rf_api layer */
//  rtc_irq=0;
//
//  uint32_t rtc_wup_tick, next_rtc_wakeup_tick;
//
//  PRINTF("MCU_API_timer_start IN %d\n\r", time_duration_in_s);
//
//  Configure_RTC_Clock();
//  notify_end=1;
//  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RtcHandler, RTC_FLAG_WUTF);
//  __HAL_RTC_CLEAR_FLAG(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);
//  n_intermediate_tim_irq=0;
//  rtc_in_use=1;
//
//  rtc_wup_tick = (time_duration_in_s)*rtc_presc;
//
//  if(rtc_wup_tick>65535)
//  {
//    next_rtc_wakeup_tick=(rtc_wup_tick)-65535;
//    rtc_wup_tick=65535;
//  }
//  else
//  {
//    next_rtc_wakeup_tick=0;
//  }
//
//  HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandler,rtc_wup_tick,RTC_WAKEUPCLOCK_RTCCLK_DIV16);
//
//  next_rtc_wakeup=next_rtc_wakeup_tick/rtc_presc;
//
//  PRINTF("MCU_API_timer_start OUT %d\n\r", next_rtc_wakeup);
//  return SFX_ERR_NONE;
//}

/*
* @brief  Timer stop.
* @param  None 
* @retval sfx_u8 : status
*/
//sfx_u8 MCU_API_timer_stop(void)
//{
//  PRINTF("MCU_API_timer_stop IN\n\r");
//  HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandler);
//  rtc_in_use=0;
//  PRINTF("MCU_API_timer_stop OUT\n\r");
//  return SFX_ERR_NONE;
//}

/*
* @brief  Timer stop carrier sense.
* @param  None 
* @retval sfx_u8 : status
*/
//sfx_u8 MCU_API_timer_stop_carrier_sense(void)
//{
//  PRINTF("MCU_API_timer_stop_carrier_sense IN\n\r");
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
//  return SFX_ERR_NONE;
//
//}

/*
* @brief  Timer wait for end.
* @param  None 
* @retval sfx_u8 : status
*/
//sfx_u8 MCU_API_timer_wait_for_end(void)
//{
//  PRINTF("MCU_API_timer_wait_for_end IN\n\r");
//
//  while(!rtc_irq)/*(!(next_rtc_wakeup==0 || rtc_irq==1)) */
//  {
//    ST_MCU_API_WaitForInterrupt();
//  }
//  rtc_irq=0;
//  PRINTF("MCU_API_timer_wait_for_end OUT\n\r");
//  return SFX_ERR_NONE;
//}



/*
* @brief  Reports test results.
* @param  status : status 
* @param  rssi   : RSSI 
* @retval sfx_u8 : status
*/
sfx_u8 MCU_API_report_test_result(sfx_bool status, sfx_s16 rssi)
{    
  
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

#if ITSDK_SIGFOX_NVM_SOURCE == __SFX_NVM_M95640

	// from Sigfox Retriever library
	enc_utils_get_id(dev_id);
   (*payload_encryption_enabled) = itsdk_sigfox_config.payload_encryption;

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
	itsdk_sigfox_config.low_power_flag=low_power_flag;
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
  uint16_t gpioPin;
  uint8_t  gpioBank;

  log_info("S2LP Configuring Gpio %d as %d,%d\r\n",pin,new_state,trigger);

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
    gpioPin = ITSDK_S2LP_GPIO1_PIN;
    gpioBank = ITSDK_S2LP_GPIO1_BANK;
    break;
  case 3:
    gpioPin = ITSDK_S2LP_GPIO1_PIN;
    gpioBank = ITSDK_S2LP_GPIO1_BANK;
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

/*
* @brief  MCU API shutdown.
* @param  value : TRUE/FALSE
* @retval none
*/
void ST_MCU_API_Shutdown(sfx_u8 value)
{
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
	itsdk_sigfox_config.payload_encryption = ePayload;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
