## RTC features

RTC is used to count time and maintain current time even with sleep periods related to low power mode.
The RTC can be clocked by the internal oscillator, usually not precise or by an external christal to provide a better accurancy.

### RTC in it_sdk framework

There is no direct use of the RTC at user level. The RTC interactions are made inside the itsdk layer with the MCU specific layer. Final program should not directly interract with RTC.

The Time functions using the RTC are located in __it_sdk/time.h__ and __it_sdk/lowpower.h__

The RTC is enabled with the following defines:
* ITSDK_WITH_RTC => __RTC_ENABLED / __RTC_DISABLED as a global setting. RTC is needed for low power feature
* ITSDK_RTC_CLKFREQ => indicates the RTC clock frequency in use

The RTC can be calibrated for a better acurancy. Depending on the clock configuration the most accurate clk source can be the RTC, in this case the framework will calibrate the main clk based on RTC. It can be the main clock, in this cas the RTC will be calibrated with the main clk signal.
* ITSDK_WITH_CLK_ADJUST => __ENABLE / __DISABLE will activate calibration code creating a dependency with timer | RTC features
* ITSDK_CLK_BEST_SOURCE => __CLK_BEST_SRC_RTC / __CLK_BEST_SRC_CLK is indicating the CLK source to be used as the most accurate to adjust the second one.
* ITSDK_CLK_CORRECTION => defines a manual ratio to correct the RTC clk. The value is in o/oo

The RTC is on of the way used to wake-up the MCU during deep-sleep. This is activated by adding the  **__LOWPWR_MODE_WAKE_RTC** flag in the **ITSDK_LOWPOWER_MOD** define.
The RTC wake up frequency is indicated with the **ITSDK_LOWPOWER_RTC_MS** define

### STM32 implementation

The RTC need to be activated with the following configuration:
* ClkSource activated
* Calendar activated
* Internal WakeUp activated
* The clk configuration can be LSI or LSE based on your configuration
* The RTC/NVIC Interrupt is activated.

The RTC predivider are configured to provide a 1Hz signal to the RTC according to the formula:
1Hz= (RTCCLK)/((1+Async_previv)*(1+Sync_prediv))
* for 37KHz (LSI) Asycn_prediv 124 & Sync_prediv 295
* for 32.768KHz (LSE) Async_prediv 127 & Sync_prediv 255

The wake-up unit for the different sleep mode is directly connected to RTCLK signal with a predivision. The setting have to be RTCLK/16.


Adjustment requires Timer TIM21 to be activated.

 