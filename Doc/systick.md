## SysTick configuration (STM32)
Systick timer is used to maintain the time duration between sleeps where the RTC is used to resycnhronize time. The systick interrupt is used for this purpose.

The configuration of the systick timer must be 1ms for a 16MHz and adapted regarding the frequency to maintain this ratio ( ok, this need to be improved later...)

The systick is calibrated when the **ITSDK_WITH_CLK_ADJUST** define is set. The clock used for the calibration depends on the setting selected for **ITSDK_CLK_BEST_SOURCE** the source can be the master clock or the RTC clock.