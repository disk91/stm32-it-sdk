## PWM features

PWM driver needs to ne enable in the config.h file, old project needs to get the configuration defines from template.

```
#define ITSDK_WITH_PWM				__ENABLE								// Enable PWM code
#define ITSDK_WITH_PWM_TIMER		__TIMER_1								// Select timer handler to be used __TIMER_NONE for none
``` 

The timer configuration depends on target. For STM32, the CubeMx configuration is made with following elements:
1. Enable expected timer is `enable`
2. Clock Source is `internal`
3. Expected channels are `enable` as PWM generation on Channel X
4. Auto-Reload Preload set to `enable`.
5. Make sure Counter Period is `65536` and Channel Mode is `PWM mode 1`
