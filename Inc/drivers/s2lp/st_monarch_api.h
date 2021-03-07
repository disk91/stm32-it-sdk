
/*!
 * \file st_monarch_api.h
 * \brief Sigfox manufacturer functions
 * \author  AMG - RF Application team
 * \version 2.3.1
 * \date April 25, 2018
 * \copyright COPYRIGHT 2018 STMicroelectronics
 *
 * This file defines the manufacturer's RF functions to be implemented
 * for library usage.
 */
#ifdef MONARCH_GPIO_SAMPLING
	void ST_MONARCH_API_Timer_CB(void);
	void ST_MONARCH_API_SetSamplingGPIO(sfx_u8 pin);
#else
	void ST_MONARCH_API_AFTHR_GPIO_CB(void);
#endif
