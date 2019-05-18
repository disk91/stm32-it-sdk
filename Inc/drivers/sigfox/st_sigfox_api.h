/******************************************************************************
 * @file    st_sigfox_api.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   add-on above sigfox_api.h
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other 
 *    contributors to this software may be used to endorse or promote products 
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this 
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under 
 *    this license is void and will automatically terminate your rights under 
 *    this license. 
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ST_SIGFOX_API_H__
#define __ST_SIGFOX_API_H__



#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <drivers/sigfox/sigfox_types.h>
#include <drivers/sigfox/sigfox_api.h>

/* Exported types ------------------------------------------------------------*/  

typedef enum {
  RC1_ID =1,
  RC2_ID ,
  RC3C_ID ,
  RC4_ID ,
} e_sfx_rc_id_t;

typedef struct {
  e_sfx_rc_id_t id;
  sfx_rc_t param;
} st_sfx_rc_t;

/* Exported constants --------------------------------------------------------*/

#define ST_RC1 {RC1_ID, RC1}
#define ST_RC2 {RC2_ID, RC2}
#define ST_RC3C {RC3C_ID, RC3C}
#define ST_RC4 {RC4_ID, RC4}

#ifdef __cplusplus
}
#endif

#endif /* __ST_SIGFOX_API_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
