/**
* @file    S2LP_FEM.h
* @author  AMG - RF Application team
* @version 1.0.0
* @date    October 1, 2018
* @brief   FEM (Front end module, aka power amplifier) management
* @details This module exports API to manage a FEM possibly present on an
* S2-LP board. The API is intended to provided users hooks to configure the
* FEM in his own board.
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*
* <h2><center>&copy; COPYRIGHT 2018 STMicroelectronics</center></h2>
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_FEM_H__
#define __S2LP_FEM_H__

/**
 * @defgroup S2LP_FEM
 * @brief Management of FEM (if present).
 * @details See the file <i>@ref S2LP_FEM.h</i> for more details.
 * @{
 */

/**
 * @brief  Opcode for FEM management
 */

#define POWER_DBM	12

/*Structure to manage External PA */
typedef enum
{
  FEM_SHUTDOWN     = 0x00,
  FEM_TX_BYPASS    = 0x01,
  FEM_TX     	   = 0x02,
  FEM_RX     	   = 0x03,
} FEM_OperationType;

/*Structure to describe gain parametr */
typedef enum
{
  FEM_GAIN_TX    = 0x00,
  FEM_GAIN_RX    = 0x01,
} FEM_GainType;

/**
* @brief  Initializes the power amplfier control lines.
* @param  None
* @retval None
*/
void FEM_Init(void);

/**
* @brief  Perform operation on FEM, usually controlling the configuration lines via GPIO in the uC.
* If FEM is under control of S2-LP GPIOs, possibly this function can be implemented as dummy function.
* @param  operation: operation requested, if FEM is not present this is implemented by dummy function
* @retval None
*/
void FEM_Operation(FEM_OperationType operation);

/**
* @brief  Return gain for the FEM.
* @retval FEM_GainType
*/
FEM_GainType FEM_GetGain(void);

/**
* @brief  Set gain value for the FEM.
* @param  direction: operation requested, if FEM is not present this is implemented by dummy function
* @retval None
*/
void FEM_SetGain(FEM_GainType direction);

/**
* @}
*/

/**
* @}
*/


#endif
 /******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

