/* ==========================================================
 * s2lp_spi.h - Header for S2LP spi part
 * Project : IngeniousThings SDK
 * ----------------------------------------------------------
 * Created on: 4 Nov. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018  IngeniousThings and Disk91
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
 * Some peaces of that code directly comes from ST Libraries
 * and identified with << COPYRIGHT(c) 2018 STMicroelectronics >>
 * ==========================================================
 */

#ifndef IT_SDK_DRIVERS_S2LP_SPI_H_
#define IT_SDK_DRIVERS_S2LP_SPI_H_

#include <it_sdk/config.h>
#include <stm32l_sdk/spi/spi.h>

#define S2LP_SPI_DELAY		2	// 2 ms delay between chip select & SPI access

// << COPYRIGHT(c) 2018 STMicroelectronics >>

// S2-LP_Spi_config_Headers
#define HEADER_WRITE_MASK     0x00                                // Write mask for header byte
#define HEADER_READ_MASK      0x01                                // Read mask for header byte
#define HEADER_ADDRESS_MASK   0x00                                // Address mask for header byte
#define HEADER_COMMAND_MASK   0x80                                // Command mask for header byte

#define LINEAR_FIFO_ADDRESS   0xFF                                // Linear FIFO address


#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)
#define WRITE_HEADER        BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK)
#define READ_HEADER         BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)
#define COMMAND_HEADER      BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK)


typedef enum {
  MC_STATE_READY             =0x00,  // READY
  MC_STATE_SLEEP_NOFIFO      =0x01,  // SLEEP NO FIFO RETENTION
  MC_STATE_STANDBY           =0x02,  // STANDBY
  MC_STATE_SLEEP             =0x03,  // SLEEP
  MC_STATE_LOCKON            =0x0C,  // LOCKON
  MC_STATE_RX                =0x30,  // RX
  MC_STATE_LOCK_ST           =0x14,  // LOCK_ST
  MC_STATE_TX                =0x5C,  // TX
  MC_STATE_SYNTH_SETUP       =0x50   // SYNTH_SETUP
} S2LPState;


/**
 * @brief S2LP Status. This definition represents the single field of the S2LP
 *        status returned on each SPI transaction, equal also to the MC_STATE registers.
 *        This field-oriented structure allows user to address in simple way the single
 *        field of the S2LP status.
 *        The user shall define a variable of S2LPStatus type to access on S2LP status fields.
 * @note  The fields order in the structure depends on used endianness (little or big
 *        endian). The actual definition is valid ONLY for LITTLE ENDIAN mode. Be sure to
 *        change opportunely the fields order when use a different endianness.
 */
typedef struct {
  uint8_t XO_ON:1;           // XO is operating state
  S2LPState MC_STATE: 7;     // The state of the Main Controller of S2LP @ref S2LPState
  uint8_t ERROR_LOCK: 1;     // RCO calibration error
  uint8_t RX_FIFO_EMPTY: 1;  // RX FIFO is empty
  uint8_t TX_FIFO_FULL: 1;   // TX FIFO is full
  uint8_t ANT_SELECT: 1;     // Currently selected antenna
  uint8_t RCCAL_OK: 1;       // RCO successfully terminated
  uint8_t : 3;               // This 3 bits field are reserved and equal to 2
} S2LPStatus;

typedef S2LPStatus S2LP_SPI_StatusBytes;
// End of >> COPYRIGHT(c) 2018 STMicroelectronics <<


S2LP_SPI_StatusBytes s2lp_spi_writeRegisters(
		__SPI_HANDLER_TYPE * spi,
		uint8_t cRegAddress,
        uint8_t cNbBytes,
        uint8_t* pcBuffer
);

S2LP_SPI_StatusBytes s2lp_spi_readRegisters(
		__SPI_HANDLER_TYPE * spi,
		uint8_t cRegAddress,
        uint8_t cNbBytes,
        uint8_t* pcBuffer
);

#endif /* IT_SDK_DRIVERS_S2LP_SPI_H_ */
