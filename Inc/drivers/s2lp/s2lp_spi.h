/* ==========================================================
 * s2lp_spi.h - Header for S2LP spi part
 * Project : Disk91 SDK
 * ----------------------------------------------------------
 * Created on: 4 Nov. 2018
 *     Author: Paul Pinault aka Disk91
 * ----------------------------------------------------------
 * Copyright (C) 2018 Disk91
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
#include <it_sdk/wrappers.h>

#define S2LP_SPI_DELAY		2	// 2 ms delay between chip select & SPI access

typedef enum {
	S2LP_READ_REGISTER	= 0x00,
	S2LP_WRITE_REGISTER = 0x01
} S2LP_ACCESS;


// ------------------------------------------------------------------------------
// S2LP REGISTERS
// ------------------------------------------------------------------------------

#define S2LP_REG_GPIO0_CONF			0x00
#define S2LP_REG_GPIO1_CONF			0x01
#define S2LP_REG_GPIO2_CONF			0x02
#define S2LP_REG_GPIO3_CONF			0x03
#define S2LP_REG_SYNT3				0x05
#define S2LP_REG_SYNT2				0x06
#define S2LP_REG_SYNT1				0x07
#define S2LP_REG_SYNT0				0x08
#define S2LP_REG_IF_OFFSET_ANA		0x09
#define S2LP_REG_IF_OFFSET_DIG		0x0A
#define S2LP_REG_CHSPACE			0x0C
#define S2LP_REG_CHNUM				0x0D
#define S2LP_REG_MOD4				0x0E
#define S2LP_REG_MOD3				0x0F
#define S2LP_REG_MOD2				0x10
#define S2LP_REG_MOD1				0x11
#define S2LP_REG_MOD0				0x12
#define S2LP_REG_CHFLT				0x13
#define S2LP_REG_AFC2				0x14
#define S2LP_REG_AFC1				0x15
#define S2LP_REG_AFC0				0x16
#define S2LP_REG_RSSI_FLT			0x17
#define S2LP_REG_RSSI_TH			0x18
#define S2LP_REG_AGCCTRL4			0x1A
#define S2LP_REG_AGCCTRL3			0x1B
#define S2LP_REG_AGCCTRL2			0x1C
#define S2LP_REG_AGCCTRL1			0x1D
#define S2LP_REG_AGCCTRL0			0x1E
#define S2LP_REG_ANT_SELECT_CONF	0x1F
#define S2LP_REG_CLOCKREC2			0x20
#define S2LP_REG_CLOCKREC1			0x21
#define S2LP_REG_PCKTCTRL6			0x2B
#define S2LP_REG_PCKTCTRL5			0x2C
#define S2LP_REG_PCKTCTRL4			0x2D
#define S2LP_REG_PCKTCTRL3			0x2E
#define S2LP_REG_PCKTCTRL2			0x2F
#define S2LP_REG_PCKTCTRL1			0x30
#define S2LP_REG_PCKTLEN1			0x31
#define S2LP_REG_PCKTLEN0			0x32
#define S2LP_REG_SYNC3				0x33
#define S2LP_REG_SYNC2				0x34
#define S2LP_REG_SYNC1				0x35
#define S2LP_REG_SYNC0				0x36
#define S2LP_REG_QI					0x37
#define S2LP_REG_PCKT_PSTMBL		0x38
#define S2LP_REG_PROTOCOL2			0x39
#define S2LP_REG_PROTOCOL1			0x3A
#define S2LP_REG_PROTOCOL0			0x3B
#define S2LP_REG_FIFO_CONFIG3		0x3C
#define S2LP_REG_FIFO_CONFIG2		0x3D
#define S2LP_REG_FIFO_CONFIG1		0x3E
#define S2LP_REG_FIFO_CONFIG0		0x3F
#define S2LP_REG_PCKT_FLT_OPTIONS	0x40
#define S2LP_REG_PCKT_FLT_GOALS4	0x41
#define S2LP_REG_PCKT_FLT_GOALS3	0x42
#define S2LP_REG_PCKT_FLT_GOALS2	0x43
#define S2LP_REG_PCKT_FLT_GOALS1	0x44
#define S2LP_REG_PCKT_FLT_GOALS0	0x45
#define S2LP_REG_TIMERS5			0x46
#define S2LP_REG_TIMERS4			0x47
#define S2LP_REG_TIMERS3			0x48
#define S2LP_REG_TIMERS2			0x49
#define S2LP_REG_TIMERS1			0x4A
#define S2LP_REG_TIMERS0			0x4B
#define S2LP_REG_CSMA_CONF3			0x4C
#define S2LP_REG_CSMA_CONF2			0x4D
#define S2LP_REG_CSMA_CONF1			0x4E
#define S2LP_REG_CSMA_CONF0			0x4F
#define S2LP_REG_IRQ_MASK3			0x50
#define S2LP_REG_IRQ_MASK2			0x51
#define S2LP_REG_IRQ_MASK1			0x52
#define S2LP_REG_IRQ_MASK0			0x53
#define S2LP_REG_FAST_RX_TIMER		0x54
#define S2LP_REG_PA_POWER8			0x5A
#define S2LP_REG_PA_POWER7			0x5B
#define S2LP_REG_PA_POWER6			0x5C
#define S2LP_REG_PA_POWER5			0x5D
#define S2LP_REG_PA_POWER4			0x5E
#define S2LP_REG_PA_POWER3			0x5F
#define S2LP_REG_PA_POWER2			0x60
#define S2LP_REG_PA_POWER1			0x61
#define S2LP_REG_PA_POWER0			0x62
#define S2LP_REG_PA_CONFIG1			0x63
#define S2LP_REG_PA_CONFIG0			0x64
#define S2LP_REG_SYNTH_CONFIG2		0x65
#define S2LP_REG_VCO_CONFIG			0x68
#define S2LP_REG_VCO_CALIBR_IN2		0x69
#define S2LP_REG_VCO_CALIBR_IN1		0x6A
#define S2LP_REG_VCO_CALIBR_IN0		0x6B
#define S2LP_REG_XO_RCO_CONF1		0x6C
#define S2LP_REG_XO_RCO_CONF0		0x6D
#define S2LP_REG_RCO_CALIBR_CONF3	0x6E
#define S2LP_REG_RCO_CALIBR_CONF2	0x6F
#define S2LP_REG_PM_CONF4			0x75
#define S2LP_REG_PM_CONF3			0x76
#define S2LP_REG_PM_CONF2			0x77
#define S2LP_REG_PM_CONF1			0x78
#define S2LP_REG_PM_CONF0			0x79
#define S2LP_REG_MC_STATE1			0x8D
#define S2LP_REG_MC_STATE0			0x8E
#define S2LP_REG_TX_FIFO_STATUS		0x8F
#define S2LP_REG_RX_FIFO_STATUS		0x90
#define S2LP_REG_RCO_CALIBR_OUT4	0x94
#define S2LP_REG_RCO_CALIBR_OUT3	0x95
#define S2LP_REG_VCO_CALIBR_OUT1	0x99
#define S2LP_REG_VCO_CALIBROUT0		0x9A
#define S2LP_REG_TX_PCKT_INFO		0x9C
#define S2LP_REG_RX_PCKT_INFO		0x9D
#define S2LP_REG_AFC_CORR			0x9E
#define S2LP_REG_LINK_QUALIF2		0x9F
#define S2LP_REG_LINK_QUALIF1		0xA0
#define S2LP_REG_RSSI_LEVEL			0xA2
#define S2LP_REG_RX_PCKT_LEN1		0xA4
#define S2LP_REG_RX_PCKT_LEN0		0xA5
#define S2LP_REG_CRC_FIELD3			0xA6
#define S2LP_REG_CRC_FIELD2			0xA7
#define S2LP_REG_CRC_FIELD1			0xA8
#define S2LP_REG_CRC_FIELD0			0xA9
#define S2LP_REG_RX_ADDRE_FIELD1	0xAA
#define S2LP_REG_RX_ADDRE_FIELD0	0xAB
#define S2LP_REG_RSSI_LEVEL_RUN		0xEF
#define S2LP_REG_DEVICE_INFO1		0xF0		// Part Number
#define S2LP_REG_DEVICE_INFO0		0xF1		// Version
#define S2LP_REG_IRQ_STATUS3		0xFA
#define S2LP_REG_IRQ_STATUS2		0xFB
#define S2LP_REG_IRQ_STATUS1		0xFC
#define S2LP_REG_IRQ_STATUS0		0xFD


#define S2LP_VERSION_2_0			0x81
#define S2LP_VERSION_2_1			0x91
#define S2LP_VERSION_3_0			0xC1


#define WRITE_HEADER        0x00
#define READ_HEADER         0x01
#define COMMAND_HEADER      0x80


// << COPYRIGHT(c) 2018 STMicroelectronics >>

// S2-LP_Spi_config_Headers
#define LINEAR_FIFO_ADDRESS   0xFF                                // Linear FIFO address



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
  uint8_t XO_ON:1;           // XO is operating state (MC_STATE0)
  S2LPState MC_STATE: 7;     // The state of the Main Controller of S2LP @ref S2LPState  (MC_STATE0)
  uint8_t ERROR_LOCK: 1;     // RCO calibration error (MC_STATE1)
  uint8_t RX_FIFO_EMPTY: 1;  // RX FIFO is empty (MC_STATE1)
  uint8_t TX_FIFO_FULL: 1;   // TX FIFO is full (MC_STATE1)
  uint8_t ANT_SELECT: 1;     // Currently selected antenna (MC_STATE1)
  uint8_t RCCAL_OK: 1;       // RCO successfully terminated (MC_STATE1)
  uint8_t : 3;               // This 3 bits field are reserved and equal to 2 (MC_STATE1)
} s_S2LPStatus;

typedef s_S2LPStatus S2LP_SPI_StatusBytes;
// End of >> COPYRIGHT(c) 2018 STMicroelectronics <<


S2LP_SPI_StatusBytes s2lp_spi_writeRegisters(
		ITSDK_SPI_HANDLER_TYPE * spi,
		uint8_t cRegAddress,
        uint8_t cNbBytes,
        uint8_t* pcBuffer
);

S2LP_SPI_StatusBytes s2lp_spi_readRegisters(
		ITSDK_SPI_HANDLER_TYPE * spi,
		uint8_t cRegAddress,
        uint8_t cNbBytes,
        uint8_t* pcBuffer
);

S2LP_SPI_StatusBytes s2lp_spi_accessRaw(
		ITSDK_SPI_HANDLER_TYPE * spi,
		uint8_t*  pInBuffer,
        uint8_t*  pOutBuffer,
        uint8_t   cNbBytes
);

void s2lp_spi_setCsLow();
void s2lp_spi_setCsHigh();

#endif /* IT_SDK_DRIVERS_S2LP_SPI_H_ */

