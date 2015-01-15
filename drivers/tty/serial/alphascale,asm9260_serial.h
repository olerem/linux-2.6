/*
 * Copyright (C) 2014 Oleksij Rempel <linux@rempel-privat.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ALPHASCALE_ASM9260_SERIAL_H
#define _ALPHASCALE_ASM9260_SERIAL_H

/* RX ctrl register */
#define ASM9260_HW_CTRL0			0x0000
/* RW. Set to zero for normal operation. */
#define ASM9260_BM_CTRL0_SFTRST			BIT(31)
/*
 * RW. 0 for normal operation; 1 gates all of the block level clocks off for
 * miniminizing AC energy consumption.
 */
#define ASM9260_BM_CTRL0_CLKGATE		BIT(30)
/*
 * RW. Tell the UART to execute the RX DMA Command. The
 * UART will clear this bit at the end of receive execution.
 */
#define ASM9260_BM_CTRL0_RXDMA_RUN		BIT(28)
/* RW. 0 use FIFO for status register; 1 use DMA */
#define ASM9260_BM_CTRL0_RXTO_SOURCE_STATUS	BIT(25)
/*
 * RW. RX TIMEOUT Enable. Valid for FIFO and DMA.
 * Warning: If this bit is set to 0, the RX timeout will not affect receive DMA
 * operation. If this bit is set to 1, a receive timeout will cause the receive
 * DMA logic to terminate by filling the remaining DMA bytes with garbage data.
 */
#define ASM9260_BM_CTRL0_RXTO_ENABLE		BIT(24)
/*
 * RW. Receive Timeout Counter Value: number of 8-bit-time to wait before
 * asserting timeout on the RX input. If the RXFIFO is not empty and the RX
 * input is idle, then the watchdog counter will decrement each bit-time. Note
 * 7-bit-time is added to the programmed value, so a value of zero will set
 * the counter to 7-bit-time, a value of 0x1 gives 15-bit-time and so on. Also
 * note that the counter is reloaded at the end of each frame, so if the frame
 * is 10 bits long and the timeout counter value is zero, then timeout will
 * occur (when FIFO is not empty) even if the RX input is not idle. The default
 * value is 0x3 (31 bit-time).
 */
#define ASM9260_BM_CTRL0_RXTO_MASK		(0xff<<16)
/* TIMEOUT = (100*7+1)*(1/BAUD) */
#define ASM9260_BM_CTRL0_DEFAULT_RXTIMEOUT	(20<<16)
/* RW. Number of bytes to receive. This must be a multiple of 4 */
#define ASM9260_BM_CTRL0_RXDMA_COUNT_MASK	(0xffff<<0)

/* TX ctrl register */
#define ASM9260_HW_CTRL1			0x0010
/*
 * RW. Tell the UART to execute the TX DMA Command. The
 * UART will clear this bit at the end of transmit execution.
 */
#define ASM9260_BM_CTRL1_TXDMA_RUN		BIT(28)
/* RW. Number of bytes to transmit. */
#define ASM9260_BM_CTRL1_TXDMA_COUNT_MASK	(0xffff << 0)

#define ASM9260_HW_CTRL2			0x0020
/*
 * RW. Receive dma will terminate on error. (Cmd_end signal may not be asserted
 * when this occurs.)
 */
#define ASM9260_BM_CTRL2_DMAONERROR		BIT(26)
/*
 * RW. Transmit DMA Enable. Data Register can be loaded with up to 4 bytes per
 * write. TXFIFO must be enabled in TXDMA mode.
 */
#define ASM9260_BM_CTRL2_TXDMAE			BIT(25)
/*
 * RW. Receive DMA Enable. Data Register can be contain up to 4 bytes per read.
 * RXFIFO must be enabled in RXDMA mode.
 */
#define ASM9260_BM_CTRL2_RXDMAE			BIT(24)
/*
 * RW. Receive Interrupt FIFO Level Select.
 * The trigger points for the receive interrupt are as follows:
 * ONE_EIGHTHS = 0x0 Trigger on FIFO full to at least 2 of 16 entries.
 * ONE_QUARTER = 0x1 Trigger on FIFO full to at least 4 of 16 entries.
 * ONE_HALF = 0x2 Trigger on FIFO full to at least 8 of 16 entries.
 * THREE_QUARTERS = 0x3 Trigger on FIFO full to at least 12 of 16 entries.
 * SEVEN_EIGHTHS = 0x4 Trigger on FIFO full to at least 14 of 16 entries.
 */
#define ASM9260_BM_CTRL2_RXIFLSEL		(7<<20)
#define ASM9260_BM_CTRL2_DEFAULT_RXIFLSEL	(3<<20)
/* RW. Same as RXIFLSEL */
#define ASM9260_BM_CTRL2_TXIFLSEL		(7<<16)
#define ASM9260_BM_CTRL2_DEFAULT_TXIFLSEL	(2<<16)
/* RW. CTS Enable */
#define ASM9260_BM_CTRL2_CTSE			BIT(15)
/* RW. RTS Enable */
#define ASM9260_BM_CTRL2_RTSE			BIT(14)
/*
 * RW. Manually trigger RTS. Works only if ASM9260_BM_CTRL2_RTSE = 0.
 * When this bit is 1, the output is 0.
 */
#define ASM9260_BM_CTRL2_RTS			BIT(11)
/* RW. Set DTR. When this bit is 1, the output is 0. */
#define ASM9260_BM_CTRL2_DTR			BIT(10)
/* RW. RX Enable */
#define ASM9260_BM_CTRL2_RXE			BIT(9)
/* RW. TX Enable */
#define ASM9260_BM_CTRL2_TXE			BIT(8)
/* RW. Loop Back Enable */
#define ASM9260_BM_CTRL2_LBE			BIT(7)
#define ASM9260_BM_CTRL2_PORT_ENABLE		BIT(0)

#define ASM9260_HW_LINECTRL			0x0030
#define ASM9260_BM_LCTRL_BAUD_DIVINT		(0xFFFF<<16)
#define ASM9260_BM_LCTRL_BAUD_DIVFRA		(0x3F<<8)
/*
 * RW. Stick Parity Select. When bits 1, 2, and 7 of this register are set, the
 * parity bit is transmitted and checked as a 0. When bits 1 and 7 are set,
 * and bit 2 is 0, the parity bit is transmitted and checked as a 1. When this
 * bit is cleared stick parity is disabled.
 */
#define ASM9260_BM_LCTRL_SPS			BIT(7)
/* RW. Word length */
#define ASM9260_BM_LCTRL_WLEN			(3<<5)
#define ASM9260_BM_LCTRL_CHRL_5			(0<<5)
#define ASM9260_BM_LCTRL_CHRL_6			(1<<5)
#define ASM9260_BM_LCTRL_CHRL_7			(2<<5)
#define ASM9260_BM_LCTRL_CHRL_8			(3<<5)
/*
 * RW. Enable FIFOs. If this bit is set to 1, transmit and receive FIFO buffers
 * are enabled (FIFO mode). When cleared to 0, the FIFOs are disabled (character
 * mode); that is, the FIFOs become 1-byte-deep holding registers.
 */
#define ASM9260_BM_LCTRL_FEN			BIT(4)
/*
 * RW. Two Stop Bits Select. If this bit is set to 1, two stop bits are
 * transmitted at the end of the frame. The receive logic does not check for
 * two stop bits being received.
 */
#define ASM9260_BM_LCTRL_STP2			BIT(3)
#define ASM9260_BM_LCTRL_NBSTOP_1		(0<<3)
#define ASM9260_BM_LCTRL_NBSTOP_2		(1<<3)
/* RW. Even Parity Select. If disabled, then odd parity is performed. */
#define ASM9260_BM_LCTRL_EPS			BIT(2)
/* Parity Enable. */
#define ASM9260_BM_LCTRL_PEN			BIT(1)
#define ASM9260_BM_LCTRL_PAR_MARK		((3<<1) | (1<<7))
#define ASM9260_BM_LCTRL_PAR_SPACE		((1<<1) | (1<<7))
#define ASM9260_BM_LCTRL_PAR_ODD		((1<<1) | (0<<7))
#define ASM9260_BM_LCTRL_PAR_EVEN		((3<<1) | (0<<7))
#define ASM9260_BM_LCTRL_PAR_NONE		(0<<1)
/*
 * RW. Send Break. If this bit is set to 1, a low-level is continually output on
 * the UARTTXD output, after completing transmission of the current character.
 * For the proper execution of the break command, the software must set this bit
 * for at least two complete frames. For normal use, this bit must be cleared
 * to 0.
 */
#define ASM9260_BM_LCTRL_BREAK			BIT(0)

/*
 * Interrupt register.
 * contains the interrupt enables and the interrupt status bits
 */
#define ASM9260_HW_INTR				0x0040
/* Tx FIFO EMPTY Raw Interrupt enable */
#define ASM9260_BM_INTR_TFEIEN			BIT(27)
/* Overrun Error Interrupt Enable. */
#define ASM9260_BM_INTR_OEIEN			BIT(26)
/* Break Error Interrupt Enable. */
#define ASM9260_BM_INTR_BEIEN			BIT(25)
/* Parity Error Interrupt Enable. */
#define ASM9260_BM_INTR_PEIEN			BIT(24)
/* Framing Error Interrupt Enable. */
#define ASM9260_BM_INTR_FEIEN			BIT(23)
/*
 * RW. Receive Timeout Interrupt Enable.
 * If not set and FIFO is enabled, then RX will be triggered only
 * if FIFO is full.
 */
#define ASM9260_BM_INTR_RTIEN			BIT(22)
/* Transmit Interrupt Enable. */
#define ASM9260_BM_INTR_TXIEN			BIT(21)
/* Receive Interrupt Enable. */
#define ASM9260_BM_INTR_RXIEN			BIT(20)
/* nUARTDSR Modem Interrupt Enable. */
#define ASM9260_BM_INTR_DSRMIEN			BIT(19)
/* nUARTDCD Modem Interrupt Enable. */
#define ASM9260_BM_INTR_DCDMIEN			BIT(18)
/* nUARTCTS Modem Interrupt Enable. */
#define ASM9260_BM_INTR_CTSMIEN			BIT(17)
/* nUARTRI Modem Interrupt Enable. */
#define ASM9260_BM_INTR_RIMIEN			BIT(16)
/* Auto-Boud Timeout */
#define ASM9260_BM_INTR_ABTO			BIT(13)
#define ASM9260_BM_INTR_ABEO			BIT(12)
/* Tx FIFO EMPTY Raw Interrupt state */
#define ASM9260_BM_INTR_TFEIS			BIT(11)
/* Overrun Error */
#define ASM9260_BM_INTR_OEIS			BIT(10)
/* Break Error */
#define ASM9260_BM_INTR_BEIS			BIT(9)
/* Parity Error */
#define ASM9260_BM_INTR_PEIS			BIT(8)
/* Framing Error */
#define ASM9260_BM_INTR_FEIS			BIT(7)
/* Receive Timeout */
#define ASM9260_BM_INTR_RTIS			BIT(6)
/* Transmit done */
#define ASM9260_BM_INTR_TXIS			BIT(5)
/* Receive done */
#define ASM9260_BM_INTR_RXIS			BIT(4)
#define ASM9260_BM_INTR_DSRMIS			BIT(3)
#define ASM9260_BM_INTR_DCDMIS			BIT(2)
#define ASM9260_BM_INTR_CTSMIS			BIT(1)
#define ASM9260_BM_INTR_RIMIS			BIT(0)
#define ASM9260_BM_INTR_DEF_MASK	(ASM9260_BM_INTR_RXIEN \
		| ASM9260_BM_INTR_TXIEN | ASM9260_BM_INTR_RTIEN \
		| ASM9260_BM_INTR_FEIEN | ASM9260_BM_INTR_PEIEN \
		| ASM9260_BM_INTR_BEIEN | ASM9260_BM_INTR_OEIEN)
#define ASM9260_BM_INTR_DEF_IS_MASK		(BM_INTR_DEF_MASK >> 16)
#define ASM9260_BM_INTR_EN_MASK			(0x3fff0000)
#define ASM9260_BM_INTR_IS_MASK			(0x00003fff)

/*
 * RW. In DMA mode, up to 4 Received/Transmit characters can be accessed at a
 * time. In PIO mode, only one character can be accessed at a time. The status
 * register contains the receive data flags and valid bits.
 */
#define ASM9260_HW_DATA				0x0050

#define ASM9260_HW_STAT				0x0060
/* RO. If 1, UARTAPP is present in this product. */
#define ASM9260_BM_STAT_PRESENT			BIT(31)
/* RO. If 1, HISPEED is present in this product. */
#define ASM9260_BM_STAT_HISPEED			BIT(30)
/* RO. UART Busy. */
#define ASM9260_BM_STAT_BUSY			BIT(29)
/* RO. Clear To Send. */
#define ASM9260_BM_STAT_CTS			BIT(28)
/* RO. Transmit FIFO/PIO Empty */
#define ASM9260_BM_STAT_TXEMPTY			BIT(27)
/* RO. Receive FIFO Full. */
#define ASM9260_BM_STAT_RXFULL			BIT(26)
/* RO. Transmit FIFO Full. */
#define ASM9260_BM_STAT_TXFULL			BIT(25)
/* RO. Receive FIFO Empty. */
#define ASM9260_BM_STAT_RXEMPTY			BIT(24)
/*
 * RW. The invalid state of the last read of Receive Data. Each
 * bit corresponds to one byte of the RX data. (1 = invalid.)
 */
#define ASM9260_BM_STAT_RXBYTE_INVALID_MASK	(0xf<<20)
/*
 * RO. Overrun Error. This bit is set to 1 if data is received and the FIFO is
 * already full. This bit is cleared to 0 by any write to the Status Register.
 * The FIFO contents remain valid since no further data is written when the
 * FIFO is full; only the contents of the shift register are overwritten. The
 * CPU must now read the data in order to empty the FIFO.
 */
#define ASM9260_BM_STAT_OVERRUNERR		BIT(19)
/*
 * RW. Break Error. For PIO mode, this is for the last character read from the
 * data register. For DMA mode, it will be set to 1 if any received character
 * for a particular RXDMA command had a Break Error. To clear this bit, write a
 * zero to it. Note that clearing this bit does not affect the interrupt status,
 * which must be cleared by writing the interrupt register.
 */
#define ASM9260_BM_STAT_BREAKERR		BIT(18)
/* RW. Parity Error. Same as BREAKERR. */
#define ASM9260_BM_STAT_PARITYERR		BIT(17)
/* RW. Framing Erro. Same as BREAKERR. */
#define ASM9260_BM_STAT_FRAMEERR		BIT(16)
/* RO. Number of bytes received during a Receive DMA command. */
#define ASM9260_BM_STAT_RXCOUNT_MASK		(0xffff<<0)

/* RO. The UART Debug Register contains the state of the DMA signals. */
#define ASM9260_HW_DEBUG			0x0070
/* DMA Command Run Status */
#define ASM9260_BM_DEBUG_TXDMARUN		BIT(5)
#define ASM9260_BM_DEBUG_RXDMARUN		BIT(4)
/* DMA Command End Status */
#define ASM9260_BM_DEBUG_TXCMDEND		BIT(3)
#define ASM9260_BM_DEBUG_RXCMDEND		BIT(2)
/* DMA Request Status */
#define ASM9260_BM_DEBUG_TXDMARQ		BIT(1)
#define ASM9260_BM_DEBUG_RXDMARQ		BIT(0)

#define ASM9260_HW_ILPR				0x0080

#define ASM9260_HW_RS485CTRL			0x0090
/*
 * RW. This bit reverses the polarity of the direction control signal on the RTS
 * (or DTR) pin.
 * If 0, The direction control pin will be driven to logic ‘0’ when the
 * transmitter has data to be sent. It will be driven to logic ‘1’ after the
 * last bit of data has been transmitted.
 */
#define ASM9260_BM_RS485CTRL_ONIV		BIT(5)
/* RW. Enable Auto Direction Control. */
#define ASM9260_BM_RS485CTRL_DIR_CTRL		BIT(4)
/*
 * RW. If 0 and DIR_CTRL = 1, pin RTS is used for direction control.
 * If 1 and DIR_CTRL = 1, pin DTR is used for direction control.
 */
#define ASM9260_BM_RS485CTRL_PINSEL		BIT(3)
/* RW. Enable Auto Address Detect (AAD). */
#define ASM9260_BM_RS485CTRL_AADEN		BIT(2)
/* RW. Disable receiver. */
#define ASM9260_BM_RS485CTRL_RXDIS		BIT(1)
/* RW. Enable RS-485/EIA-485 Normal Multidrop Mode (NMM) */
#define ASM9260_BM_RS485CTRL_RS485EN		BIT(0)

#define ASM9260_HW_RS485ADRMATCH		0x00a0
/* Contains the address match value. */
#define ASM9260_BM_RS485ADRMATCH_MASK		(0xff<<0)

#define ASM9260_HW_RS485DLY			0x00b0
/*
 * RW. Contains the direction control (RTS or DTR) delay value. This delay time
 * is in periods of the baud clock.
 */
#define ASM9260_BM_RS485DLY_MASK		(0xff<<0)

#define ASM9260_HW_AUTOBAUD			0x00c0
/* WO. Auto-baud time-out interrupt clear bit. */
#define ASM9260_BM_AUTOBAUD_ABTOIntClr		BIT(9)
/* WO. End of auto-baud interrupt clear bit. */
#define ASM9260_BM_AUTOBAUD_ABEOIntClr		BIT(8)
/* Restart in case of timeout (counter restarts at next UART Rx falling edge) */
#define ASM9260_BM_AUTOBAUD_AUTORESTART		BIT(2)
/* Auto-baud mode select bit. 0 - Mode 0, 1 - Mode 1. */
#define ASM9260_BM_AUTOBAUD_MODE		BIT(1)
/*
 * Auto-baud start (auto-baud is running). Auto-baud run bit. This bit is
 * automatically cleared after auto-baud completion.
 */
#define ASM9260_BM_AUTOBAUD_START		BIT(0)

#define ASM9260_HW_CTRL3			0x00d0
#define ASM9260_BM_CTRL3_OUTCLK_DIV_MASK	(0xffff<<16)
/*
 * RW. Provide clk over OUTCLK pin. In case of asm9260 it can be configured on
 * pins 137 and 144.
 */
#define ASM9260_BM_CTRL3_MASTERMODE		BIT(6)
/* RW. Baud Rate Mode: 1 - Enable sync mode. 0 - async mode. */
#define ASM9260_BM_CTRL3_SYNCMODE		BIT(4)
/* RW. 1 - MSB bit send frist; 0 - LSB bit frist. */
#define ASM9260_BM_CTRL3_MSBF			BIT(2)
/* RW. 1 - sample rate = 8 x Baudrate; 0 - sample rate = 16 x Baudrate. */
#define ASM9260_BM_CTRL3_BAUD8			BIT(1)
/* RW. 1 - Set word lenght to 9bit. 0 - use ASM9260_BM_LCTRL_WLEN */
#define ASM9260_BM_CTRL3_9BIT			BIT(0)

#define ASM9260_HW_ISO7816_CTRL			0x00e0
/* RW. Enable High Speed mode. */
#define ASM9260_BM_ISO7816CTRL_HS		BIT(12)
/* Disable Successive Receive NACK */
#define ASM9260_BM_ISO7816CTRL_DS_NACK		BIT(8)
#define ASM9260_BM_ISO7816CTRL_MAX_ITER_MASK	(0xff<<4)
/* Receive NACK Inhibit */
#define ASM9260_BM_ISO7816CTRL_INACK		BIT(3)
#define ASM9260_BM_ISO7816CTRL_NEG_DATA		BIT(2)
/* RW. 1 - ISO7816 mode; 0 - USART mode */
#define ASM9260_BM_ISO7816CTRL_ENABLE		BIT(0)

#define ASM9260_HW_ISO7816_ERRCNT		0x00f0
/* Parity error counter. Will be cleared after reading */
#define ASM9260_BM_ISO7816_NB_ERRORS_MASK	(0xff<<0)

#define ASM9260_HW_ISO7816_STATUS		0x0100
/* Max number of Repetitions Reached */
#define ASM9260_BM_ISO7816_STAT_ITERATION	BIT(0)
#endif
