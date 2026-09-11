/*
 * SMSC LAN91C111 on the EtherNAT: register definitions and access macros.
 *
 * This file belongs to FreeMiNT. It's not in the original MiNT 1.12
 * distribution. See the file CHANGES for a detailed log of changes.
 *
 * This is a port of Linux drivers/net/ethernet/smsc/smc91x.h (Linux 7.3)
 * with the platform data bits of include/linux/smc91x.h, reduced to the
 * CONFIG_ATARI bus configuration (8, 16 and 32 bit accesses, no wait
 * states, 16 bit registers byte swapped on the bus) on top of the
 * accessors that asm/io.h gives Linux on m68k. Names are kept as in
 * Linux so that fixes can be carried over.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* SPDX-License-Identifier: GPL-2.0-or-later */
/*------------------------------------------------------------------------
 . smc91x.h - macros for SMSC's 91C9x/91C1xx single-chip Ethernet device.
 .
 . Copyright (C) 1996 by Erik Stahlman
 . Copyright (C) 2001 Standard Microsystems Corporation
 .	Developed by Simple Network Magic Corporation
 . Copyright (C) 2003 Monta Vista Software, Inc.
 .	Unified SMC91x driver by Nicolas Pitre
 .
 .
 . Information contained in this file was obtained from the LAN91C111
 . manual from SMC.  To get a copy, if you really want one, you can find
 . information under www.smsc.com.
 .
 . Authors
 .	Erik Stahlman		<erik@vt.edu>
 .	Daris A Nevil		<dnevil@snmc.com>
 .	Nicolas Pitre 		<nico@fluxnic.net>
 .
 ---------------------------------------------------------------------------*/

#ifndef _SMC91X_H_
#define _SMC91X_H_

typedef unsigned long	u32;
typedef unsigned short	u16;
typedef unsigned char	u8;

#include "mii.h"

/*
 * The EtherNAT wires the 91C111 16 bit data bus with its byte lanes
 * swapped relative to the 68060, so 16 and 32 bit registers come out
 * byte swapped and the packet data comes out in order. This is exactly
 * what readw()/readl() (little endian) and readsl() (raw) give Linux on
 * m68k, so the accessors keep those names and semantics.
 */

static inline u16 swab16(u16 x)
{
	return (x << 8) | (x >> 8);
}

static inline u32 swab32(u32 x)
{
	return (x << 24) | ((x & 0xff00UL) << 8) | ((x >> 8) & 0xff00UL) | (x >> 24);
}

static inline u8 readb(const volatile void *a)
{
	return *(const volatile u8 *) a;
}

static inline u16 readw(const volatile void *a)
{
	return swab16(*(const volatile u16 *) a);
}

static inline u32 readl(const volatile void *a)
{
	return swab32(*(const volatile u32 *) a);
}

static inline void writeb(u8 v, volatile void *a)
{
	*(volatile u8 *) a = v;
}

static inline void writew(u16 v, volatile void *a)
{
	*(volatile u16 *) a = swab16(v);
}

static inline void writel(u32 v, volatile void *a)
{
	*(volatile u32 *) a = swab32(v);
}

/* raw_insw()/raw_outsw(): word streams, no swapping */
static inline void readsw(const volatile void *a, void *p, long l)
{
	volatile u16 *wp = p;

	while (l-- > 0)
		*wp++ = *(const volatile u16 *) a;
}

static inline void writesw(volatile void *a, const void *p, long l)
{
	const volatile u16 *wp = p;

	while (l-- > 0)
		*(volatile u16 *) a = *wp++;
}

/* raw_insb()/raw_outsb(): byte streams */
static inline void readsb(const volatile void *a, void *p, long l)
{
	volatile u8 *bp = p;

	while (l-- > 0)
		*bp++ = *(const volatile u8 *) a;
}

static inline void writesb(volatile void *a, const void *p, long l)
{
	const volatile u8 *bp = p;

	while (l-- > 0)
		*(volatile u8 *) a = *bp++;
}

/* raw_insl()/raw_outsl(): longword streams, no swapping */
static inline void readsl(const volatile void *a, void *p, long l)
{
	volatile u32 *lp = p;

	while (l-- > 0)
		*lp++ = *(const volatile u32 *) a;
}

static inline void writesl(volatile void *a, const void *p, long l)
{
	const volatile u32 *lp = p;

	while (l-- > 0)
		*(volatile u32 *) a = *lp++;
}


/*
 * Define your architecture specific bus configuration parameters here.
 */

#define SMC_CAN_USE_8BIT        1
#define SMC_CAN_USE_16BIT       1
#define SMC_CAN_USE_32BIT       1
#define SMC_NOWAIT              1

#define SMC_inb(a, r)           readb((a) + (r))
#define SMC_inw(a, r)           readw((a) + (r))
#define SMC_inl(a, r)           readl((a) + (r))
#define SMC_outb(v, a, r)       writeb(v, (a) + (r))
#define SMC_outw(lp, v, a, r)   writew(v, (a) + (r))
#define SMC_outl(v, a, r)       writel(v, (a) + (r))
#define SMC_insb(a, r, p, l)    readsb((a) + (r), p, l)
#define SMC_outsb(a, r, p, l)   writesb((a) + (r), p, l)
#define SMC_insw(a, r, p, l)    readsw((a) + (r), p, l)
#define SMC_outsw(a, r, p, l)   writesw((a) + (r), p, l)
#define SMC_insl(a, r, p, l)    readsl((a) + (r), p, l)
#define SMC_outsl(a, r, p, l)   writesl((a) + (r), p, l)

#define RPC_LSA_DEFAULT         RPC_LED_100_10
#define RPC_LSB_DEFAULT         RPC_LED_TX_RX

/* include/linux/smc91x.h */
/* SPDX-License-Identifier: GPL-2.0 */
/*
 * These bits define which access sizes a platform can support, rather
 * than the maximal access size.  So, if your platform can do 16-bit
 * and 32-bit accesses to the SMC91x device, but not 8-bit, set both
 * SMC91X_USE_16BIT and SMC91X_USE_32BIT.
 *
 * The SMC91x driver requires at least one of SMC91X_USE_8BIT or
 * SMC91X_USE_16BIT to be supported - just setting SMC91X_USE_32BIT is
 * an invalid configuration.
 */
#define SMC91X_USE_8BIT (1 << 0)
#define SMC91X_USE_16BIT (1 << 1)
#define SMC91X_USE_32BIT (1 << 2)

#define SMC91X_NOWAIT		(1 << 3)

#define RPC_LED_100_10	(0x00)	/* LED = 100Mbps OR's with 10Mbps link detect */
#define RPC_LED_RES	(0x01)	/* LED = Reserved */
#define RPC_LED_10	(0x02)	/* LED = 10Mbps link detect */
#define RPC_LED_FD	(0x03)	/* LED = Full Duplex Mode */
#define RPC_LED_TX_RX	(0x04)	/* LED = TX or RX packet occurred */
#define RPC_LED_100	(0x05)	/* LED = 100Mbps link detect */
#define RPC_LED_TX	(0x06)	/* LED = TX packet occurred */
#define RPC_LED_RX	(0x07)	/* LED = RX packet occurred */

struct smc91x_platdata {
	unsigned long flags;
	unsigned char leda;
	unsigned char ledb;
};

/* the smc_hardware_send_pkt() tasklet and the smc_phy_configure() work
 * item are plain flags here, see ethernat.c
 */

/* store this information for the driver.. */
struct smc_local {
	/*
	 * If I have to wait until memory is available to send a
	 * packet, I will store the skbuff here, until I get the
	 * desired memory.  Then, I'll send it out and free it.
	 */
	BUF *pending_tx_skb;
	short tx_task;

	/* version/revision of the SMC91x chip */
	long	version;

	/* Contains the current active transmission mode */
	long	tcr_cur_mode;

	/* Contains the current active receive mode */
	long	rcr_cur_mode;

	/* Contains the current active receive/phy mode */
	long	rpc_cur_mode;
	long	ctl_rfduplx;
	long	ctl_rspeed;

	u32	msg_enable;
	u32	phy_type;
	struct mii_if_info mii;

	/* work queue */
	short	phy_configure;
	struct netif *dev;
	short	work_pending;

	/* spin_lock_irq(): the saved status register */
	struct { ushort sr; } lock;

	char *base;

	struct smc91x_platdata cfg;

	/* netif_carrier_ok() */
	short	carrier;

	/* netif_{start,stop,wake}_queue() */
	short	queue_stopped;
	short	pumping;
	short	wake_pending;
	/* netif_trans_update(), in 200 Hz ticks, for the watchdog */
	ulong	trans_start;

	/* the multicast list, IP groups from IGMP with their users */
	ulong	mc_group[16];
	short	mc_refs[16];
	short	mc_count;
	short	mc_overflow;
};

#define SMC_8BIT(p)	((p)->cfg.flags & SMC91X_USE_8BIT)
#define SMC_16BIT(p)	((p)->cfg.flags & SMC91X_USE_16BIT)
#define SMC_32BIT(p)	((p)->cfg.flags & SMC91X_USE_32BIT)

#define SMC_IO_SHIFT	0

/* Because of bank switching, the LAN91x uses only 16 I/O ports */
#define SMC_IO_EXTENT	(16 << SMC_IO_SHIFT)
#define SMC_DATA_EXTENT (4)

/*
 . Bank Select Register:
 .
 .		yyyy yyyy 0000 00xx
 .		xx 		= bank number
 .		yyyy yyyy	= 0x33, for identification purposes.
*/
#define BANK_SELECT		(14 << SMC_IO_SHIFT)


// Transmit Control Register
/* BANK 0  */
#define TCR_REG(lp) 	SMC_REG(lp, 0x0000, 0)
#define TCR_ENABLE	0x0001	// When 1 we can transmit
#define TCR_LOOP	0x0002	// Controls output pin LBK
#define TCR_FORCOL	0x0004	// When 1 will force a collision
#define TCR_PAD_EN	0x0080	// When 1 will pad tx frames < 64 bytes w/0
#define TCR_NOCRC	0x0100	// When 1 will not append CRC to tx frames
#define TCR_MON_CSN	0x0400	// When 1 tx monitors carrier
#define TCR_FDUPLX    	0x0800  // When 1 enables full duplex operation
#define TCR_STP_SQET	0x1000	// When 1 stops tx if Signal Quality Error
#define TCR_EPH_LOOP	0x2000	// When 1 enables EPH block loopback
#define TCR_SWFDUP	0x8000	// When 1 enables Switched Full Duplex mode

#define TCR_CLEAR	0	/* do NOTHING */
/* the default settings for the TCR register : */
#define TCR_DEFAULT	(TCR_ENABLE | TCR_PAD_EN)


// EPH Status Register
/* BANK 0  */
#define EPH_STATUS_REG(lp)	SMC_REG(lp, 0x0002, 0)
#define ES_TX_SUC	0x0001	// Last TX was successful
#define ES_SNGL_COL	0x0002	// Single collision detected for last tx
#define ES_MUL_COL	0x0004	// Multiple collisions detected for last tx
#define ES_LTX_MULT	0x0008	// Last tx was a multicast
#define ES_16COL	0x0010	// 16 Collisions Reached
#define ES_SQET		0x0020	// Signal Quality Error Test
#define ES_LTXBRD	0x0040	// Last tx was a broadcast
#define ES_TXDEFR	0x0080	// Transmit Deferred
#define ES_LATCOL	0x0200	// Late collision detected on last tx
#define ES_LOSTCARR	0x0400	// Lost Carrier Sense
#define ES_EXC_DEF	0x0800	// Excessive Deferral
#define ES_CTR_ROL	0x1000	// Counter Roll Over indication
#define ES_LINK_OK	0x4000	// Driven by inverted value of nLNK pin
#define ES_TXUNRN	0x8000	// Tx Underrun


// Receive Control Register
/* BANK 0  */
#define RCR_REG(lp)		SMC_REG(lp, 0x0004, 0)
#define RCR_RX_ABORT	0x0001	// Set if a rx frame was aborted
#define RCR_PRMS	0x0002	// Enable promiscuous mode
#define RCR_ALMUL	0x0004	// When set accepts all multicast frames
#define RCR_RXEN	0x0100	// IFF this is set, we can receive packets
#define RCR_STRIP_CRC	0x0200	// When set strips CRC from rx packets
#define RCR_ABORT_ENB	0x0200	// When set will abort rx on collision
#define RCR_FILT_CAR	0x0400	// When set filters leading 12 bit s of carrier
#define RCR_SOFTRST	0x8000 	// resets the chip

/* the normal settings for the RCR register : */
#define RCR_DEFAULT	(RCR_STRIP_CRC | RCR_RXEN)
#define RCR_CLEAR	0x0	// set it to a base state


// Counter Register
/* BANK 0  */
#define COUNTER_REG(lp)	SMC_REG(lp, 0x0006, 0)


// Memory Information Register
/* BANK 0  */
#define MIR_REG(lp)		SMC_REG(lp, 0x0008, 0)


// Receive/Phy Control Register
/* BANK 0  */
#define RPC_REG(lp)		SMC_REG(lp, 0x000A, 0)
#define RPC_SPEED	0x2000	// When 1 PHY is in 100Mbps mode.
#define RPC_DPLX	0x1000	// When 1 PHY is in Full-Duplex Mode
#define RPC_ANEG	0x0800	// When 1 PHY is in Auto-Negotiate Mode
#define RPC_LSXA_SHFT	5	// Bits to shift LS2A,LS1A,LS0A to lsb
#define RPC_LSXB_SHFT	2	// Bits to get LS2B,LS1B,LS0B to lsb

#define RPC_DEFAULT (RPC_ANEG | RPC_SPEED | RPC_DPLX)


/* Bank 0 0x0C is reserved */

// Bank Select Register
/* All Banks */
#define BSR_REG		0x000E


// Configuration Reg
/* BANK 1 */
#define CONFIG_REG(lp)	SMC_REG(lp, 0x0000,	1)
#define CONFIG_EXT_PHY	0x0200	// 1=external MII, 0=internal Phy
#define CONFIG_GPCNTRL	0x0400	// Inverse value drives pin nCNTRL
#define CONFIG_NO_WAIT	0x1000	// When 1 no extra wait states on ISA bus
#define CONFIG_EPH_POWER_EN 0x8000 // When 0 EPH is placed into low power mode.

// Default is powered-up, Internal Phy, Wait States, and pin nCNTRL=low
#define CONFIG_DEFAULT	(CONFIG_EPH_POWER_EN)


// Base Address Register
/* BANK 1 */
#define BASE_REG(lp)	SMC_REG(lp, 0x0002, 1)


// Individual Address Registers
/* BANK 1 */
#define ADDR0_REG(lp)	SMC_REG(lp, 0x0004, 1)
#define ADDR1_REG(lp)	SMC_REG(lp, 0x0006, 1)
#define ADDR2_REG(lp)	SMC_REG(lp, 0x0008, 1)


// General Purpose Register
/* BANK 1 */
#define GP_REG(lp)		SMC_REG(lp, 0x000A, 1)


// Control Register
/* BANK 1 */
#define CTL_REG(lp)		SMC_REG(lp, 0x000C, 1)
#define CTL_RCV_BAD	0x4000 // When 1 bad CRC packets are received
#define CTL_AUTO_RELEASE 0x0800 // When 1 tx pages are released automatically
#define CTL_LE_ENABLE	0x0080 // When 1 enables Link Error interrupt
#define CTL_CR_ENABLE	0x0040 // When 1 enables Counter Rollover interrupt
#define CTL_TE_ENABLE	0x0020 // When 1 enables Transmit Error interrupt
#define CTL_EEPROM_SELECT 0x0004 // Controls EEPROM reload & store
#define CTL_RELOAD	0x0002 // When set reads EEPROM into registers
#define CTL_STORE	0x0001 // When set stores registers into EEPROM


// MMU Command Register
/* BANK 2 */
#define MMU_CMD_REG(lp)	SMC_REG(lp, 0x0000, 2)
#define MC_BUSY		1	// When 1 the last release has not completed
#define MC_NOP		(0<<5)	// No Op
#define MC_ALLOC	(1<<5) 	// OR with number of 256 byte packets
#define MC_RESET	(2<<5)	// Reset MMU to initial state
#define MC_REMOVE	(3<<5) 	// Remove the current rx packet
#define MC_RELEASE  	(4<<5) 	// Remove and release the current rx packet
#define MC_FREEPKT  	(5<<5) 	// Release packet in PNR register
#define MC_ENQUEUE	(6<<5)	// Enqueue the packet for transmit
#define MC_RSTTXFIFO	(7<<5)	// Reset the TX FIFOs


// Packet Number Register
/* BANK 2 */
#define PN_REG(lp)		SMC_REG(lp, 0x0002, 2)


// Allocation Result Register
/* BANK 2 */
#define AR_REG(lp)		SMC_REG(lp, 0x0003, 2)
#define AR_FAILED	0x80	// Alocation Failed


// TX FIFO Ports Register
/* BANK 2 */
#define TXFIFO_REG(lp)	SMC_REG(lp, 0x0004, 2)
#define TXFIFO_TEMPTY	0x80	// TX FIFO Empty

// RX FIFO Ports Register
/* BANK 2 */
#define RXFIFO_REG(lp)	SMC_REG(lp, 0x0005, 2)
#define RXFIFO_REMPTY	0x80	// RX FIFO Empty

#define FIFO_REG(lp)	SMC_REG(lp, 0x0004, 2)

// Pointer Register
/* BANK 2 */
#define PTR_REG(lp)		SMC_REG(lp, 0x0006, 2)
#define PTR_RCV		0x8000 // 1=Receive area, 0=Transmit area
#define PTR_AUTOINC 	0x4000 // Auto increment the pointer on each access
#define PTR_READ	0x2000 // When 1 the operation is a read


// Data Register
/* BANK 2 */
#define DATA_REG(lp)	SMC_REG(lp, 0x0008, 2)


// Interrupt Status/Acknowledge Register
/* BANK 2 */
#define INT_REG(lp)		SMC_REG(lp, 0x000C, 2)


// Interrupt Mask Register
/* BANK 2 */
#define IM_REG(lp)		SMC_REG(lp, 0x000D, 2)
#define IM_MDINT	0x80 // PHY MI Register 18 Interrupt
#define IM_ERCV_INT	0x40 // Early Receive Interrupt
#define IM_EPH_INT	0x20 // Set by Ethernet Protocol Handler section
#define IM_RX_OVRN_INT	0x10 // Set by Receiver Overruns
#define IM_ALLOC_INT	0x08 // Set when allocation request is completed
#define IM_TX_EMPTY_INT	0x04 // Set if the TX FIFO goes empty
#define IM_TX_INT	0x02 // Transmit Interrupt
#define IM_RCV_INT	0x01 // Receive Interrupt


// Multicast Table Registers
/* BANK 3 */
#define MCAST_REG1(lp)	SMC_REG(lp, 0x0000, 3)
#define MCAST_REG2(lp)	SMC_REG(lp, 0x0002, 3)
#define MCAST_REG3(lp)	SMC_REG(lp, 0x0004, 3)
#define MCAST_REG4(lp)	SMC_REG(lp, 0x0006, 3)


// Management Interface Register (MII)
/* BANK 3 */
#define MII_REG(lp)		SMC_REG(lp, 0x0008, 3)
#define MII_MSK_CRS100	0x4000 // Disables CRS100 detection during tx half dup
#define MII_MDOE	0x0008 // MII Output Enable
#define MII_MCLK	0x0004 // MII Clock, pin MDCLK
#define MII_MDI		0x0002 // MII Input, pin MDI
#define MII_MDO		0x0001 // MII Output, pin MDO


// Revision Register
/* BANK 3 */
/* ( hi: chip id   low: rev # ) */
#define REV_REG(lp)		SMC_REG(lp, 0x000A, 3)


// Early RCV Register
/* BANK 3 */
/* this is NOT on SMC9192 */
#define ERCV_REG(lp)	SMC_REG(lp, 0x000C, 3)
#define ERCV_RCV_DISCRD	0x0080 // When 1 discards a packet being received
#define ERCV_THRESHOLD	0x001F // ERCV Threshold Mask


// External Register
/* BANK 7 */
#define EXT_REG(lp)		SMC_REG(lp, 0x0000, 7)


#define CHIP_9192	3
#define CHIP_9194	4
#define CHIP_9195	5
#define CHIP_9196	6
#define CHIP_91100	7
#define CHIP_91100FD	8
#define CHIP_91111FD	9


/*
 . Receive status bits
*/
#define RS_ALGNERR	0x8000
#define RS_BRODCAST	0x4000
#define RS_BADCRC	0x2000
#define RS_ODDFRAME	0x1000
#define RS_TOOLONG	0x0800
#define RS_TOOSHORT	0x0400
#define RS_MULTICAST	0x0001
#define RS_ERRORS	(RS_ALGNERR | RS_BADCRC | RS_TOOLONG | RS_TOOSHORT)


/*
 * PHY IDs
 *  LAN83C183 == LAN91C111 Internal PHY
 */
#define PHY_LAN83C183	0x0016f840
#define PHY_LAN83C180	0x02821c50

/*
 * PHY Register Addresses (LAN91C111 Internal PHY)
 *
 * Generic PHY registers can be found in <linux/mii.h>
 *
 * These phy registers are specific to our on-board phy.
 */

// PHY Configuration Register 1
#define PHY_CFG1_REG		0x10
#define PHY_CFG1_LNKDIS		0x8000	// 1=Rx Link Detect Function disabled
#define PHY_CFG1_XMTDIS		0x4000	// 1=TP Transmitter Disabled
#define PHY_CFG1_XMTPDN		0x2000	// 1=TP Transmitter Powered Down
#define PHY_CFG1_BYPSCR		0x0400	// 1=Bypass scrambler/descrambler
#define PHY_CFG1_UNSCDS		0x0200	// 1=Unscramble Idle Reception Disable
#define PHY_CFG1_EQLZR		0x0100	// 1=Rx Equalizer Disabled
#define PHY_CFG1_CABLE		0x0080	// 1=STP(150ohm), 0=UTP(100ohm)
#define PHY_CFG1_RLVL0		0x0040	// 1=Rx Squelch level reduced by 4.5db
#define PHY_CFG1_TLVL_SHIFT	2	// Transmit Output Level Adjust
#define PHY_CFG1_TLVL_MASK	0x003C
#define PHY_CFG1_TRF_MASK	0x0003	// Transmitter Rise/Fall time


// PHY Configuration Register 2
#define PHY_CFG2_REG		0x11
#define PHY_CFG2_APOLDIS	0x0020	// 1=Auto Polarity Correction disabled
#define PHY_CFG2_JABDIS		0x0010	// 1=Jabber disabled
#define PHY_CFG2_MREG		0x0008	// 1=Multiple register access (MII mgt)
#define PHY_CFG2_INTMDIO	0x0004	// 1=Interrupt signaled with MDIO pulseo

// PHY Status Output (and Interrupt status) Register
#define PHY_INT_REG		0x12	// Status Output (Interrupt Status)
#define PHY_INT_INT		0x8000	// 1=bits have changed since last read
#define PHY_INT_LNKFAIL		0x4000	// 1=Link Not detected
#define PHY_INT_LOSSSYNC	0x2000	// 1=Descrambler has lost sync
#define PHY_INT_CWRD		0x1000	// 1=Invalid 4B5B code detected on rx
#define PHY_INT_SSD		0x0800	// 1=No Start Of Stream detected on rx
#define PHY_INT_ESD		0x0400	// 1=No End Of Stream detected on rx
#define PHY_INT_RPOL		0x0200	// 1=Reverse Polarity detected
#define PHY_INT_JAB		0x0100	// 1=Jabber detected
#define PHY_INT_SPDDET		0x0080	// 1=100Base-TX mode, 0=10Base-T mode
#define PHY_INT_DPLXDET		0x0040	// 1=Device in Full Duplex

// PHY Interrupt/Status Mask Register
#define PHY_MASK_REG		0x13	// Interrupt Mask
// Uses the same bit definitions as PHY_INT_REG


/*
 * Macros to abstract register access according to the data bus
 * capabilities.  Please use those and not the in/out primitives.
 * Note: the following macros do *not* select the bank -- this must
 * be done separately as needed in the main code.  The SMC_REG() macro
 * only uses the bank argument for debugging purposes (when enabled).
 *
 * Note: despite inline functions being safer, everything leading to this
 * should preferably be macros to let BUG() display the line number in
 * the core source code since we're interested in the top call site
 * not in any inline function location.
 */

#define SMC_REG(lp, reg, bank)	(reg<<SMC_IO_SHIFT)

/*
 * Hack Alert: Some setups just can't write 8 or 16 bits reliably when not
 * aligned to a 32 bit boundary.  I tell you that does exist!
 * Fortunately the affected register accesses can be easily worked around
 * since we can write zeroes to the preceding 16 bits without adverse
 * effects and use a 32-bit access.
 *
 * Enforce it on any 32-bit capable setup for now.
 */
#define SMC_MUST_ALIGN_WRITE(lp)	SMC_32BIT(lp)

#define SMC_GET_PN(lp)						\
	(SMC_8BIT(lp)	? (SMC_inb(ioaddr, PN_REG(lp)))	\
				: (SMC_inw(ioaddr, PN_REG(lp)) & 0xFF))

#define SMC_SET_PN(lp, x)						\
	do {								\
		if (SMC_MUST_ALIGN_WRITE(lp))				\
			SMC_outl((u32)(x)<<16, ioaddr, SMC_REG(lp, 0, 2));	\
		else if (SMC_8BIT(lp))				\
			SMC_outb(x, ioaddr, PN_REG(lp));		\
		else							\
			SMC_outw(lp, x, ioaddr, PN_REG(lp));		\
	} while (0)

#define SMC_GET_AR(lp)						\
	(SMC_8BIT(lp)	? (SMC_inb(ioaddr, AR_REG(lp)))	\
				: (SMC_inw(ioaddr, PN_REG(lp)) >> 8))

#define SMC_GET_TXFIFO(lp)						\
	(SMC_8BIT(lp)	? (SMC_inb(ioaddr, TXFIFO_REG(lp)))	\
				: (SMC_inw(ioaddr, TXFIFO_REG(lp)) & 0xFF))

#define SMC_GET_RXFIFO(lp)						\
	(SMC_8BIT(lp)	? (SMC_inb(ioaddr, RXFIFO_REG(lp)))	\
				: (SMC_inw(ioaddr, TXFIFO_REG(lp)) >> 8))

#define SMC_GET_INT(lp)						\
	(SMC_8BIT(lp)	? (SMC_inb(ioaddr, INT_REG(lp)))	\
				: (SMC_inw(ioaddr, INT_REG(lp)) & 0xFF))

#define SMC_ACK_INT(lp, x)						\
	do {								\
		if (SMC_8BIT(lp))					\
			SMC_outb(x, ioaddr, INT_REG(lp));		\
		else {							\
			ushort __flags;					\
			long __mask;					\
			__flags = spl7();				\
			__mask = SMC_inw(ioaddr, INT_REG(lp)) & ~0xff; \
			SMC_outw(lp, __mask | (x), ioaddr, INT_REG(lp)); \
			spl(__flags);					\
		}							\
	} while (0)

#define SMC_GET_INT_MASK(lp)						\
	(SMC_8BIT(lp)	? (SMC_inb(ioaddr, IM_REG(lp)))	\
				: (SMC_inw(ioaddr, INT_REG(lp)) >> 8))

#define SMC_SET_INT_MASK(lp, x)					\
	do {								\
		if (SMC_8BIT(lp))					\
			SMC_outb(x, ioaddr, IM_REG(lp));		\
		else							\
			SMC_outw(lp, (x) << 8, ioaddr, INT_REG(lp));	\
	} while (0)

#define SMC_CURRENT_BANK(lp)	SMC_inw(ioaddr, BANK_SELECT)

#define SMC_SELECT_BANK(lp, x)					\
	do {								\
		if (SMC_MUST_ALIGN_WRITE(lp))				\
			SMC_outl((u32)(x)<<16, ioaddr, 12<<SMC_IO_SHIFT);	\
		else							\
			SMC_outw(lp, x, ioaddr, BANK_SELECT);		\
	} while (0)

#define SMC_GET_BASE(lp)		SMC_inw(ioaddr, BASE_REG(lp))

#define SMC_SET_BASE(lp, x)	SMC_outw(lp, x, ioaddr, BASE_REG(lp))

#define SMC_GET_CONFIG(lp)	SMC_inw(ioaddr, CONFIG_REG(lp))

#define SMC_SET_CONFIG(lp, x)	SMC_outw(lp, x, ioaddr, CONFIG_REG(lp))

#define SMC_GET_COUNTER(lp)	SMC_inw(ioaddr, COUNTER_REG(lp))

#define SMC_GET_CTL(lp)		SMC_inw(ioaddr, CTL_REG(lp))

#define SMC_SET_CTL(lp, x)	SMC_outw(lp, x, ioaddr, CTL_REG(lp))

#define SMC_GET_MII(lp)		SMC_inw(ioaddr, MII_REG(lp))

#define SMC_GET_GP(lp)		SMC_inw(ioaddr, GP_REG(lp))

#define SMC_SET_GP(lp, x)						\
	do {								\
		if (SMC_MUST_ALIGN_WRITE(lp))				\
			SMC_outl((u32)(x)<<16, ioaddr, SMC_REG(lp, 8, 1));	\
		else							\
			SMC_outw(lp, x, ioaddr, GP_REG(lp));		\
	} while (0)

#define SMC_SET_MII(lp, x)	SMC_outw(lp, x, ioaddr, MII_REG(lp))

#define SMC_GET_MIR(lp)		SMC_inw(ioaddr, MIR_REG(lp))

#define SMC_SET_MIR(lp, x)	SMC_outw(lp, x, ioaddr, MIR_REG(lp))

#define SMC_GET_MMU_CMD(lp)	SMC_inw(ioaddr, MMU_CMD_REG(lp))

#define SMC_SET_MMU_CMD(lp, x)	SMC_outw(lp, x, ioaddr, MMU_CMD_REG(lp))

#define SMC_GET_FIFO(lp)	SMC_inw(ioaddr, FIFO_REG(lp))

#define SMC_GET_PTR(lp)		SMC_inw(ioaddr, PTR_REG(lp))

#define SMC_SET_PTR(lp, x)						\
	do {								\
		if (SMC_MUST_ALIGN_WRITE(lp))				\
			SMC_outl((u32)(x)<<16, ioaddr, SMC_REG(lp, 4, 2));	\
		else							\
			SMC_outw(lp, x, ioaddr, PTR_REG(lp));		\
	} while (0)

#define SMC_GET_EPH_STATUS(lp)	SMC_inw(ioaddr, EPH_STATUS_REG(lp))

#define SMC_GET_RCR(lp)		SMC_inw(ioaddr, RCR_REG(lp))

#define SMC_SET_RCR(lp, x)		SMC_outw(lp, x, ioaddr, RCR_REG(lp))

#define SMC_GET_REV(lp)		SMC_inw(ioaddr, REV_REG(lp))

#define SMC_GET_RPC(lp)		SMC_inw(ioaddr, RPC_REG(lp))

#define SMC_SET_RPC(lp, x)						\
	do {								\
		if (SMC_MUST_ALIGN_WRITE(lp))				\
			SMC_outl((u32)(x)<<16, ioaddr, SMC_REG(lp, 8, 0));	\
		else							\
			SMC_outw(lp, x, ioaddr, RPC_REG(lp));		\
	} while (0)

#define SMC_GET_TCR(lp)		SMC_inw(ioaddr, TCR_REG(lp))

#define SMC_SET_TCR(lp, x)	SMC_outw(lp, x, ioaddr, TCR_REG(lp))

#define SMC_GET_MAC_ADDR(lp, addr)					\
	do {								\
		u16 __v;						\
		__v = SMC_inw(ioaddr, ADDR0_REG(lp));			\
		addr[0] = __v; addr[1] = __v >> 8;			\
		__v = SMC_inw(ioaddr, ADDR1_REG(lp));			\
		addr[2] = __v; addr[3] = __v >> 8;			\
		__v = SMC_inw(ioaddr, ADDR2_REG(lp));			\
		addr[4] = __v; addr[5] = __v >> 8;			\
	} while (0)

#define SMC_SET_MAC_ADDR(lp, addr)					\
	do {								\
		SMC_outw(lp, addr[0] | (addr[1] << 8), ioaddr, ADDR0_REG(lp)); \
		SMC_outw(lp, addr[2] | (addr[3] << 8), ioaddr, ADDR1_REG(lp)); \
		SMC_outw(lp, addr[4] | (addr[5] << 8), ioaddr, ADDR2_REG(lp)); \
	} while (0)

#define SMC_SET_MCAST(lp, x)						\
	do {								\
		const unsigned char *mt = (x);				\
		SMC_outw(lp, mt[0] | (mt[1] << 8), ioaddr, MCAST_REG1(lp)); \
		SMC_outw(lp, mt[2] | (mt[3] << 8), ioaddr, MCAST_REG2(lp)); \
		SMC_outw(lp, mt[4] | (mt[5] << 8), ioaddr, MCAST_REG3(lp)); \
		SMC_outw(lp, mt[6] | (mt[7] << 8), ioaddr, MCAST_REG4(lp)); \
	} while (0)

#define SMC_PUT_PKT_HDR(lp, status, length)				\
	do {								\
		if (SMC_32BIT(lp))					\
			SMC_outl((u32)(status) | (u32)(length)<<16, ioaddr,	\
				 DATA_REG(lp));			\
		else {							\
			SMC_outw(lp, status, ioaddr, DATA_REG(lp));	\
			SMC_outw(lp, length, ioaddr, DATA_REG(lp));	\
		}							\
	} while (0)

#define SMC_GET_PKT_HDR(lp, status, length)				\
	do {								\
		if (SMC_32BIT(lp)) {				\
			u32 __val = SMC_inl(ioaddr, DATA_REG(lp)); \
			(status) = __val & 0xffff;			\
			(length) = __val >> 16;				\
		} else {						\
			(status) = SMC_inw(ioaddr, DATA_REG(lp));	\
			(length) = SMC_inw(ioaddr, DATA_REG(lp));	\
		}							\
	} while (0)

#define SMC_PUSH_DATA(lp, p, l)					\
	do {								\
		if (SMC_32BIT(lp)) {				\
			char *__ptr = (char *)(p);			\
			long __len = (l);				\
			if (__len >= 2 && (unsigned long)__ptr & 2) {	\
				__len -= 2;				\
				SMC_outsw(ioaddr, DATA_REG(lp), __ptr, 1); \
				__ptr += 2;				\
			}						\
			SMC_outsl(ioaddr, DATA_REG(lp), __ptr, __len>>2); \
			if (__len & 2) {				\
				__ptr += (__len & ~3);			\
				SMC_outsw(ioaddr, DATA_REG(lp), __ptr, 1); \
			}						\
		} else if (SMC_16BIT(lp))				\
			SMC_outsw(ioaddr, DATA_REG(lp), p, (l) >> 1);	\
		else if (SMC_8BIT(lp))				\
			SMC_outsb(ioaddr, DATA_REG(lp), p, l);	\
	} while (0)

#define SMC_PULL_DATA(lp, p, l)					\
	do {								\
		if (SMC_32BIT(lp)) {				\
			char *__ptr = (char *)(p);			\
			long __len = (l);				\
			if ((unsigned long)__ptr & 2) {			\
				/*					\
				 * We want 32bit alignment here.	\
				 * Since some buses perform a full	\
				 * 32bit fetch even for 16bit data	\
				 * we can't use SMC_inw() here.		\
				 * Back both source (on-chip) and	\
				 * destination pointers of 2 bytes.	\
				 * This is possible since the call to	\
				 * SMC_GET_PKT_HDR() already advanced	\
				 * the source pointer of 4 bytes, and	\
				 * the skb_reserve(skb, 2) advanced	\
				 * the destination pointer of 2 bytes.	\
				 */					\
				__ptr -= 2;				\
				__len += 2;				\
				SMC_SET_PTR(lp,			\
					2|PTR_READ|PTR_RCV|PTR_AUTOINC); \
			}						\
			__len += 2;					\
			SMC_insl(ioaddr, DATA_REG(lp), __ptr, __len>>2); \
		} else if (SMC_16BIT(lp))				\
			SMC_insw(ioaddr, DATA_REG(lp), p, (l) >> 1);	\
		else if (SMC_8BIT(lp))				\
			SMC_insb(ioaddr, DATA_REG(lp), p, l);		\
	} while (0)

#endif  /* _SMC91X_H_ */
