/*
 * EtherNAT driver for FreeMiNT: the SMSC LAN91C111 on the EtherNAT board
 * for the CT60.
 *
 * This file belongs to FreeMiNT. It's not in the original MiNT 1.12
 * distribution. See the file CHANGES for a detailed log of changes.
 *
 * This is a port of the Linux smc91x driver, drivers/net/ethernet/smsc/
 * smc91x.c (Linux 7.3), and of the EtherNAT platform glue in arch/m68k/
 * atari/config.c and ataints.c; the helpers it uses from drivers/net/
 * mii.c are in mii.c. Function, variable and register names are kept as in Linux
 * so that fixes can be carried over; the MiNTNet interface (ethernat_*)
 * at the end of this file takes the place of the Linux net_device. What
 * is not ported: ethtool, the EEPROM interface, IRQ autodetection, DMA,
 * netpoll and power management.
 *
 * Two Linux mechanisms are mapped onto FreeMiNT ones: the tasklet that
 * pushes a packet once the chip has found memory for it runs at the end
 * of the interrupt handler, and the work item that reconfigures the PHY
 * after a transmit timeout runs in a kernel thread.
 *
 * The kernel is built with -mshort, so Linux "int" is "long" here.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * smc91x.c
 * This is a driver for SMSC's 91C9x/91C1xx single-chip Ethernet devices.
 *
 * Copyright (C) 1996 by Erik Stahlman
 * Copyright (C) 2001 Standard Microsystems Corporation
 *	Developed by Simple Network Magic Corporation
 * Copyright (C) 2003 Monta Vista Software, Inc.
 *	Unified SMC91x driver by Nicolas Pitre
 *
 * Arguments:
 * 	io	= for the base address
 *	irq	= for the IRQ
 *	nowait	= 0 for normal wait states, 1 eliminates additional wait states
 *
 * original author:
 * 	Erik Stahlman <erik@vt.edu>
 *
 * hardware multicast code:
 *    Peter Cammaert <pc@denkart.be>
 *
 * contributors:
 * 	Daris A Nevil <dnevil@snmc.com>
 *      Nicolas Pitre <nico@fluxnic.net>
 *	Russell King <rmk@arm.linux.org.uk>
 *
 * History:
 *   08/20/00  Arnaldo Melo       fix kfree(skb) in smc_hardware_send_packet
 *   12/15/00  Christian Jullien  fix "Warning: kfree_skb on hard IRQ"
 *   03/16/01  Daris A Nevil      modified smc9194.c for use with LAN91C111
 *   08/22/01  Scott Anderson     merge changes from smc9194 to smc91111
 *   08/21/01  Pramod B Bhardwaj  added support for RevB of LAN91C111
 *   12/20/01  Jeff Sutherland    initial port to Xscale PXA with DMA support
 *   04/07/03  Nicolas Pitre      unified SMC91x driver, killed irq races,
 *                                more bus abstraction, big cleanup, etc.
 *   29/09/03  Russell King       - add driver model support
 *                                - ethtool support
 *                                - convert to use generic MII interface
 *                                - add link up/down notification
 *                                - don't try to handle full negotiation in
 *                                  smc_phy_configure
 *                                - clean up (and fix stack overrun) in PHY
 *                                  MII read/write functions
 *   22/09/04  Nicolas Pitre      big update (see commit log for details)
 */

# include "global.h"

# include "buf.h"
# include "inet4/if.h"
# include "inet4/ifeth.h"
# include "inet4/igmp.h"
# include "netinfo.h"

# include "mint/delay.h"
# include "mint/fcntl.h"
# include "mint/sockio.h"
# include "mint/time.h"
# include "mint/arch/asm_spl.h"

# include <mint/osbind.h>

# include "91c111.h"
# include "ethernat_200Hzint.h"


# define likely(x)	__builtin_expect(!!(x), 1)
# define unlikely(x)	__builtin_expect(!!(x), 0)

# define IRQ_HANDLED	1L

# define NETDEV_TX_OK	0

/* the 200 Hz system timer plays jiffies; the kernel's hz_200 is not
 * exported to modules
 */
# undef jiffies
# define jiffies	(*(volatile ulong *) 0x4baUL)
# define time_after(a, b)	((long) (b) - (long) (a) < 0)
# define msecs_to_jiffies(m)	((m) / (1000 / HZ))

/*
 * lp->lock: what spin_lock_irq() protects against here is the interrupt
 * handler, so the lock is the saved status register.
 */
# define spin_lock_irq(l)		((l)->sr = spl7())
# define spin_unlock_irq(l)		spl((l)->sr)
# define spin_lock_irqsave(l, f)	((f) = spl7())
# define spin_unlock_irqrestore(l, f)	spl(f)
/* only taken from the interrupt handler, where nothing else runs */
# define spin_lock(l)			((void) 0)
# define spin_unlock(l)			((void) 0)


/*
 * The platform: arch/m68k/atari/config.c and ataints.c.
 */

/* smc91x-regs */
#define ATARI_ETHERNAT_PHYS_ADDR	0x80000000UL

/* Linux IRQ 140, vector 0xc4 */
#define ATARI_ETHERNAT_VECTOR		0xc4

/*
 * CPLD interrupt register is at phys. 0x80000023
 * Bit 1 gates the LAN interrupt, bit 2 the USB interrupt, which the
 * EtherNAT USB driver handles the same way.
 */
#define ATARI_ETHERNAT_CPLD		((volatile u8 *) 0x80000023UL)
#define ATARI_ETHERNAT_CPLD_LAN		(1 << 1)


static const char version[] __attribute__((unused)) =
	"smc91x.c: v1.1, sep 22 2004 by Nicolas Pitre <nico@fluxnic.net>";

/*
 * Transmit timeout, default 5 seconds.
 */
static long watchdog = 1000;

/*
 * The internal workings of the driver.  If you are changing anything
 * here with the SMC stuff, you should have the datasheet and know
 * what you are doing.
 */
#define CARDNAME "smc91x"

/*
 * Use power-down feature of the chip
 */
#define POWER_DOWN		1

/*
 * Wait time for memory to be free.  This probably shouldn't be
 * tuned that much, as waiting for this means nothing else happens
 * in the system
 */
#define MEMORY_WAIT_TIME	16

/*
 * The maximum number of processing loops allowed for each call to the
 * IRQ handler.
 */
#define MAX_IRQ_LOOPS		8

/*
 * This selects whether TX packets are sent one by one to the SMC91x internal
 * memory and throttled until transmission completes.  This may prevent
 * RX overruns a litle by keeping much of the memory free for RX packets
 * but to the expense of reduced TX throughput and increased IRQ overhead.
 * Note this is not a cure for a too slow data bus or too high IRQ latency.
 */
#define THROTTLE_TX_PKTS	0

/*
 * The MII clock high/low times.  2x this number gives the MII clock period
 * in microseconds. (was 50, but this gives 6.4ms for each MII transaction!)
 */
#define MII_DELAY		1

/*
 * netdev_dbg()/netdev_info() of the driver core, most of them in
 * interrupt context, where nothing may print on FreeMiNT. Kept as
 * empty statements so the messages stay where Linux has them.
 */
#define DBG(n, dev, fmt, ...)	do { } while (0)
#define PRINTK(dev, fmt, ...)	do { } while (0)

static inline void PRINT_PKT(u8 *buf, long length) { UNUSED(buf); UNUSED(length); }


/* this enables an interrupt in the interrupt mask register */
#define SMC_ENABLE_INT(lp, x) do {					\
	unsigned char mask;						\
	ushort smc_enable_flags;					\
	spin_lock_irqsave(&lp->lock, smc_enable_flags);			\
	mask = SMC_GET_INT_MASK(lp);					\
	mask |= (x);							\
	SMC_SET_INT_MASK(lp, mask);					\
	spin_unlock_irqrestore(&lp->lock, smc_enable_flags);		\
} while (0)

/* this disables an interrupt from the interrupt mask register */
#define SMC_DISABLE_INT(lp, x) do {					\
	unsigned char mask;						\
	ushort smc_disable_flags;					\
	spin_lock_irqsave(&lp->lock, smc_disable_flags);		\
	mask = SMC_GET_INT_MASK(lp);					\
	mask &= ~(x);							\
	SMC_SET_INT_MASK(lp, mask);					\
	spin_unlock_irqrestore(&lp->lock, smc_disable_flags);		\
} while (0)

/*
 * Wait while MMU is busy.  This is usually in the order of a few nanosecs
 * if at all, but let's avoid deadlocking the system if the hardware
 * decides to go south.
 */
#define SMC_WAIT_MMU_BUSY(lp) do {					\
	if (unlikely(SMC_GET_MMU_CMD(lp) & MC_BUSY)) {		\
		ulong timeout = jiffies + 2;				\
		while (SMC_GET_MMU_CMD(lp) & MC_BUSY) {		\
			if (time_after(jiffies, timeout)) {		\
				DBG(0, dev, "timeout %s line %d\n",	\
					   __FILE__, __LINE__);		\
				break;					\
			}						\
		}							\
	}								\
} while (0)


static struct netif if_ethernat;
static struct smc_local smc_priv;

static void smc_hardware_send_pkt(struct smc_local *lp);
static long smc_hard_start_xmit(BUF *skb, struct netif *dev);
static void smc_phy_configure(struct smc_local *lp);


/*
 * The EtherNAT irq_chip of ataints.c: the CPLD gate of the LAN interrupt.
 * It is opened once when the vector is installed and, as on Linux, never
 * closed again; the chip's own interrupt mask does the silencing. The USB
 * driver changes the same register, so the read-modify-write is done
 * with interrupts off.
 */
static void atari_ethernat_enable(void)
{
	ushort sr = spl7();

	*ATARI_ETHERNAT_CPLD |= ATARI_ETHERNAT_CPLD_LAN;

	spl(sr);
}


/*
 * netif_carrier_*() and netif_*_queue(). Linux stops calling
 * smc_hard_start_xmit() while the queue is stopped and its qdisc keeps the
 * packets. Here ethernat_output() puts them into dev->snd and
 * netif_wake_queue() feeds them to the chip again.
 */

long netif_carrier_ok(struct netif *dev)
{
	struct smc_local *lp = dev->data;

	return lp->carrier;
}

void netif_carrier_on(struct netif *dev)
{
	struct smc_local *lp = dev->data;

	lp->carrier = 1;
}

void netif_carrier_off(struct netif *dev)
{
	struct smc_local *lp = dev->data;

	lp->carrier = 0;
}

static inline void netif_trans_update(struct netif *dev)
{
	struct smc_local *lp = dev->data;

	lp->trans_start = jiffies;
}

static inline long netif_queue_stopped(struct netif *dev)
{
	struct smc_local *lp = dev->data;

	return lp->queue_stopped;
}

static void netif_start_queue(struct netif *dev)
{
	struct smc_local *lp = dev->data;

	lp->queue_stopped = 0;
	netif_trans_update(dev);
}

static void netif_stop_queue(struct netif *dev)
{
	struct smc_local *lp = dev->data;

	lp->queue_stopped = 1;
}

/*
 * Runs from process context and from the interrupt handler, where
 * smc_hardware_send_pkt() wakes the queue: a nested call only leaves a
 * note for the loop that is already running.
 */
static void netif_wake_queue(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	BUF *skb;
	ushort sr;

	sr = spl7();
	lp->queue_stopped = 0;
	if (lp->pumping)
	{
		lp->wake_pending = 1;
		spl(sr);
		return;
	}
	lp->pumping = 1;

	do
	{
		lp->wake_pending = 0;
		spl(sr);

		while (!lp->queue_stopped && (skb = if_dequeue(&dev->snd)) != NULL)
			smc_hard_start_xmit(skb, dev);

		sr = spl7();
	}
	while (lp->wake_pending && !lp->queue_stopped);

	lp->pumping = 0;
	spl(sr);
}


/*
 * this does a soft reset on the device
 */
static void smc_reset(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	long ctl, cfg;
	BUF *pending_skb;

	DBG(2, dev, "%s\n", __func__);

	/* Disable all interrupts, block TX tasklet */
	spin_lock_irq(&lp->lock);
	SMC_SELECT_BANK(lp, 2);
	SMC_SET_INT_MASK(lp, 0);
	pending_skb = lp->pending_tx_skb;
	lp->pending_tx_skb = NULL;
	spin_unlock_irq(&lp->lock);

	/* free any pending tx skb */
	if (pending_skb) {
		buf_deref(pending_skb, BUF_ATOMIC);
		dev->out_errors++;
	}

	/*
	 * This resets the registers mostly to defaults, but doesn't
	 * affect EEPROM.  That seems unnecessary
	 */
	SMC_SELECT_BANK(lp, 0);
	SMC_SET_RCR(lp, RCR_SOFTRST);

	/*
	 * Setup the Configuration Register
	 * This is necessary because the CONFIG_REG is not affected
	 * by a soft reset
	 */
	SMC_SELECT_BANK(lp, 1);

	cfg = CONFIG_DEFAULT;

	/*
	 * Setup for fast accesses if requested.  If the card/system
	 * can't handle it then there will be no recovery except for
	 * a hard reset or power cycle
	 */
	if (lp->cfg.flags & SMC91X_NOWAIT)
		cfg |= CONFIG_NO_WAIT;

	/*
	 * Release from possible power-down state
	 * Configuration register is not affected by Soft Reset
	 */
	cfg |= CONFIG_EPH_POWER_EN;

	SMC_SET_CONFIG(lp, cfg);

	/* this should pause enough for the chip to be happy */
	/*
	 * elaborate?  What does the chip _need_? --jgarzik
	 *
	 * This seems to be undocumented, but something the original
	 * driver(s) have always done.  Suspect undocumented timing
	 * info/determined empirically. --rmk
	 */
	udelay(1);

	/* Disable transmit and receive functionality */
	SMC_SELECT_BANK(lp, 0);
	SMC_SET_RCR(lp, RCR_CLEAR);
	SMC_SET_TCR(lp, TCR_CLEAR);

	SMC_SELECT_BANK(lp, 1);
	ctl = SMC_GET_CTL(lp) | CTL_LE_ENABLE;

	/*
	 * Set the control register to automatically release successfully
	 * transmitted packets, to make the best use out of our limited
	 * memory
	 */
	if(!THROTTLE_TX_PKTS)
		ctl |= CTL_AUTO_RELEASE;
	else
		ctl &= ~CTL_AUTO_RELEASE;
	SMC_SET_CTL(lp, ctl);

	/* Reset the MMU */
	SMC_SELECT_BANK(lp, 2);
	SMC_SET_MMU_CMD(lp, MC_RESET);
	SMC_WAIT_MMU_BUSY(lp);
}

/*
 * Enable Interrupts, Receive, and Transmit
 */
static void smc_enable(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	long mask;
	const u8 *dev_addr = dev->hwlocal.adr.bytes;

	DBG(2, dev, "%s\n", __func__);

	/* see the header file for options in TCR/RCR DEFAULT */
	SMC_SELECT_BANK(lp, 0);
	SMC_SET_TCR(lp, lp->tcr_cur_mode);
	SMC_SET_RCR(lp, lp->rcr_cur_mode);

	SMC_SELECT_BANK(lp, 1);
	SMC_SET_MAC_ADDR(lp, dev_addr);

	/* now, enable interrupts */
	mask = IM_EPH_INT|IM_RX_OVRN_INT|IM_RCV_INT;
	if (lp->version >= (CHIP_91100 << 4))
		mask |= IM_MDINT;
	SMC_SELECT_BANK(lp, 2);
	SMC_SET_INT_MASK(lp, mask);

	/*
	 * From this point the register bank must _NOT_ be switched away
	 * to something else than bank 2 without proper locking against
	 * races with any tasklet or interrupt handlers until smc_shutdown()
	 * or smc_reset() is called.
	 */
}

/*
 * this puts the device in an inactive state
 */
static void smc_shutdown(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	BUF *pending_skb;

	DBG(2, dev, "%s: %s\n", CARDNAME, __func__);

	/* no more interrupts for me */
	spin_lock_irq(&lp->lock);
	SMC_SELECT_BANK(lp, 2);
	SMC_SET_INT_MASK(lp, 0);
	pending_skb = lp->pending_tx_skb;
	lp->pending_tx_skb = NULL;
	spin_unlock_irq(&lp->lock);
	if (pending_skb)
		buf_deref(pending_skb, BUF_ATOMIC);

	/* and tell the card to stay away from that nasty outside world */
	SMC_SELECT_BANK(lp, 0);
	SMC_SET_RCR(lp, RCR_CLEAR);
	SMC_SET_TCR(lp, TCR_CLEAR);

#ifdef POWER_DOWN
	/* finally, shut the chip down */
	SMC_SELECT_BANK(lp, 1);
	SMC_SET_CONFIG(lp, SMC_GET_CONFIG(lp) & ~CONFIG_EPH_POWER_EN);
#endif
}

/*
 * This is the procedure to handle the receipt of a packet.
 */
static inline void  smc_rcv(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	u32 packet_number, status, packet_len;

	DBG(3, dev, "%s\n", __func__);

	packet_number = SMC_GET_RXFIFO(lp);
	if (unlikely(packet_number & RXFIFO_REMPTY)) {
		PRINTK(dev, "smc_rcv with nothing on FIFO.\n");
		return;
	}

	/* read from start of packet */
	SMC_SET_PTR(lp, PTR_READ | PTR_RCV | PTR_AUTOINC);

	/* First two words are status and packet length */
	SMC_GET_PKT_HDR(lp, status, packet_len);
	packet_len &= 0x07ff;  /* mask off top bits */
	DBG(2, dev, "RX PNR 0x%x STATUS 0x%04x LENGTH 0x%04x (%d)\n",
	    packet_number, status, packet_len, packet_len);

	back:
	if (unlikely(packet_len < 6 || status & RS_ERRORS)) {
		if (status & RS_TOOLONG && packet_len <= (1514 + 4 + 6)) {
			/* accept VLAN packets */
			status &= ~RS_TOOLONG;
			goto back;
		}
		if (packet_len < 6) {
			/* bloody hardware */
			PRINTK(dev, "fubar (rxlen %u status %x\n",
				   packet_len, status);
			status |= RS_TOOSHORT;
		}
		SMC_WAIT_MMU_BUSY(lp);
		SMC_SET_MMU_CMD(lp, MC_RELEASE);
		/* rx_errors, and rx_frame_errors, rx_length_errors or
		 * rx_crc_errors on Linux
		 */
		dev->in_errors++;
	} else {
		BUF *skb;
		unsigned char *data;
		u32 data_len;
		short type;

		/*
		 * Actual payload is packet_len - 6 (or 5 if odd byte).
		 * We want skb_reserve(2) and the final ctrl word
		 * (2 bytes, possibly containing the payload odd byte).
		 * Furthermore, we add 2 bytes to allow rounding up to
		 * multiple of 4 bytes on 32 bit buses.
		 * Hence packet_len - 6 + 2 + 2 + 2.
		 */
		skb = buf_alloc(packet_len + 32, 16, BUF_ATOMIC);
		if (unlikely(skb == NULL)) {
			SMC_WAIT_MMU_BUSY(lp);
			SMC_SET_MMU_CMD(lp, MC_RELEASE);
			dev->in_errors++;
			return;
		}

		/* Align IP header to 32 bits */
		skb->dstart = (char *)(((ulong) skb->dstart + 3) & ~3UL) + 2;
		skb->dend = skb->dstart;

		/* BUG: the LAN91C111 rev A never sets this bit. Force it. */
		if (lp->version == 0x90)
			status |= RS_ODDFRAME;

		/*
		 * If odd length: packet_len - 5,
		 * otherwise packet_len - 6.
		 * With the trailing ctrl byte it's packet_len - 4.
		 */
		data_len = packet_len - ((status & RS_ODDFRAME) ? 5 : 6);
		data = (unsigned char *) skb->dend;
		skb->dend += data_len;
		SMC_PULL_DATA(lp, data, packet_len - 4);

		SMC_WAIT_MMU_BUSY(lp);
		SMC_SET_MMU_CMD(lp, MC_RELEASE);

		PRINT_PKT(data, packet_len - 4);

		/* eth_type_trans(): the packet filter sees the frame before
		 * the header goes
		 */
		if (dev->bpf)
			bpf_input(dev, skb);
		type = eth_remove_hdr(skb);

		/* netif_rx(); if_input() frees the buffer when its queue is
		 * full
		 */
		if (if_input(dev, skb, 0, type))
			dev->in_errors++;
		else
			dev->in_packets++;
	}
}

/*
 * This is called to actually send a packet to the chip.
 *
 * On Linux this is a tasklet; here it is called directly, at the end of
 * the interrupt handler or from smc_hard_start_xmit(), so there is no
 * lock to try.
 */
static void smc_hardware_send_pkt(struct smc_local *lp)
{
	struct netif *dev = lp->dev;
	char *ioaddr = lp->base;
	BUF *skb;
	u32 packet_no, len;
	unsigned char *buf;

	DBG(3, dev, "%s\n", __func__);

	skb = lp->pending_tx_skb;
	if (unlikely(!skb)) {
		return;
	}
	lp->pending_tx_skb = NULL;

	packet_no = SMC_GET_AR(lp);
	if (unlikely(packet_no & AR_FAILED)) {
		PRINTK(dev, "Memory allocation failed.\n");
		dev->out_errors++;
		goto done;
	}

	/* point to the beginning of the packet */
	SMC_SET_PN(lp, packet_no);
	SMC_SET_PTR(lp, PTR_AUTOINC);

	buf = (unsigned char *) skb->dstart;
	len = skb->dend - skb->dstart;
	DBG(2, dev, "TX PNR 0x%x LENGTH 0x%04x (%d) BUF 0x%p\n",
	    packet_no, len, len, buf);
	PRINT_PKT(buf, len);

	/*
	 * Send the packet length (+6 for status words, length, and ctl.
	 * The card will pad to 64 bytes with zeroes if packet is too small.
	 */
	SMC_PUT_PKT_HDR(lp, 0, len + 6);

	/* send the actual data */
	SMC_PUSH_DATA(lp, (char *) buf, len & ~1);

	/* Send final ctl word with the last byte if there is one */
	SMC_outw(lp, ((len & 1) ? (0x2000 | buf[len - 1]) : 0), ioaddr,
		 DATA_REG(lp));

	/*
	 * If THROTTLE_TX_PKTS is set, we stop the queue here. This will
	 * have the effect of having at most one packet queued for TX
	 * in the chip's memory at all time.
	 *
	 * If THROTTLE_TX_PKTS is not set then the queue is stopped only
	 * when memory allocation (MC_ALLOC) does not succeed right away.
	 */
	if (THROTTLE_TX_PKTS)
		netif_stop_queue(dev);

	/* queue the packet for TX */
	SMC_SET_MMU_CMD(lp, MC_ENQUEUE);

	netif_trans_update(dev);
	dev->out_packets++;

	SMC_ENABLE_INT(lp, IM_TX_INT | IM_TX_EMPTY_INT);

done:	if (!THROTTLE_TX_PKTS)
		netif_wake_queue(dev);

	buf_deref(skb, BUF_ATOMIC);
}

/*
 * Since I am not sure if I will have enough room in the chip's ram
 * to store the packet, I call this routine which either sends it
 * now, or set the card to generates an interrupt when ready
 * for the packet.
 *
 * The buffer is the Linux skb: dstart to dend hold the complete Ethernet
 * frame. It is consumed here, whether it was sent or dropped.
 */
static long
smc_hard_start_xmit(BUF *skb, struct netif *dev)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	u32 numPages, poll_count, status;
	u32 len = skb->dend - skb->dstart;

	DBG(3, dev, "%s\n", __func__);

	/* BUG_ON(lp->pending_tx_skb != NULL): the queue is stopped while a
	 * packet waits for memory, so this cannot happen; keep the packet
	 * rather than lose it if it ever does
	 */
	if (lp->pending_tx_skb != NULL) {
		if (if_enqueue(&dev->snd, skb, skb->info))
			dev->out_errors++;
		return NETDEV_TX_OK;
	}

	/*
	 * The MMU wants the number of pages to be the number of 256 bytes
	 * 'pages', minus 1 (since a packet can't ever have 0 pages :))
	 *
	 * The 91C111 ignores the size bits, but earlier models don't.
	 *
	 * Pkt size for allocating is data length +6 (for additional status
	 * words, length and ctl)
	 *
	 * If odd size then last byte is included in ctl word.
	 */
	numPages = ((len & ~1) + (6 - 1)) >> 8;
	if (unlikely(numPages > 7)) {
		PRINTK(dev, "Far too big packet error.\n");
		dev->out_errors++;
		buf_deref(skb, BUF_ATOMIC);
		return NETDEV_TX_OK;
	}

	/* now, try to allocate the memory */
	SMC_SET_MMU_CMD(lp, MC_ALLOC | numPages);

	/*
	 * Poll the chip for a short amount of time in case the
	 * allocation succeeds quickly.
	 */
	poll_count = MEMORY_WAIT_TIME;
	do {
		status = SMC_GET_INT(lp);
		if (status & IM_ALLOC_INT) {
			SMC_ACK_INT(lp, IM_ALLOC_INT);
  			break;
		}
   	} while (--poll_count);

	lp->pending_tx_skb = skb;
	if (!poll_count) {
		/* oh well, wait until the chip finds memory later */
		netif_stop_queue(dev);
		DBG(2, dev, "TX memory allocation deferred.\n");
		SMC_ENABLE_INT(lp, IM_ALLOC_INT);
	} else {
		/*
		 * Allocation succeeded: push packet to the chip's own memory
		 * immediately.
		 */
		smc_hardware_send_pkt(lp);
	}

	return NETDEV_TX_OK;
}

/*
 * This handles a TX interrupt, which is only called when:
 * - a TX error occurred, or
 * - CTL_AUTO_RELEASE is not set and TX of a packet completed.
 */
static void smc_tx(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	u32 saved_packet, packet_no, tx_status;
	u32 pkt_len;

	DBG(3, dev, "%s\n", __func__);

	/* If the TX FIFO is empty then nothing to do */
	packet_no = SMC_GET_TXFIFO(lp);
	if (unlikely(packet_no & TXFIFO_TEMPTY)) {
		PRINTK(dev, "smc_tx with nothing on FIFO.\n");
		return;
	}

	/* select packet to read from */
	saved_packet = SMC_GET_PN(lp);
	SMC_SET_PN(lp, packet_no);

	/* read the first word (status word) from this packet */
	SMC_SET_PTR(lp, PTR_AUTOINC | PTR_READ);
	SMC_GET_PKT_HDR(lp, tx_status, pkt_len);
	UNUSED(pkt_len);
	DBG(2, dev, "TX STATUS 0x%04x PNR 0x%02x\n",
	    tx_status, packet_no);

	if (!(tx_status & ES_TX_SUC))
		dev->out_errors++;

	if (tx_status & (ES_LATCOL | ES_16COL)) {
		PRINTK(dev, "%s occurred on last xmit\n",
		       (tx_status & ES_LATCOL) ?
			"late collision" : "too many collisions");
		/* tx_window_errors */
		dev->collisions++;
	}

	/* kill the packet */
	SMC_WAIT_MMU_BUSY(lp);
	SMC_SET_MMU_CMD(lp, MC_FREEPKT);

	/* Don't restore Packet Number Reg until busy bit is cleared */
	SMC_WAIT_MMU_BUSY(lp);
	SMC_SET_PN(lp, saved_packet);

	/* re-enable transmit */
	SMC_SELECT_BANK(lp, 0);
	SMC_SET_TCR(lp, lp->tcr_cur_mode);
	SMC_SELECT_BANK(lp, 2);
}


/*---PHY CONTROL AND CONFIGURATION-----------------------------------------*/

static void smc_mii_out(struct netif *dev, u32 val, long bits)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	u32 mii_reg, mask;

	mii_reg = SMC_GET_MII(lp) & ~(MII_MCLK | MII_MDOE | MII_MDO);
	mii_reg |= MII_MDOE;

	for (mask = 1UL << (bits - 1); mask; mask >>= 1) {
		if (val & mask)
			mii_reg |= MII_MDO;
		else
			mii_reg &= ~MII_MDO;

		SMC_SET_MII(lp, mii_reg);
		udelay(MII_DELAY);
		SMC_SET_MII(lp, mii_reg | MII_MCLK);
		udelay(MII_DELAY);
	}
}

static u32 smc_mii_in(struct netif *dev, long bits)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	u32 mii_reg, mask, val;

	mii_reg = SMC_GET_MII(lp) & ~(MII_MCLK | MII_MDOE | MII_MDO);
	SMC_SET_MII(lp, mii_reg);

	for (mask = 1UL << (bits - 1), val = 0; mask; mask >>= 1) {
		if (SMC_GET_MII(lp) & MII_MDI)
			val |= mask;

		SMC_SET_MII(lp, mii_reg);
		udelay(MII_DELAY);
		SMC_SET_MII(lp, mii_reg | MII_MCLK);
		udelay(MII_DELAY);
	}

	return val;
}

/*
 * Reads a register from the MII Management serial interface
 */
static long smc_phy_read(struct netif *dev, long phyaddr, long phyreg)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	u32 phydata;

	SMC_SELECT_BANK(lp, 3);

	/* Idle - 32 ones */
	smc_mii_out(dev, 0xffffffffUL, 32);

	/* Start code (01) + read (10) + phyaddr + phyreg */
	smc_mii_out(dev, 6UL << 10 | (u32) phyaddr << 5 | phyreg, 14);

	/* Turnaround (2bits) + phydata */
	phydata = smc_mii_in(dev, 18);

	/* Return to idle state */
	SMC_SET_MII(lp, SMC_GET_MII(lp) & ~(MII_MCLK|MII_MDOE|MII_MDO));

	DBG(3, dev, "%s: phyaddr=0x%x, phyreg=0x%x, phydata=0x%x\n",
	    __func__, phyaddr, phyreg, phydata);

	SMC_SELECT_BANK(lp, 2);
	return phydata;
}

/*
 * Writes a register to the MII Management serial interface
 */
static void smc_phy_write(struct netif *dev, long phyaddr, long phyreg,
			  long phydata)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;

	SMC_SELECT_BANK(lp, 3);

	/* Idle - 32 ones */
	smc_mii_out(dev, 0xffffffffUL, 32);

	/* Start code (01) + write (01) + phyaddr + phyreg + turnaround + phydata */
	smc_mii_out(dev, 5UL << 28 | (u32) phyaddr << 23 | (u32) phyreg << 18 | 2UL << 16 | (u32) phydata, 32);

	/* Return to idle state */
	SMC_SET_MII(lp, SMC_GET_MII(lp) & ~(MII_MCLK|MII_MDOE|MII_MDO));

	DBG(3, dev, "%s: phyaddr=0x%x, phyreg=0x%x, phydata=0x%x\n",
	    __func__, phyaddr, phyreg, phydata);

	SMC_SELECT_BANK(lp, 2);
}

/*
 * Finds and reports the PHY address
 */
static void smc_phy_detect(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	long phyaddr;

	DBG(2, dev, "%s\n", __func__);

	lp->phy_type = 0;

	/*
	 * Scan all 32 PHY addresses if necessary, starting at
	 * PHY#1 to PHY#31, and then PHY#0 last.
	 */
	for (phyaddr = 1; phyaddr < 33; ++phyaddr) {
		u32 id1, id2;

		/* Read the PHY identifiers */
		id1 = smc_phy_read(dev, phyaddr & 31, MII_PHYSID1);
		id2 = smc_phy_read(dev, phyaddr & 31, MII_PHYSID2);

		DBG(3, dev, "phy_id1=0x%x, phy_id2=0x%x\n",
		    id1, id2);

		/* Make sure it is a valid identifier */
		if (id1 != 0x0000 && id1 != 0xffff && id1 != 0x8000 &&
		    id2 != 0x0000 && id2 != 0xffff && id2 != 0x8000) {
			/* Save the PHY's address */
			lp->mii.phy_id = phyaddr & 31;
			lp->phy_type = id1 << 16 | id2;
			break;
		}
	}
}

/*
 * Sets the PHY to a configuration as determined by the user
 */
static long smc_phy_fixed(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	long phyaddr = lp->mii.phy_id;
	long bmcr, cfg1;

	DBG(3, dev, "%s\n", __func__);

	/* Enter Link Disable state */
	cfg1 = smc_phy_read(dev, phyaddr, PHY_CFG1_REG);
	cfg1 |= PHY_CFG1_LNKDIS;
	smc_phy_write(dev, phyaddr, PHY_CFG1_REG, cfg1);

	/*
	 * Set our fixed capabilities
	 * Disable auto-negotiation
	 */
	bmcr = 0;

	if (lp->ctl_rfduplx)
		bmcr |= BMCR_FULLDPLX;

	if (lp->ctl_rspeed == 100)
		bmcr |= BMCR_SPEED100;

	/* Write our capabilities to the phy control register */
	smc_phy_write(dev, phyaddr, MII_BMCR, bmcr);

	/* Re-Configure the Receive/Phy Control register */
	SMC_SELECT_BANK(lp, 0);
	SMC_SET_RPC(lp, lp->rpc_cur_mode);
	SMC_SELECT_BANK(lp, 2);

	return 1;
}

/**
 * smc_phy_reset - reset the phy
 * @dev: net device
 * @phy: phy address
 *
 * Issue a software reset for the specified PHY and
 * wait up to 100ms for the reset to complete.  We should
 * not access the PHY for 50ms after issuing the reset.
 *
 * The time to wait appears to be dependent on the PHY.
 *
 * Must be called with lp->lock locked.
 */
static long smc_phy_reset(struct netif *dev, long phy)
{
	struct smc_local *lp = dev->data;
	u32 bmcr;
	long timeout;

	smc_phy_write(dev, phy, MII_BMCR, BMCR_RESET);

	for (timeout = 2; timeout; timeout--) {
		spin_unlock_irq(&lp->lock);
		nap(50);
		spin_lock_irq(&lp->lock);

		bmcr = smc_phy_read(dev, phy, MII_BMCR);
		if (!(bmcr & BMCR_RESET))
			break;
	}

	return bmcr & BMCR_RESET;
}

/**
 * smc_phy_powerdown - powerdown phy
 * @dev: net device
 *
 * Power down the specified PHY
 */
static void smc_phy_powerdown(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	u32 bmcr;
	long phy = lp->mii.phy_id;

	if (lp->phy_type == 0)
		return;

	/* We need to ensure that no calls to smc_phy_configure are
	   pending.
	*/
	lp->work_pending = 0;

	bmcr = smc_phy_read(dev, phy, MII_BMCR);
	smc_phy_write(dev, phy, MII_BMCR, bmcr | BMCR_PDOWN);
}


/**
 * smc_phy_check_media - check the media status and adjust TCR
 * @dev: net device
 * @init: set true for initialisation
 *
 * Select duplex mode depending on negotiation state.  This
 * also updates our carrier state.
 */
static void smc_phy_check_media(struct netif *dev, long init)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;

	if (mii_check_media(&lp->mii, 1, init)) {
		/* duplex state has changed */
		if (lp->mii.full_duplex) {
			lp->tcr_cur_mode |= TCR_SWFDUP;
		} else {
			lp->tcr_cur_mode &= ~TCR_SWFDUP;
		}

		SMC_SELECT_BANK(lp, 0);
		SMC_SET_TCR(lp, lp->tcr_cur_mode);
	}
}

/*
 * Configures the specified PHY through the MII management interface
 * using Autonegotiation.
 * Calls smc_phy_fixed() if the user has requested a certain config.
 * If RPC ANEG bit is set, the media selection is dependent purely on
 * the selection by the MII (either in the MII BMCR reg or the result
 * of autonegotiation.)  If the RPC ANEG bit is cleared, the selection
 * is controlled by the RPC SPEED and RPC DPLX bits.
 */
static void smc_phy_configure(struct smc_local *lp)
{
	struct netif *dev = lp->dev;
	char *ioaddr = lp->base;
	long phyaddr = lp->mii.phy_id;
	long my_phy_caps; /* My PHY capabilities */
	long my_ad_caps; /* My Advertised capabilities */

	DBG(3, dev, "smc_program_phy()\n");

	spin_lock_irq(&lp->lock);

	/*
	 * We should not be called if phy_type is zero.
	 */
	if (lp->phy_type == 0)
		goto smc_phy_configure_exit;

	if (smc_phy_reset(dev, phyaddr)) {
		PRINTK(dev, "PHY reset timed out\n");
		goto smc_phy_configure_exit;
	}

	/*
	 * Enable PHY Interrupts (for register 18)
	 * Interrupts listed here are disabled
	 */
	smc_phy_write(dev, phyaddr, PHY_MASK_REG,
		PHY_INT_LOSSSYNC | PHY_INT_CWRD | PHY_INT_SSD |
		PHY_INT_ESD | PHY_INT_RPOL | PHY_INT_JAB |
		PHY_INT_SPDDET | PHY_INT_DPLXDET);

	/* Configure the Receive/Phy Control register */
	SMC_SELECT_BANK(lp, 0);
	SMC_SET_RPC(lp, lp->rpc_cur_mode);

	/* If the user requested no auto neg, then go set his request */
	if (lp->mii.force_media) {
		smc_phy_fixed(dev);
		goto smc_phy_configure_exit;
	}

	/* Copy our capabilities from MII_BMSR to MII_ADVERTISE */
	my_phy_caps = smc_phy_read(dev, phyaddr, MII_BMSR);

	if (!(my_phy_caps & BMSR_ANEGCAPABLE)) {
		PRINTK(dev, "Auto negotiation NOT supported\n");
		smc_phy_fixed(dev);
		goto smc_phy_configure_exit;
	}

	my_ad_caps = ADVERTISE_CSMA; /* I am CSMA capable */

	if (my_phy_caps & BMSR_100BASE4)
		my_ad_caps |= ADVERTISE_100BASE4;
	if (my_phy_caps & BMSR_100FULL)
		my_ad_caps |= ADVERTISE_100FULL;
	if (my_phy_caps & BMSR_100HALF)
		my_ad_caps |= ADVERTISE_100HALF;
	if (my_phy_caps & BMSR_10FULL)
		my_ad_caps |= ADVERTISE_10FULL;
	if (my_phy_caps & BMSR_10HALF)
		my_ad_caps |= ADVERTISE_10HALF;

	/* Disable capabilities not selected by our user */
	if (lp->ctl_rspeed != 100)
		my_ad_caps &= ~(ADVERTISE_100BASE4|ADVERTISE_100FULL|ADVERTISE_100HALF);

	if (!lp->ctl_rfduplx)
		my_ad_caps &= ~(ADVERTISE_100FULL|ADVERTISE_10FULL);

	/* Update our Auto-Neg Advertisement Register */
	smc_phy_write(dev, phyaddr, MII_ADVERTISE, my_ad_caps);
	lp->mii.advertising = my_ad_caps;

	/*
	 * Read the register back.  Without this, it appears that when
	 * auto-negotiation is restarted, sometimes it isn't ready and
	 * the link does not come up.
	 */
	smc_phy_read(dev, phyaddr, MII_ADVERTISE);

	DBG(2, dev, "phy caps=%x\n", my_phy_caps);
	DBG(2, dev, "phy advertised caps=%x\n", my_ad_caps);

	/* Restart auto-negotiation process in order to advertise my caps */
	smc_phy_write(dev, phyaddr, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);

	smc_phy_check_media(dev, 1);

smc_phy_configure_exit:
	SMC_SELECT_BANK(lp, 2);
	spin_unlock_irq(&lp->lock);
}

/* the phy_configure work item: a kernel thread, which may sleep */
static void _cdecl smc_phy_configure_work(void *arg)
{
	struct smc_local *lp = arg;

	if (lp->work_pending && (lp->dev->flags & IFF_UP))
		smc_phy_configure(lp);
	lp->work_pending = 0;

	kthread_exit(0);
}

static void schedule_work(struct smc_local *lp)
{
	if (lp->work_pending)
		return;

	lp->work_pending = 1;
	if (kthread_create(smc_phy_configure_work, lp, NULL, "ethernat_phy") < 0)
		lp->work_pending = 0;
}

/*
 * smc_phy_interrupt
 *
 * Purpose:  Handle interrupts relating to PHY register 18. This is
 *  called from the "hard" interrupt handler under our private spinlock.
 */
static void smc_phy_interrupt(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	long phyaddr = lp->mii.phy_id;
	long phy18;

	DBG(2, dev, "%s\n", __func__);

	if (lp->phy_type == 0)
		return;

	for(;;) {
		smc_phy_check_media(dev, 0);

		/* Read PHY Register 18, Status Output */
		phy18 = smc_phy_read(dev, phyaddr, PHY_INT_REG);
		if ((phy18 & PHY_INT_INT) == 0)
			break;
	}
}

/*--- END PHY CONTROL AND CONFIGURATION-------------------------------------*/

static void smc_10bt_check_media(struct netif *dev, long init)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	u32 old_carrier, new_carrier;

	old_carrier = netif_carrier_ok(dev) ? 1 : 0;

	SMC_SELECT_BANK(lp, 0);
	new_carrier = (SMC_GET_EPH_STATUS(lp) & ES_LINK_OK) ? 1 : 0;
	SMC_SELECT_BANK(lp, 2);

	if (init || (old_carrier != new_carrier)) {
		if (!new_carrier) {
			netif_carrier_off(dev);
		} else {
			netif_carrier_on(dev);
		}
		mii_netdev_info(&lp->mii, "link %s",
			    new_carrier ? "up" : "down");
	}
}

static void smc_eph_interrupt(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	u32 ctl;

	smc_10bt_check_media(dev, 0);

	SMC_SELECT_BANK(lp, 1);
	ctl = SMC_GET_CTL(lp);
	SMC_SET_CTL(lp, ctl & ~CTL_LE_ENABLE);
	SMC_SET_CTL(lp, ctl);
	SMC_SELECT_BANK(lp, 2);
}

/*
 * This is the main routine of the driver, to handle the device when
 * it needs some attention.
 *
 * Called from ethernat_interrupt at IPL 6.
 */
long _cdecl smc_interrupt(void)
{
	struct netif *dev = &if_ethernat;
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	long status, mask, timeout, card_stats;
	long saved_pointer;

	DBG(3, dev, "%s\n", __func__);

	spin_lock(&lp->lock);

	saved_pointer = SMC_GET_PTR(lp);
	mask = SMC_GET_INT_MASK(lp);
	SMC_SET_INT_MASK(lp, 0);

	/* set a timeout value, so I don't stay here forever */
	timeout = MAX_IRQ_LOOPS;

	do {
		status = SMC_GET_INT(lp);

		DBG(2, dev, "INT 0x%02x MASK 0x%02x\n", status, mask);

		status &= mask;
		if (!status)
			break;

		if (status & IM_TX_INT) {
			/* do this before RX as it will free memory quickly */
			DBG(3, dev, "TX int\n");
			smc_tx(dev);
			SMC_ACK_INT(lp, IM_TX_INT);
			if (THROTTLE_TX_PKTS)
				netif_wake_queue(dev);
		} else if (status & IM_RCV_INT) {
			DBG(3, dev, "RX irq\n");
			smc_rcv(dev);
		} else if (status & IM_ALLOC_INT) {
			DBG(3, dev, "Allocation irq\n");
			/* tasklet_hi_schedule(&lp->tx_task) */
			lp->tx_task = 1;
			mask &= ~IM_ALLOC_INT;
		} else if (status & IM_TX_EMPTY_INT) {
			DBG(3, dev, "TX empty\n");
			mask &= ~IM_TX_EMPTY_INT;

			/* update stats */
			SMC_SELECT_BANK(lp, 0);
			card_stats = SMC_GET_COUNTER(lp);
			SMC_SELECT_BANK(lp, 2);

			/* single collisions */
			dev->collisions += card_stats & 0xF;
			card_stats >>= 4;

			/* multiple collisions */
			dev->collisions += card_stats & 0xF;
		} else if (status & IM_RX_OVRN_INT) {
			DBG(1, dev, "RX overrun\n");
			SMC_ACK_INT(lp, IM_RX_OVRN_INT);
			/* rx_errors, rx_fifo_errors */
			dev->in_errors++;
		} else if (status & IM_EPH_INT) {
			smc_eph_interrupt(dev);
		} else if (status & IM_MDINT) {
			SMC_ACK_INT(lp, IM_MDINT);
			smc_phy_interrupt(dev);
		} else if (status & IM_ERCV_INT) {
			SMC_ACK_INT(lp, IM_ERCV_INT);
			PRINTK(dev, "UNSUPPORTED: ERCV INTERRUPT\n");
		}
	} while (--timeout);

	/* restore register states */
	SMC_SET_PTR(lp, saved_pointer);
	SMC_SET_INT_MASK(lp, mask);
	spin_unlock(&lp->lock);

	/* the tasklet runs right after the handler */
	if (lp->tx_task) {
		lp->tx_task = 0;
		smc_hardware_send_pkt(lp);
	}

	DBG(3, dev, "Interrupt done (%d loops)\n",
	    MAX_IRQ_LOOPS - timeout);

	/*
	 * We return IRQ_HANDLED unconditionally here even if there was
	 * nothing to do.  There is a possibility that a packet might
	 * get enqueued into the chip right after TX_EMPTY_INT is raised
	 * but just before the CPU acknowledges the IRQ.
	 * Better take an unneeded IRQ in some occasions than complexifying
	 * the code for all cases.
	 */
	return IRQ_HANDLED;
}

/* Our watchdog timed out. Called by the networking layer */
static void smc_timeout(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	long status, mask, eph_st, meminfo, fifo;

	DBG(2, dev, "%s\n", __func__);

	spin_lock_irq(&lp->lock);
	status = SMC_GET_INT(lp);
	mask = SMC_GET_INT_MASK(lp);
	fifo = SMC_GET_FIFO(lp);
	SMC_SELECT_BANK(lp, 0);
	eph_st = SMC_GET_EPH_STATUS(lp);
	meminfo = SMC_GET_MIR(lp);
	SMC_SELECT_BANK(lp, 2);
	spin_unlock_irq(&lp->lock);
	ALERT(("%s%d: TX timeout (INT 0x%02lx INTMASK 0x%02lx MEM 0x%04lx FIFO 0x%04lx EPH_ST 0x%04lx)",
	       dev->name, dev->unit, status, mask, meminfo, fifo, eph_st));

	smc_reset(dev);
	smc_enable(dev);

	/*
	 * Reconfiguring the PHY doesn't seem like a bad idea here, but
	 * smc_phy_configure() calls msleep() which calls schedule_timeout()
	 * which calls schedule().  Hence we use a work queue.
	 */
	if (lp->phy_type != 0)
		schedule_work(lp);

	/* We can accept TX packets again */
	netif_trans_update(dev); /* prevent tx timeout */
	netif_wake_queue(dev);
}

/*
 * crc32_le() of lib/crc32.c: the little endian CRC-32 without the
 * final inversion, as smc_set_multicast_list() uses it.
 */
static u32 crc32_le(u32 crc, const u8 *p, long len)
{
	while (len-- > 0) {
		long i;

		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320UL : 0);
	}

	return crc;
}

/*
 * This routine will, depending on the values passed to it,
 * either make it accept multicast packets, go into
 * promiscuous mode (for TCPDUMP and cousins) or accept
 * a select set of multicast packets
 *
 * The multicast list is the one ethernat_igmp_mac_filter() keeps: the IP
 * groups joined, mapped to their Ethernet addresses here.
 */
static void smc_set_multicast_list(struct netif *dev)
{
	struct smc_local *lp = dev->data;
	char *ioaddr = lp->base;
	unsigned char multicast_table[8];
	long update_multicast = 0;
	long mc_count = lp->mc_overflow ? 17 : lp->mc_count;

	DBG(2, dev, "%s\n", __func__);

	if (dev->flags & IFF_PROMISC) {
		DBG(2, dev, "RCR_PRMS\n");
		lp->rcr_cur_mode |= RCR_PRMS;
	}

/* BUG?  I never disable promiscuous mode if multicasting was turned on.
   Now, I turn off promiscuous mode, but I don't do anything to multicasting
   when promiscuous mode is turned on.
*/

	/*
	 * Here, I am setting this to accept all multicast packets.
	 * I don't need to zero the multicast table, because the flag is
	 * checked before the table is
	 */
	else if (dev->flags & IFF_ALLMULTI || mc_count > 16) {
		DBG(2, dev, "RCR_ALMUL\n");
		lp->rcr_cur_mode |= RCR_ALMUL;
	}

	/*
	 * This sets the internal hardware table to filter out unwanted
	 * multicast packets before they take up memory.
	 *
	 * The SMC chip uses a hash table where the high 6 bits of the CRC of
	 * address are the offset into the table.  If that bit is 1, then the
	 * multicast packet is accepted.  Otherwise, it's dropped silently.
	 *
	 * To use the 6 bits as an offset into the table, the high 3 bits are
	 * the number of the 8 bit register, while the low 3 bits are the bit
	 * within that register.
	 */
	else if (mc_count) {
		long i;

		/* table for flipping the order of 3 bits */
		static const unsigned char invert3[] = {0, 4, 2, 6, 1, 5, 3, 7};

		/* start with a table of all zeros: reject all */
		memset(multicast_table, 0, sizeof(multicast_table));

		for (i = 0; i < 16; i++) {
			u8 addr[ETH_ALEN];
			ulong group = lp->mc_group[i];
			long position;

			if (!lp->mc_refs[i])
				continue;

			/* ip_eth_mc_map(): 01:00:5e plus the low 23 bits */
			addr[0] = 0x01;
			addr[1] = 0x00;
			addr[2] = 0x5e;
			addr[3] = (group >> 16) & 0x7f;
			addr[4] = (group >>  8) & 0xff;
			addr[5] = (group >>  0) & 0xff;

			/* only use the low order bits */
			position = crc32_le(~0UL, addr, 6) & 0x3f;

			/* do some messy swapping to put the bit in the right spot */
			multicast_table[invert3[position&7]] |=
				(1<<invert3[(position>>3)&7]);
		}

		/* be sure I get rid of flags I might have set */
		lp->rcr_cur_mode &= ~(RCR_PRMS | RCR_ALMUL);

		/* now, the table can be loaded into the chipset */
		update_multicast = 1;
	} else  {
		DBG(2, dev, "~(RCR_PRMS|RCR_ALMUL)\n");
		lp->rcr_cur_mode &= ~(RCR_PRMS | RCR_ALMUL);

		/*
		 * since I'm disabling all multicast entirely, I need to
		 * clear the multicast list
		 */
		memset(multicast_table, 0, sizeof(multicast_table));
		update_multicast = 1;
	}

	spin_lock_irq(&lp->lock);
	SMC_SELECT_BANK(lp, 0);
	SMC_SET_RCR(lp, lp->rcr_cur_mode);
	if (update_multicast) {
		SMC_SELECT_BANK(lp, 3);
		SMC_SET_MCAST(lp, multicast_table);
	}
	SMC_SELECT_BANK(lp, 2);
	spin_unlock_irq(&lp->lock);
}


/*
 * Open and Initialize the board
 *
 * Set up everything, reset the card, etc..
 */
static long
smc_open(struct netif *dev)
{
	struct smc_local *lp = dev->data;

	DBG(2, dev, "%s\n", __func__);

	/* Setup the default Register Modes */
	lp->tcr_cur_mode = TCR_DEFAULT;
	lp->rcr_cur_mode = RCR_DEFAULT;
	lp->rpc_cur_mode = RPC_DEFAULT |
				lp->cfg.leda << RPC_LSXA_SHFT |
				lp->cfg.ledb << RPC_LSXB_SHFT;

	/*
	 * If we are not using a MII interface, we need to
	 * monitor our own carrier signal to detect faults.
	 */
	if (lp->phy_type == 0)
		lp->tcr_cur_mode |= TCR_MON_CSN;

	/* reset the hardware */
	smc_reset(dev);
	smc_enable(dev);

	/* Configure the PHY, initialize the link state */
	if (lp->phy_type != 0)
		smc_phy_configure(lp);
	else {
		spin_lock_irq(&lp->lock);
		smc_10bt_check_media(dev, 1);
		spin_unlock_irq(&lp->lock);
	}

	netif_start_queue(dev);
	return 0;
}

/*
 * smc_close
 *
 * this makes the board clean up everything that it can
 * and not talk to the outside world.   Caused by
 * an 'ifconfig ethX down'
 */
static long smc_close(struct netif *dev)
{
	struct smc_local *lp = dev->data;

	DBG(2, dev, "%s\n", __func__);

	netif_stop_queue(dev);
	netif_carrier_off(dev);

	/* clear everything */
	smc_shutdown(dev);
	/* tasklet_kill(&lp->tx_task) */
	lp->tx_task = 0;
	smc_phy_powerdown(dev);
	return 0;
}

/* is_valid_ether_addr(): not multicast, not all zeros */
static long is_valid_ether_addr(const u8 *addr)
{
	long i;

	if (addr[0] & 0x01)
		return 0;

	for (i = 0; i < ETH_ALEN; i++)
		if (addr[i])
			return 1;

	return 0;
}

/*
 * eth_hw_addr_random(): a locally administered address. The kernel has
 * no entropy source, the 200 Hz counter and the time of day will do for
 * a fallback that is reported at boot.
 */
static void eth_hw_addr_random(struct netif *dev)
{
	u8 *addr = dev->hwlocal.adr.bytes;
	u32 r = jiffies;
	long i;

	if (KERNEL->xtime)
		r ^= KERNEL->xtime->tv_sec ^ (KERNEL->xtime->tv_usec << 12);

	for (i = 0; i < ETH_ALEN; i++) {
		r = r * 1103515245UL + 12345UL;
		addr[i] = r >> 24;
	}

	addr[0] &= 0xfe;	/* clear multicast bit */
	addr[0] |= 0x02;	/* set local assignment bit (IEEE802) */
}

/*
 * Function: smc_probe(unsigned long ioaddr)
 *
 * Purpose:
 *	Tests to see if a given ioaddr points to an SMC91x chip.
 *	Returns a 0 on success
 *
 * Algorithm:
 *	(1) see if the high byte of BANK_SELECT is 0x33
 * 	(2) compare the ioaddr with the base register's address
 *	(3) see if I recognize the chip ID in the appropriate register
 *
 * Here I do typical initialization tasks.
 *
 * o  Initialize the structure if needed
 * o  print out my vanity message if not done so already
 * o  print out what type of hardware is detected
 * o  print out the ethernet address
 * o  find the IRQ
 * o  set up my private data
 * o  configure the dev structure with my subroutines
 * o  actually GRAB the irq.
 * o  GRAB the region
 *
 * @hwaddr is the address from ETHERNAT.INF, all zeros if there was none.
 */
static long smc_probe(struct netif *dev, char *ioaddr, const u8 *hwaddr)
{
	struct smc_local *lp = dev->data;
	long retval;
	u32 val, revision_register;
	u8 addr[ETH_ALEN];

	DBG(2, dev, "%s: %s\n", CARDNAME, __func__);

	/* First, see if the high byte is 0x33 */
	val = SMC_CURRENT_BANK(lp);
	DBG(2, dev, "%s: bank signature probe returned 0x%04x\n",
	    CARDNAME, val);
	if ((val & 0xFF00) != 0x3300) {
		if ((val & 0xFF) == 0x33) {
			ALERT(("%s: Detected possible byte-swapped interface at IOADDR %p",
				    CARDNAME, ioaddr));
		}
		retval = ENODEV;
		goto err_out;
	}

	/*
	 * The above MIGHT indicate a device, but I need to write to
	 * further test this.
	 */
	SMC_SELECT_BANK(lp, 0);
	val = SMC_CURRENT_BANK(lp);
	if ((val & 0xFF00) != 0x3300) {
		retval = ENODEV;
		goto err_out;
	}

	/*
	 * well, we've already written once, so hopefully another
	 * time won't hurt.  This time, I need to switch the bank
	 * register to bank 1, so I can access the base address
	 * register
	 */
	SMC_SELECT_BANK(lp, 1);
	val = SMC_GET_BASE(lp);
	val = ((val & 0x1F00) >> 3) << SMC_IO_SHIFT;
	if (((unsigned long)ioaddr & (0x3e0 << SMC_IO_SHIFT)) != val) {
		DEBUG(("%s: IOADDR %p doesn't match configuration (%lx).",
			    CARDNAME, ioaddr, val));
	}

	/*
	 * check if the revision register is something that I
	 * recognize.  These might need to be added to later,
	 * as future revisions could be added.
	 */
	SMC_SELECT_BANK(lp, 3);
	revision_register = SMC_GET_REV(lp);
	DBG(2, dev, "%s: revision = 0x%04x\n", CARDNAME, revision_register);
	if (((revision_register >> 4) & 0xF) < CHIP_9192 ||
	    ((revision_register >> 4) & 0xF) > CHIP_91111FD ||
	    (revision_register & 0xff00) != 0x3300) {
		/* I don't recognize this chip, so... */
		ALERT(("%s: IO %p: Unrecognized revision register 0x%04lx, Contact author.",
			    CARDNAME, ioaddr, revision_register));

		retval = ENODEV;
		goto err_out;
	}

	/* At this point I'll assume that the chip is an SMC91x. */
	DEBUG(("%s", version));

	/* fill in some of the fields */
	lp->base = ioaddr;
	lp->version = revision_register & 0xff;

	/* Get the MAC address */
	SMC_SELECT_BANK(lp, 1);
	SMC_GET_MAC_ADDR(lp, addr);
	memcpy(dev->hwlocal.adr.bytes, addr, ETH_ALEN);

	/* ETHERNAT.INF takes precedence, like the address a platform
	 * would pass in
	 */
	if (is_valid_ether_addr(hwaddr))
		memcpy(dev->hwlocal.adr.bytes, hwaddr, ETH_ALEN);

	/* now, reset the chip, and put it into a known state */
	smc_reset(dev);

	/* the IRQ is known: vector 0xc4 through the CPLD */

	lp->dev = dev;
	lp->mii.phy_id_mask = 0x1f;
	lp->mii.reg_num_mask = 0x1f;
	lp->mii.force_media = 0;
	lp->mii.full_duplex = 0;
	lp->mii.dev = dev;
	lp->mii.mdio_read = smc_phy_read;
	lp->mii.mdio_write = smc_phy_write;

	/*
	 * Locate the phy, if any.
	 */
	if (lp->version >= (CHIP_91100 << 4))
		smc_phy_detect(dev);

	/* then shut everything down to save power */
	smc_shutdown(dev);
	smc_phy_powerdown(dev);

	/* Set default parameters */
	lp->ctl_rfduplx = 0;
	lp->ctl_rspeed = 10;

	if (lp->version >= (CHIP_91100 << 4)) {
		lp->ctl_rfduplx = 1;
		lp->ctl_rspeed = 100;
	}

	/* Grab the IRQ: request_irq() installs the vector and opens the
	 * CPLD gate
	 */
	ethernat_old_vector = (void (*)(void)) Setexc(ATARI_ETHERNAT_VECTOR, (long) ethernat_interrupt);
	atari_ethernat_enable();

	retval = 0;

err_out:
	return retval;
}


/*
 * The MiNTNet side: what net_device, its ops and the platform code do on
 * Linux.
 */

static long	ethernat_open		(struct netif *);
static long	ethernat_close		(struct netif *);
static long	ethernat_output		(struct netif *, BUF *, const char *, short, short);
static long	ethernat_ioctl		(struct netif *, short, long);
static long	ethernat_config		(struct netif *, struct ifopt *);
static void	ethernat_igmp_mac_filter (struct netif *, ulong, char);
static void	ethernat_timeout	(struct netif *);

/*
 * This gets called when someone makes an 'ifconfig up' on this interface
 * and the interface was down before.
 */
static long
ethernat_open (struct netif *nif)
{
	long error;

	error = smc_open (nif);
	if (error)
		return error;

	/* __dev_open() applies the receive mode after ndo_open */
	smc_set_multicast_list (nif);

	return 0;
}

/*
 * Opposite of ethernat_open(), is called when 'ifconfig down' on this interface
 * is done and the interface was up before.
 */
static long
ethernat_close (struct netif *nif)
{
	return smc_close (nif);
}

/*
 * This routine is responsible for enqueing a packet for later sending.
 * The packet it passed in `buf', the destination hardware address and
 * length in `hwaddr' and `hwlen' and the type of the packet is passed
 * in `pktype'.
 *
 * `hwaddr' is guaranteed to be of type nif->hwtype and `hwlen' is
 * garuanteed to be equal to nif->hwlocal.len.
 *
 * `pktype' is currently one of (definitions in if.h):
 *	PKTYPE_IP for IP packets,
 *	PKTYPE_ARP for ARP packets,
 *	PKTYPE_RARP for reverse ARP packets.
 *
 * These constants are equal to the ethernet protocol types, ie. an
 * Ethernet driver may use them directly without prior conversion to
 * write them into the `proto' field of the ethernet header.
 *
 * If the hardware is currently busy, then you can use the interface
 * output queue (nif->snd) to store the packet for later transmission:
 *	if_enqueue (&nif->snd, buf, buf->info).
 *
 * `buf->info' specifies the packet's delivering priority. if_enqueue()
 * uses it to do some priority queuing on the packets, ie. if you enqueue
 * a high priority packet it may jump over some lower priority packets
 * that were already in the queue (ie that is *no* FIFO queue).
 *
 * You can dequeue a packet later by doing:
 *	buf = if_dequeue (&nif->snd);
 *
 * This will return NULL is no more packets are left in the queue.
 *
 * The buffer handling uses the structure BUF that is defined in buf.h.
 * Basically a BUF looks like this:
 *
 * typedef struct {
 *	long buflen;
 *	char *dstart;
 *	char *dend;
 *	...
 *	char data[0];
 * } BUF;
 *
 * The structure consists of BUF.buflen bytes. Up until BUF.data there are
 * some header fields as shown above. Beginning at BUF.data there are
 * BUF.buflen - sizeof (BUF) bytes (called userspace) used for storing the
 * packet.
 *
 * BUF.dstart must always point to the first byte of the packet contained
 * within the BUF, BUF.dend points to the first byte after the packet.
 *
 * BUF.dstart should be word aligned if you pass the BUF to any MintNet
 * functions! (except for the buf_* functions itself).
 *
 * BUF's are allocated by
 *	nbuf = buf_alloc (space, reserve, mode);
 *
 * where `space' is the size of the userspace of the BUF you need, `reserve'
 * is used to set BUF.dstart = BUF.dend = BUF.data + `reserve' and mode is
 * one of
 *	BUF_NORMAL for calls from kernel space,
 *	BUF_ATOMIC for calls from interrupt handlers.
 *
 * buf_alloc() returns NULL on failure.
 *
 * Usually you need to pre- or postpend some headers to the packet contained
 * in the passed BUF. To make sure there is enough space in the BUF for this
 * use
 *	nbuf = buf_reserve (obuf, reserve, where);
 *
 * where `obuf' is the BUF where you want to reserve some space, `reserve'
 * is the amount of space to reserve and `where' is one of
 *	BUF_RESERVE_START for reserving space before BUF.dstart
 *	BUF_RESERVE_END for reserving space after BUF.dend
 *
 * Note that buf_reserve() returns pointer to a new buffer `nbuf' (possibly
 * != obuf) that is a clone of `obuf' with enough space allocated. `obuf'
 * is no longer existant afterwards.
 *
 * However, if buf_reserve() returns NULL for failure then `obuf' is
 * untouched.
 *
 * buf_reserve() does not modify the BUF.dstart or BUF.dend pointers, it
 * only makes sure you have the space to do so.
 *
 * In the worst case (if the BUF is to small), buf_reserve() allocates a new
 * BUF and copies the old one to the new one (this is when `nbuf' != `obuf').
 *
 * To avoid this you should reserve enough space when calling buf_alloc(), so
 * buf_reserve() does not need to copy. This is what MintNet does with the BUFs
 * passed to the output function, so that copying is never needed. You should
 * do the same for input BUFs, ie allocate the packet as eg.
 *	buf = buf_alloc (nif->mtu+sizeof (eth_hdr)+100, 50, BUF_ATOMIC);
 *
 * Then up to nif->mtu plus the length of the ethernet header bytes long
 * frames may ne received and there are still 50 bytes after and before
 * the packet.
 *
 * If you have sent the contents of the BUF you should free it by calling
 *	buf_deref (`buf', `mode');
 *
 * where `buf' should be freed and `mode' is one of the modes described for
 * buf_alloc().
 *
 * Functions that can be called from interrupt:
 *	buf_alloc (..., ..., BUF_ATOMIC);
 *	buf_deref (..., BUF_ATOMIC);
 *	if_enqueue ();
 *	if_dequeue ();
 *	if_input ();
 *	eth_remove_hdr ();
 *	addroottimeout (..., ..., 1);
 */
/* ndo_start_xmit with the qdisc in front of it */
static long
ethernat_output (struct netif *nif, BUF *buf, const char *hwaddr, short hwlen, short pktype)
{
	struct smc_local *lp = nif->data;
	BUF *nbuf;

	nbuf = eth_build_hdr (buf, nif, hwaddr, pktype);
	if (nbuf == NULL)
	{
		nif->out_errors++;
		return ENOMEM;
	}

	if (nif->bpf)
		bpf_input (nif, nbuf);

	if (lp->queue_stopped)
	{
		/* if_enqueue() frees the buffer when the queue is full */
		if (if_enqueue (&nif->snd, nbuf, nbuf->info))
			nif->out_errors++;
		return 0;
	}

	return smc_hard_start_xmit (nbuf, nif);
}

/*
 * MintNet notifies you of some noteable IOCLT's. Usually you don't
 * need to act on them because MintNet already has done so and only
 * tells you that an ioctl happened.
 *
 * One useful thing might be SIOCGLNKFLAGS and SIOCSLNKFLAGS for setting
 * and getting flags specific to your driver. For an example how to use
 * them look at slip.c
 */
static long
ethernat_ioctl (struct netif *nif, short cmd, long arg)
{
	struct ifreq *ifr;

	switch (cmd)
	{
		case SIOCSIFNETMASK:
		case SIOCSIFADDR:
			return 0;

		case SIOCSIFFLAGS:
			/* ndo_set_rx_mode */
			if (nif->flags & IFF_UP)
				smc_set_multicast_list (nif);
			return 0;

		case SIOCSIFMTU:
			/*
			 * Limit MTU to 1500 bytes. MintNet has alraedy set nif->mtu
			 * to the new value, we only limit it here.
			 */
			if (nif->mtu > ETH_MAX_DLEN)
				nif->mtu = ETH_MAX_DLEN;
			return 0;

		case SIOCSIFOPT:
			ifr = (struct ifreq *) arg;
			return ethernat_config (nif, ifr->ifru.data);
	}

	return ENOSYS;
}

/*
 * Interface configuration via SIOCSIFOPT. The ioctl is passed a
 * struct ifreq *ifr. ifr->ifru.data points to a struct ifopt, which
 * we get as the second argument here.
 *
 * If the user MUST configure some parameters before the interface
 * can run make sure that ethernat_open() fails unless all the necessary
 * parameters are set.
 *
 * Return values	meaning
 * ENOSYS		option not supported
 * ENOENT		invalid option value
 * 0			Ok
 */
static long
ethernat_config (struct netif *nif, struct ifopt *ifo)
{
# define STRNCMP(s)	(strncmp ((s), ifo->option, sizeof (ifo->option)))

	if (!STRNCMP ("hwaddr"))
	{
		/*
		 * Set hardware address: eth_mac_addr(), which refuses while
		 * the interface is up; the chip is programmed on open
		 */
		if (ifo->valtype != IFO_HWADDR)
			return ENOENT;
		if (nif->flags & IFF_UP)
			return EBUSY;
		if (!is_valid_ether_addr ((const u8 *) ifo->ifou.v_string))
			return EADDRNOTAVAIL;
		memcpy (nif->hwlocal.adr.bytes, ifo->ifou.v_string, ETH_ALEN);
		return 0;
	}
	else if (!STRNCMP ("braddr"))
	{
		/*
		 * Set broadcast address
		 */
		if (ifo->valtype != IFO_HWADDR)
			return ENOENT;
		memcpy (nif->hwbrcst.adr.bytes, ifo->ifou.v_string, ETH_ALEN);
		return 0;
	}

	return ENOSYS;
}

/*
 * The IGMP code reports groups one at a time; the Linux multicast list
 * is kept here, with up to 16 groups, since beyond that the chip
 * accepts all multicasts anyway.
 */
static void
ethernat_igmp_mac_filter (struct netif *nif, ulong group, char action)
{
	struct smc_local *lp = nif->data;
	long i, slot = -1;

	for (i = 0; i < 16; i++)
	{
		if (lp->mc_refs[i] && lp->mc_group[i] == group)
		{
			slot = i;
			break;
		}
		if (slot < 0 && !lp->mc_refs[i])
			slot = i;
	}

	if (action == IGMP_ADD_MAC_FILTER)
	{
		if (slot < 0)
			lp->mc_overflow++;
		else
		{
			if (!lp->mc_refs[slot])
			{
				lp->mc_group[slot] = group;
				lp->mc_count++;
			}
			lp->mc_refs[slot]++;
		}
	}
	else
	{
		if (slot >= 0 && lp->mc_refs[slot] && lp->mc_group[slot] == group)
		{
			if (--lp->mc_refs[slot] == 0)
				lp->mc_count--;
		}
		else if (lp->mc_overflow)
			lp->mc_overflow--;
	}

	if (nif->flags & IFF_UP)
		smc_set_multicast_list (nif);
}

/*
 * Called every IF_SLOWTIMEOUT (one second) while the interface is up:
 * the netdev watchdog, and the place where link changes found by the
 * interrupt handler are reported.
 */
static void
ethernat_timeout (struct netif *nif)
{
	struct smc_local *lp = nif->data;

	if (lp->mii.link_msg_pending)
	{
		lp->mii.link_msg_pending = 0;
		ALERT (("%s%d: %s", nif->name, nif->unit, lp->mii.link_msg));
	}

	/* dev_watchdog(): a stopped queue that saw no transmission */
	if (lp->queue_stopped &&
	    time_after (jiffies, lp->trans_start + msecs_to_jiffies (watchdog)))
		smc_timeout (nif);
}

/*
 * ETHERNAT.INF holds the address as 12 hex digits, optionally separated
 * by ':' or '-', looked for in the current directory (the sysdir at boot)
 * and then in the root of the boot drive.
 */
# define DriveToLetter(d) ((d) < 26 ? 'A' + (d) : (d) - 26 + '1')

static long
ethernat_read_inf (u8 *hwaddr)
{
	char buf[32];
	char path[] = "A:\\ETHERNAT.INF";
	long fd, n, i, digits = 0;
	u8 value = 0;

	memset (hwaddr, 0, ETH_ALEN);

	fd = f_open ("ethernat.inf", O_RDONLY);
	if (fd < 0)
	{
		path[0] = DriveToLetter (*(short *) 0x446L);
		fd = f_open (path, O_RDONLY);
	}
	if (fd < 0)
		return fd;

	n = f_read (fd, sizeof (buf), buf);
	f_close (fd);
	if (n < 0)
		return n;

	for (i = 0; i < n && digits < 12; i++)
	{
		char c = buf[i];
		u8 nibble;

		if (c >= '0' && c <= '9')
			nibble = c - '0';
		else if (c >= 'a' && c <= 'f')
			nibble = c - 'a' + 10;
		else if (c >= 'A' && c <= 'F')
			nibble = c - 'A' + 10;
		else if (c == ':' || c == '-')
			continue;
		else
			break;

		value = (value << 4) | nibble;
		if (digits & 1)
			hwaddr[digits >> 1] = value;
		digits++;
	}

	if (digits != 12)
	{
		memset (hwaddr, 0, ETH_ALEN);
		return EINVAL;
	}

	return 0;
}

long driver_init (void);

/*
 * Initialization. This is called when the driver is loaded. If you
 * link the driver with main.o and init.o then this must be called
 * driver_init() because main() calles a function with this name.
 *
 * You should probe for your hardware here, setup the interface
 * structure and register your interface.
 *
 * This function should return 0 on success and != 0 if initialization
 * fails.
 */
long
driver_init (void)
{
	static char message[128];
	struct netif *nif = &if_ethernat;
	struct smc_local *lp = &smc_priv;
	u8 hwaddr[ETH_ALEN];
	const u8 *mac;

	/* the hwreg_present() check of the platform code */
	if (!hwreg_present (ATARI_ETHERNAT_CPLD))
	{
		c_conws ("EtherNAT: not found\r\n");
		return -1;
	}

	ethernat_read_inf (hwaddr);

	/* the platform data: every bus width, no wait states, and the LEDs */
	lp->cfg.flags = 0;
	lp->cfg.flags |= (SMC_CAN_USE_8BIT)  ? SMC91X_USE_8BIT  : 0;
	lp->cfg.flags |= (SMC_CAN_USE_16BIT) ? SMC91X_USE_16BIT : 0;
	lp->cfg.flags |= (SMC_CAN_USE_32BIT) ? SMC91X_USE_32BIT : 0;
	lp->cfg.flags |= (SMC_NOWAIT) ? SMC91X_NOWAIT : 0;
	lp->cfg.leda = RPC_LSA_DEFAULT;
	lp->cfg.ledb = RPC_LSB_DEFAULT;

	/*
	 * Set interface name
	 */
	strcpy (nif->name, "en");
	/*
	 * Set interface unit. if_getfreeunit("name") returns a yet
	 * unused unit number for the interface type "name".
	 */
	nif->unit = if_getfreeunit ("en");
	/*
	 * Alays set to zero
	 */
	nif->metric = 0;
	/*
	 * Initial interface flags, should be IFF_BROADCAST for
	 * Ethernet.
	 */
	nif->flags = IFF_BROADCAST;
	/*
	 * Maximum transmission unit, should be >= 46 and <= 1500 for
	 * Ethernet
	 */
	nif->mtu = 1500;
	/*
	 * Time in ms between calls to (*nif->timeout) ();
	 */
	nif->timer = 0;

	/*
	 * Interface hardware type
	 */
	nif->hwtype = HWTYPE_ETH;
	/*
	 * Hardware address length, 6 bytes for Ethernet
	 */
	nif->hwlocal.len =
	nif->hwbrcst.len = ETH_ALEN;

	memcpy (nif->hwbrcst.adr.bytes, "\377\377\377\377\377\377", ETH_ALEN);

	/*
	 * Set length of send and receive queue. IF_MAXQ is a good value.
	 */
	nif->rcv.maxqlen = IF_MAXQ;
	nif->snd.maxqlen = IF_MAXQ;
	/*
	 * Setup pointers to service functions
	 */
	nif->open = ethernat_open;
	nif->close = ethernat_close;
	nif->output = ethernat_output;
	nif->ioctl = ethernat_ioctl;
	nif->igmp_mac_filter = ethernat_igmp_mac_filter;
	/*
	 * Timer function that is called every second.
	 */
	nif->timeout = ethernat_timeout;

	nif->data = lp;

	/*
	 * Number of packets the hardware can receive in fast succession,
	 * 0 means unlimited. The chip has 8 KB for everything.
	 */
	nif->maxpackets = 4;

	if (smc_probe (nif, (char *) ATARI_ETHERNAT_PHYS_ADDR, hwaddr) != 0)
	{
		c_conws ("EtherNAT: no LAN91C111 found\r\n");
		return -1;
	}

	if (!is_valid_ether_addr (nif->hwlocal.adr.bytes))
	{
		eth_hw_addr_random (nif);
		c_conws ("EtherNAT: no address in ETHERNAT.INF, using a random one\r\n");
	}

	/*
	 * Register the interface.
	 */
	if_register (nif);

	mac = nif->hwlocal.adr.bytes;
	ksprintf (message, "EtherNAT driver v1.0 (%s%d): LAN91C111 rev %ld, PHY %08lx at %ld, %02x:%02x:%02x:%02x:%02x:%02x\r\n",
		nif->name, nif->unit, lp->version & 0x0f, lp->phy_type, lp->mii.phy_id,
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	c_conws (message);

	return 0;
}
