/*
 * NetUSBee/EtherNEC driver for FreeMiNT: an NE2000 compatible card (the
 * RTL8019AS on the NetUSBee) on the cartridge port.
 *
 * This file belongs to FreeMiNT. It's not in the original MiNT 1.12
 * distribution. See the file CHANGES for a detailed log of changes.
 *
 * This is a port of the Linux ne driver, drivers/net/ethernet/8390/ne.c
 * (Linux 7.3), with the 8390 core, lib8390.c, included the way Linux does
 * it, and of the cartridge port ISA emulation of arch/m68k/include/asm/
 * raw_io.h (CONFIG_ATARI_ROM_ISA). Names are kept as in Linux so that
 * fixes can be carried over; the MiNTNet interface (netusbee_*) at the
 * end of this file takes the place of the Linux net_device. What is not
 * ported: ISA autoprobing, ISAPnP, IRQ autodetection, module parameters
 * and power management.
 *
 * The card has no interrupt line. Linux polls it from the MFP timer D
 * interrupt; here the 8390 interrupt handler runs from the 200 Hz system
 * timer, like the EtherNE driver of Thomas Redelberger, whose cartridge
 * port encoding this is too.
 *
 * The kernel is built with -mshort, so Linux "int" is "long" here.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

// SPDX-License-Identifier: GPL-1.0+
/* ne.c: A general non-shared-memory NS8390 ethernet driver for linux. */
/*
    Written 1992-94 by Donald Becker.

    Copyright 1993 United States Government as represented by the
    Director, National Security Agency.

    The author may be reached as becker@scyld.com, or C/O
    Scyld Computing Corporation, 410 Severn Ave., Suite 210, Annapolis MD 21403

    This driver should work with many programmed-I/O 8390-based ethernet
    boards.  Currently it supports the NE1000, NE2000, many clones,
    and some Cabletron products.

    Changelog:

    Paul Gortmaker	: use ENISR_RDC to monitor Tx PIO uploads, made
			  sanity checks and bad clone support optional.
    Paul Gortmaker	: new reset code, reset card after probe at boot.
    Paul Gortmaker	: multiple card support for module users.
    Paul Gortmaker	: Support for PCI ne2k clones, similar to lance.c
    Paul Gortmaker	: Allow users with bad cards to avoid full probe.
    Paul Gortmaker	: PCI probe changes, more PCI cards supported.
    rjohnson@analogic.com : Changed init order so an interrupt will only
    occur after memory is allocated for dev->priv. Deallocated memory
    last in cleanup_modue()
    Richard Guenther    : Added support for ISAPnP cards
    Paul Gortmaker	: Discontinued PCI support - use ne2k-pci.c instead.
    Hayato Fujiwara	: Add m32r support.

*/

# include "global.h"

# include "buf.h"
# include "inet4/if.h"
# include "inet4/ifeth.h"
# include "inet4/igmp.h"
# include "netinfo.h"

# include "mint/delay.h"
# include "mint/mdelay.h"
# include "mint/sockio.h"
# include "mint/arch/asm_spl.h"

# include <mint/osbind.h>

# include "8390.h"
# include "netusbee_int.h"

# define likely(x)	__builtin_expect(!!(x), 1)
# define unlikely(x)	__builtin_expect(!!(x), 0)

/* the 200 Hz system timer plays jiffies; the kernel's hz_200 is not
 * exported to modules
 */
# undef jiffies
# define jiffies	(*(volatile ulong *) 0x4baUL)
# define time_after(a, b)	((long) (b) - (long) (a) < 0)


/* arch/m68k/include/asm/raw_io.h */
/* SPDX-License-Identifier: GPL-2.0 */
/*
 * linux/include/asm-m68k/raw_io.h
 *
 * 10/20/00 RZ: - created from bits of io.h and ide.h to cleanup namespace
 *
 */

/*
 * The cartridge port as an ISA bus, from arch/m68k/include/asm/raw_io.h
 * and io_mm.h (CONFIG_ATARI_ROM_ISA).
 *
 * The ISA address goes to cartridge port address lines A9-A15. A read is
 * a byte read in the ROM4 area, the data is on D8-D15. The port is read
 * only, so a write is a byte read in the ROM3 area with the data on A1-A8
 * (see HARDWARE.TXT of the EtherNE package). Linux uses the 0xfffa0000
 * mirror of the area; this uses 0x00fa0000 like the EtherNE and NetUSBee
 * USB drivers, which the same hardware has been run with under TOS and
 * MiNT on the 68000, the 68030 and the CT60. The 020+ builds add the nop
 * after a write that the EtherNE bus layer for those machines has
 * (BUSENEC3.I).
 */

#define enec_isa_read_base  0x00fa0000UL
#define enec_isa_write_base 0x00fb0000UL

#define ENEC_ISA_IO_B(ioaddr)	(enec_isa_read_base+((((unsigned long)(ioaddr))&0x7F)<<9))

static inline u8 rom_in_8(unsigned long addr)
{
	return *(volatile u8 *) addr;
}

static inline void rom_out_8(unsigned long addr, u8 b)
{
	(void) *(volatile u8 *)((addr | 0x10000UL) + ((unsigned long) b << 1));
#if defined(__mc68020__) || defined(__mc68030__) || defined(__mc68040__) || defined(__mc68060__)
	__asm__ __volatile__ ("nop");
#endif
}

#define inb(port)	rom_in_8(ENEC_ISA_IO_B(port))
#define outb(val, port)	rom_out_8(ENEC_ISA_IO_B(port), (val))
/* isa_delay() is empty for the cartridge port */
#define inb_p(port)	inb(port)
#define outb_p(val, port) outb((val), (port))

static inline void insb(unsigned long port, void *buf, long len)
{
	unsigned long addr = ENEC_ISA_IO_B(port);
	u8 *p = buf;
	long i;

	for (i = 0; i < len; i++)
		*p++ = rom_in_8(addr);
}

static inline void outsb(unsigned long port, const void *buf, long len)
{
	unsigned long addr = ENEC_ISA_IO_B(port);
	const u8 *p = buf;
	long i;

	for (i = 0; i < len; i++)
		rom_out_8(addr, *p++);
}

/* the platform data of arch/m68k/atari/config.c */
#define ATARI_ETHERNEC_BASE		0x300
/* Linux: IRQ_MFP_TIMER1, here the 200 Hz system timer */
#define ATARI_ETHERNEC_VECTOR		(0x114 / 4)


static struct netif if_netusbee;
static struct ei_device ei_priv;

#include "lib8390.c"

#define DRV_NAME "ne"

/* Some defines that people can play with if so inclined. */

/* Do we support clones that don't adhere to 14,15 of the SAprom ? */
#define SUPPORT_NE_BAD_CLONES
/* 0xbad = bad sig or no reset ack */
#define BAD 0xbad

/* Do we perform extra sanity checks on stuff ? */
/* #define NE_SANITY_CHECK */

/* Do we implement the read before write bugfix ? */
/* #define NE_RW_BUGFIX */

/* Do we have a non std. amount of memory? (in units of 256 byte pages) */
/* #define PACKETBUF_MEMSIZE	0x40 */

#ifdef SUPPORT_NE_BAD_CLONES
/* A list of bad clones that we none-the-less recognize. */
static const struct { const char *name8, *name16; unsigned char SAprefix[4];}
bad_clone_list[] = {
    {"DE100", "DE200", {0x00, 0xDE, 0x01,}},
    {"DE120", "DE220", {0x00, 0x80, 0xc8,}},
    {"DFI1000", "DFI2000", {'D', 'F', 'I',}}, /* Original, eh?  */
    {"EtherNext UTP8", "EtherNext UTP16", {0x00, 0x00, 0x79}},
    {"NE1000","NE2000-invalid", {0x00, 0x00, 0xd8}}, /* Ancient real NE1000. */
    {"NN1000", "NN2000",  {0x08, 0x03, 0x08}}, /* Outlaw no-name clone. */
    {"4-DIM8","4-DIM16", {0x00,0x00,0x4d,}},  /* Outlaw 4-Dimension cards. */
    {"Con-Intl_8", "Con-Intl_16", {0x00, 0x00, 0x24}}, /* Connect Int'nl */
    {"ET-100","ET-200", {0x00, 0x45, 0x54}}, /* YANG and YA clone */
    {"COMPEX","COMPEX16",{0x00,0x80,0x48}}, /* Broken ISA Compex cards */
    {"E-LAN100", "E-LAN200", {0x00, 0x00, 0x5d}}, /* Broken ne1000 clones */
    {"PCM-4823", "PCM-4823", {0x00, 0xc0, 0x6c}}, /* Broken Advantech MoBo */
    {"REALTEK", "RTL8019", {0x00, 0x00, 0xe8}}, /* no-name with Realtek chip */
    {"LCS-8834", "LCS-8836", {0x04, 0x04, 0x37}}, /* ShinyNet (SET) */
    {NULL,}
};
#endif

/* ---- No user-serviceable parts below ---- */

#define NE_BASE	 ((unsigned long) dev->base_addr)
#define NE_CMD	 	0x00
#define NE_DATAPORT	0x10	/* NatSemi-defined port window offset. */
#define NE_RESET	0x1f	/* Issue a read to reset, a write to clear. */
#define NE_IO_EXTENT	0x20

#define NE1SM_START_PG	0x20	/* First page of TX buffer */
#define NE1SM_STOP_PG 	0x40	/* Last page +1 of RX ring */
#define NESM_START_PG	0x40	/* First page of TX buffer */
#define NESM_STOP_PG	0x80	/* Last page +1 of RX ring */

/* 8-bit mode on Atari */
#define DCR_VAL 0x48

static long ne_probe1(struct netif *dev, unsigned long ioaddr);

static void ne_reset_8390(struct netif *dev);
static void ne_get_8390_hdr(struct netif *dev, struct e8390_pkt_hdr *hdr,
			  long ring_page);
static void ne_block_input(struct netif *dev, long count,
			  BUF *skb, long ring_offset);
static void ne_block_output(struct netif *dev, const long count,
		const unsigned char *buf, const long start_page);

/*  Probe for various non-shared-memory ethercards.

   NEx000-clone boards have a Station Address PROM (SAPROM) in the packet
   buffer memory space.  NE2000 clones have 0x57,0x57 in bytes 0x0e,0x0f of
   the SAPROM, while other supposed NE2000 clones must be detected by their
   SA prefix.

   Reading the SAPROM from a word-wide card with the 8390 set in byte-wide
   mode results in doubled values, which can be detected and compensated for.

   The probe is also responsible for initializing the card and filling
   in the 'dev' and 'ei_status' structures.

   We use the minimum memory size for some ethercard product lines, iff we can't
   distinguish models.  You can increase the packet buffer size by setting
   PACKETBUF_MEMSIZE.  Reported Cabletron packet buffer locations are:
	E1010   starts at 0x100 and ends at 0x2000.
	E1010-x starts at 0x100 and ends at 0x8000. ("-x" means "more memory")
	E2010	 starts at 0x100 and ends at 0x4000.
	E2010-x starts at 0x100 and ends at 0xffff.  */

static long ne_probe1(struct netif *dev, unsigned long ioaddr)
{
	long i;
	unsigned char SA_prom[32];
	long wordlength = 2;
	const char *name = NULL;
	long start_page, stop_page;
	long neX000, ctron, copam, bad_card;
	long reg0, ret;
	struct ei_device *ei_local = dev->data;

	reg0 = inb_p(ioaddr);
	if (reg0 == 0xFF) {
		ret = ENODEV;
		goto err_out;
	}

	/* Do a preliminary verification that we have a 8390. */
	{
		long regd;
		outb_p(E8390_NODMA+E8390_PAGE1+E8390_STOP, ioaddr + E8390_CMD);
		regd = inb_p(ioaddr + 0x0d);
		outb_p(0xff, ioaddr + 0x0d);
		outb_p(E8390_NODMA+E8390_PAGE0, ioaddr + E8390_CMD);
		inb_p(ioaddr + EN0_COUNTER0); /* Clear the counter by reading. */
		if (inb_p(ioaddr + EN0_COUNTER0) != 0) {
			outb_p(reg0, ioaddr);
			outb_p(regd, ioaddr + 0x0d);	/* Restore the old values. */
			ret = ENODEV;
			goto err_out;
		}
	}

	netdev_info(dev, "NE*000 ethercard probe at %#3lx:", ioaddr);

	/* A user with a poor card that fails to ack the reset, or that
	   does not have a valid 0x57,0x57 signature can still use this
	   without having to recompile. Specifying an i/o address along
	   with an otherwise unused dev->mem_end value of "0xBAD" will
	   cause the driver to skip these parts of the probe. */

	bad_card = 0;

	/* Reset card. Who knows what dain-bramaged state it was left in. */

	{
		unsigned long reset_start_time = jiffies;

		/* DON'T change these to inb_p/outb_p or reset will fail on clones. */
		outb(inb(ioaddr + NE_RESET), ioaddr + NE_RESET);

		while ((inb_p(ioaddr + EN0_ISR) & ENISR_RESET) == 0)
		if (time_after(jiffies, reset_start_time + 2*HZ/100)) {
			if (bad_card) {
				netdev_info(dev, " (warning: no reset ack)");
				break;
			} else {
				netdev_info(dev, " not found (no reset ack).\n");
				ret = ENODEV;
				goto err_out;
			}
		}

		outb_p(0xff, ioaddr + EN0_ISR);		/* Ack all intr. */
	}

	/* Read the 16 bytes of station address PROM.
	   We must first initialize registers, similar to NS8390p_init(eifdev, 0).
	   We can't reliably read the SAPROM address without this.
	   (I learned the hard way!). */
	{
		static const struct {unsigned char value, offset; } program_seq[] =
		{
			{E8390_NODMA+E8390_PAGE0+E8390_STOP, E8390_CMD}, /* Select page 0*/
			{0x48,	EN0_DCFG},	/* Set byte-wide (0x48) access. */
			{0x00,	EN0_RCNTLO},	/* Clear the count regs. */
			{0x00,	EN0_RCNTHI},
			{0x00,	EN0_IMR},	/* Mask completion irq. */
			{0xFF,	EN0_ISR},
			{E8390_RXOFF, EN0_RXCR},	/* 0x20  Set to monitor */
			{E8390_TXOFF, EN0_TXCR},	/* 0x02  and loopback mode. */
			{32,	EN0_RCNTLO},
			{0x00,	EN0_RCNTHI},
			{0x00,	EN0_RSARLO},	/* DMA starting at 0x0000. */
			{0x00,	EN0_RSARHI},
			{E8390_RREAD+E8390_START, E8390_CMD},
		};

		for (i = 0; i < (long) (sizeof(program_seq) / sizeof(program_seq[0])); i++)
			outb_p(program_seq[i].value, ioaddr + program_seq[i].offset);

	}
	for(i = 0; i < 32 /*sizeof(SA_prom)*/; i+=2) {
		SA_prom[i] = inb(ioaddr + NE_DATAPORT);
		SA_prom[i+1] = inb(ioaddr + NE_DATAPORT);
		if (SA_prom[i] != SA_prom[i+1])
			wordlength = 1;
	}

	if (wordlength == 2)
	{
		for (i = 0; i < 16; i++)
			SA_prom[i] = SA_prom[i+i];
		/* We must set the 8390 for word mode. */
		outb_p(DCR_VAL, ioaddr + EN0_DCFG);
		start_page = NESM_START_PG;

		/*
		 * Realtek RTL8019AS datasheet says that the PSTOP register
		 * shouldn't exceed 0x60 in 8-bit mode.
		 * This chip can be identified by reading the signature from
		 * the  remote byte count registers (otherwise write-only)...
		 */
		if ((DCR_VAL & 0x01) == 0 &&		/* 8-bit mode */
		    inb(ioaddr + EN0_RCNTLO) == 0x50 &&
		    inb(ioaddr + EN0_RCNTHI) == 0x70)
			stop_page = 0x60;
		else
			stop_page = NESM_STOP_PG;
	} else {
		start_page = NE1SM_START_PG;
		stop_page  = NE1SM_STOP_PG;
	}

	neX000 = (SA_prom[14] == 0x57  &&  SA_prom[15] == 0x57);
	ctron =  (SA_prom[0] == 0x00 && SA_prom[1] == 0x00 && SA_prom[2] == 0x1d);
	copam =  (SA_prom[14] == 0x49 && SA_prom[15] == 0x00);

	/* Set up the rest of the parameters. */
	if (neX000 || bad_card || copam) {
		name = (wordlength == 2) ? "NE2000" : "NE1000";
	}
	else if (ctron)
	{
		name = (wordlength == 2) ? "Ctron-8" : "Ctron-16";
		start_page = 0x01;
		stop_page = (wordlength == 2) ? 0x40 : 0x20;
	}
	else
	{
#ifdef SUPPORT_NE_BAD_CLONES
		/* Ack!  Well, there might be a *bad* NE*000 clone there.
		   Check for total bogus addresses. */
		for (i = 0; bad_clone_list[i].name8; i++)
		{
			if (SA_prom[0] == bad_clone_list[i].SAprefix[0] &&
				SA_prom[1] == bad_clone_list[i].SAprefix[1] &&
				SA_prom[2] == bad_clone_list[i].SAprefix[2])
			{
				if (wordlength == 2)
				{
					name = bad_clone_list[i].name16;
				} else {
					name = bad_clone_list[i].name8;
				}
				break;
			}
		}
		if (bad_clone_list[i].name8 == NULL)
		{
			netdev_info(dev, " not found (invalid signature %2.2x %2.2x).\n",
				SA_prom[14], SA_prom[15]);
			ret = ENXIO;
			goto err_out;
		}
#else
		netdev_info(dev, " not found.\n");
		ret = ENXIO;
		goto err_out;
#endif
	}

	/* The IRQ is the 200 Hz poll: no autodetection */

	/* Snarf the interrupt now.  There's no point in waiting since we cannot
	   share and the board will usually be enabled. */
	netusbee_old_vector = (void (*)(void)) Setexc(ATARI_ETHERNEC_VECTOR, (long) netusbee_interrupt);

	dev->base_addr = (uchar *) ioaddr;

	memcpy(dev->hwlocal.adr.bytes, SA_prom, ETH_ALEN);

	ei_status.name = name;
	ei_status.tx_start_page = start_page;
	ei_status.stop_page = stop_page;

	/* Use 16-bit mode only if this wasn't overridden by DCR_VAL */
	ei_status.word16 = (wordlength == 2 && (DCR_VAL & 0x01));

	ei_status.rx_start_page = start_page + TX_PAGES;
#ifdef PACKETBUF_MEMSIZE
	 /* Allow the packet buffer size to be overridden by know-it-alls. */
	ei_status.stop_page = ei_status.tx_start_page + PACKETBUF_MEMSIZE;
#endif

	ei_status.reset_8390 = &ne_reset_8390;
	ei_status.block_input = &ne_block_input;
	ei_status.block_output = &ne_block_output;
	ei_status.get_8390_hdr = &ne_get_8390_hdr;

	__NS8390_init(dev, 0);

	UNUSED(ei_local);
	return 0;

err_out:
	return ret;
}

/* Hard reset the card.  This used to pause for the same period that a
   8390 reset command required, but that shouldn't be necessary. */

static void ne_reset_8390(struct netif *dev)
{
	unsigned long reset_start_time = jiffies;
	struct ei_device *ei_local = dev->data;

	UNUSED(ei_local);
	netif_dbg(ei_local, hw, dev, "resetting the 8390 t=%ld...\n", jiffies);

	/* DON'T change these to inb_p/outb_p or reset will fail on clones. */
	outb(inb(NE_BASE + NE_RESET), NE_BASE + NE_RESET);

	ei_status.txing = 0;
	ei_status.dmaing = 0;

	/* This check _should_not_ be necessary, omit eventually. */
	while ((inb_p(NE_BASE+EN0_ISR) & ENISR_RESET) == 0)
		if (time_after(jiffies, reset_start_time + 2*HZ/100)) {
			netdev_err(dev, "ne_reset_8390() did not complete.\n");
			break;
		}
	outb_p(ENISR_RESET, NE_BASE + EN0_ISR);	/* Ack intr. */
}

/* Grab the 8390 specific header. Similar to the block_input routine, but
   we don't need to be concerned with ring wrap as the header will be at
   the start of a page, so we optimize accordingly. */

static void ne_get_8390_hdr(struct netif *dev, struct e8390_pkt_hdr *hdr, long ring_page)
{
	unsigned long nic_base = (unsigned long) dev->base_addr;

	/* This *shouldn't* happen. If it does, it's the last thing you'll see */

	if (ei_status.dmaing)
	{
		netdev_err(dev, "DMAing conflict in ne_get_8390_hdr "
			   "[DMAstat:%d][irqlock:%d].\n",
			   ei_status.dmaing, ei_status.irqlock);
		return;
	}

	ei_status.dmaing |= 0x01;
	outb_p(E8390_NODMA+E8390_PAGE0+E8390_START, nic_base+ NE_CMD);
	outb_p(sizeof(struct e8390_pkt_hdr), nic_base + EN0_RCNTLO);
	outb_p(0, nic_base + EN0_RCNTHI);
	outb_p(0, nic_base + EN0_RSARLO);		/* On page boundary */
	outb_p(ring_page, nic_base + EN0_RSARHI);
	outb_p(E8390_RREAD+E8390_START, nic_base + NE_CMD);

	/* word16 is never set on Atari (DCR_VAL 0x48), byte transfers only */
	insb(NE_BASE + NE_DATAPORT, hdr, sizeof(struct e8390_pkt_hdr));

	outb_p(ENISR_RDC, nic_base + EN0_ISR);	/* Ack intr. */
	ei_status.dmaing &= ~0x01;

	/* le16_to_cpus(&hdr->count) */
	hdr->count = (hdr->count << 8) | (hdr->count >> 8);
}

/* Block input and output, similar to the Crynwr packet driver.  If you
   are porting to a new ethercard, look at the packet driver source for hints.
   The NEx000 doesn't share the on-board packet memory -- you have to put
   the packet out through the "remote DMA" dataport using outb. */

static void ne_block_input(struct netif *dev, long count, BUF *skb, long ring_offset)
{
	unsigned long nic_base = (unsigned long) dev->base_addr;
	char *buf = skb->dstart;

	/* This *shouldn't* happen. If it does, it's the last thing you'll see */
	if (ei_status.dmaing)
	{
		netdev_err(dev, "DMAing conflict in ne_block_input "
			   "[DMAstat:%d][irqlock:%d].\n",
			   ei_status.dmaing, ei_status.irqlock);
		return;
	}
	ei_status.dmaing |= 0x01;
	outb_p(E8390_NODMA+E8390_PAGE0+E8390_START, nic_base+ NE_CMD);
	outb_p(count & 0xff, nic_base + EN0_RCNTLO);
	outb_p(count >> 8, nic_base + EN0_RCNTHI);
	outb_p(ring_offset & 0xff, nic_base + EN0_RSARLO);
	outb_p(ring_offset >> 8, nic_base + EN0_RSARHI);
	outb_p(E8390_RREAD+E8390_START, nic_base + NE_CMD);

	insb(NE_BASE + NE_DATAPORT, buf, count);

	outb_p(ENISR_RDC, nic_base + EN0_ISR);	/* Ack intr. */
	ei_status.dmaing &= ~0x01;
}

static void ne_block_output(struct netif *dev, long count,
		const unsigned char *buf, const long start_page)
{
	unsigned long nic_base = NE_BASE;
	unsigned long dma_start;

	/* Round the count up for word writes.  Do we need to do this?
	   What effect will an odd byte count have on the 8390?
	   I should check someday. */

	if (ei_status.word16 && (count & 0x01))
		count++;

	/* This *shouldn't* happen. If it does, it's the last thing you'll see */
	if (ei_status.dmaing)
	{
		netdev_err(dev, "DMAing conflict in ne_block_output."
			   "[DMAstat:%d][irqlock:%d]\n",
			   ei_status.dmaing, ei_status.irqlock);
		return;
	}
	ei_status.dmaing |= 0x01;
	/* We should already be in page 0, but to be safe... */
	outb_p(E8390_PAGE0+E8390_START+E8390_NODMA, nic_base + NE_CMD);

#ifdef NE_RW_BUGFIX
	/* Handle the read-before-write bug the same way as the
	   Crynwr packet driver -- the NatSemi method doesn't work.
	   Actually this doesn't always work either, but if you have
	   problems with your NEx000 this is better than nothing! */

	outb_p(0x42, nic_base + EN0_RCNTLO);
	outb_p(0x00,   nic_base + EN0_RCNTHI);
	outb_p(0x42, nic_base + EN0_RSARLO);
	outb_p(0x00, nic_base + EN0_RSARHI);
	outb_p(E8390_RREAD+E8390_START, nic_base + NE_CMD);
	/* Make certain that the dummy read has occurred. */
	udelay(6);
#endif

	outb_p(ENISR_RDC, nic_base + EN0_ISR);

	/* Now the normal output. */
	outb_p(count & 0xff, nic_base + EN0_RCNTLO);
	outb_p(count >> 8,   nic_base + EN0_RCNTHI);
	outb_p(0x00, nic_base + EN0_RSARLO);
	outb_p(start_page, nic_base + EN0_RSARHI);

	outb_p(E8390_RWRITE+E8390_START, nic_base + NE_CMD);

	outsb(NE_BASE + NE_DATAPORT, buf, count);

	dma_start = jiffies;

	while ((inb_p(nic_base + EN0_ISR) & ENISR_RDC) == 0)
		if (time_after(jiffies, dma_start + 2*HZ/100)) {		/* 20ms */
			netdev_warn(dev, "timeout waiting for Tx RDC.\n");
			ne_reset_8390(dev);
			__NS8390_init(dev, 1);
			break;
		}

	outb_p(ENISR_RDC, nic_base + EN0_ISR);	/* Ack intr. */
	ei_status.dmaing &= ~0x01;
}


/*
 * The MiNTNet side: what net_device, its ops and the platform code do on
 * Linux.
 */

static long	netusbee_open		(struct netif *);
static long	netusbee_close		(struct netif *);
static long	netusbee_output		(struct netif *, BUF *, const char *, short, short);
static long	netusbee_ioctl		(struct netif *, short, long);
static long	netusbee_config		(struct netif *, struct ifopt *);
static void	netusbee_igmp_mac_filter (struct netif *, ulong, char);
static void	netusbee_timeout	(struct netif *);

/*
 * The "interrupt": called from netusbee_interrupt on every 200 Hz tick
 * at IPL 6. disable_irq() from the transmit path masks it, and a card
 * that is down is stopped with its interrupts masked, so there is
 * nothing to look at then.
 */
void _cdecl
ne_poll (void)
{
	struct netif *dev = &if_netusbee;
	struct ei_device *ei_local = dev->data;

	if (ei_local->irq_disabled || !netif_running (dev))
		return;

	__ei_interrupt (dev);
}

/*
 * This gets called when someone makes an 'ifconfig up' on this interface
 * and the interface was down before.
 */
static long
netusbee_open (struct netif *nif)
{
	return __ei_open (nif);
}

/*
 * Opposite of netusbee_open(), is called when 'ifconfig down' on this interface
 * is done and the interface was up before.
 */
static long
netusbee_close (struct netif *nif)
{
	return __ei_close (nif);
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
netusbee_output (struct netif *nif, BUF *buf, const char *hwaddr, short hwlen, short pktype)
{
	struct ei_device *ei_local = nif->data;
	BUF *nbuf;

	nbuf = eth_build_hdr (buf, nif, hwaddr, pktype);
	if (nbuf == NULL)
	{
		nif->out_errors++;
		return ENOMEM;
	}

	if (nif->bpf)
		bpf_input (nif, nbuf);

	if (ei_local->queue_stopped || __ei_start_xmit (nbuf, nif) == NETDEV_TX_BUSY)
	{
		/* if_enqueue() frees the buffer when the queue is full */
		if (if_enqueue (&nif->snd, nbuf, nbuf->info))
			nif->out_errors++;
	}

	return 0;
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
netusbee_ioctl (struct netif *nif, short cmd, long arg)
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
				__ei_set_multicast_list (nif);
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
			/*
			 * Interface configuration, handled by netusbee_config()
			 */
			ifr = (struct ifreq *) arg;
			return netusbee_config (nif, ifr->ifru.data);
	}

	return ENOSYS;
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
 * Interface configuration via SIOCSIFOPT. The ioctl is passed a
 * struct ifreq *ifr. ifr->ifru.data points to a struct ifopt, which
 * we get as the second argument here.
 *
 * If the user MUST configure some parameters before the interface
 * can run make sure that netusbee_open() fails unless all the necessary
 * parameters are set.
 *
 * Return values	meaning
 * ENOSYS		option not supported
 * ENOENT		invalid option value
 * 0			Ok
 */
static long
netusbee_config (struct netif *nif, struct ifopt *ifo)
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
 * is kept here, with up to 16 groups, beyond that every multicast is
 * accepted.
 */
static void
netusbee_igmp_mac_filter (struct netif *nif, ulong group, char action)
{
	struct ei_device *ei_local = nif->data;
	long i, slot = -1;

	for (i = 0; i < 16; i++)
	{
		if (ei_local->mc_refs[i] && ei_local->mc_group[i] == group)
		{
			slot = i;
			break;
		}
		if (slot < 0 && !ei_local->mc_refs[i])
			slot = i;
	}

	if (action == IGMP_ADD_MAC_FILTER)
	{
		if (slot < 0)
			ei_local->mc_overflow++;
		else
		{
			if (!ei_local->mc_refs[slot])
			{
				ei_local->mc_group[slot] = group;
				ei_local->mc_count++;
			}
			ei_local->mc_refs[slot]++;
		}
	}
	else
	{
		if (slot >= 0 && ei_local->mc_refs[slot] && ei_local->mc_group[slot] == group)
		{
			if (--ei_local->mc_refs[slot] == 0)
				ei_local->mc_count--;
		}
		else if (ei_local->mc_overflow)
			ei_local->mc_overflow--;
	}

	if (nif->flags & IFF_UP)
		__ei_set_multicast_list (nif);
}

/*
 * Called every IF_SLOWTIMEOUT (one second) while the interface is up:
 * the netdev watchdog.
 */
static void
netusbee_timeout (struct netif *nif)
{
	struct ei_device *ei_local = nif->data;

	/* dev_watchdog(): a stopped queue that saw no transmission */
	if (ei_local->queue_stopped &&
	    time_after (jiffies, ei_local->trans_start + TX_TIMEOUT))
		__ei_tx_timeout (nif);
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
	struct netif *nif = &if_netusbee;
	struct ei_device *ei_local = &ei_priv;
	const u8 *mac;

	/* the hwreg_present() check of the platform code: is there a
	 * cartridge port at all
	 */
	if (!hwreg_present ((const volatile void *) ENEC_ISA_IO_B (ATARI_ETHERNEC_BASE)))
	{
		c_conws ("NetUSBee/EtherNEC: no cartridge port\r\n");
		return -1;
	}

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

	/*
	 * Set interface hardware and broadcast addresses. The hardware
	 * address comes from the card's PROM in ne_probe1().
	 */
	memcpy (nif->hwbrcst.adr.bytes, "\377\377\377\377\377\377", ETH_ALEN);

	/*
	 * Set length of send and receive queue. IF_MAXQ is a good value.
	 */
	nif->rcv.maxqlen = IF_MAXQ;
	nif->snd.maxqlen = IF_MAXQ;
	/*
	 * Setup pointers to service functions
	 */
	nif->open = netusbee_open;
	nif->close = netusbee_close;
	nif->output = netusbee_output;
	nif->ioctl = netusbee_ioctl;
	nif->igmp_mac_filter = netusbee_igmp_mac_filter;
	/*
	 * Timer function that is called every second.
	 */
	nif->timeout = netusbee_timeout;

	/*
	 * Here you could attach some more data your driver may need
	 */
	nif->data = ei_local;

	/*
	 * Number of packets the hardware can receive in fast succession,
	 * 0 means unlimited. The card has 8 KB in 8 bit mode.
	 */
	nif->maxpackets = 4;

	if (ne_probe1 (nif, ATARI_ETHERNEC_BASE) != 0)
	{
		c_conws ("NetUSBee/EtherNEC: no NE2000 compatible card found\r\n");
		return -1;
	}

	/*
	 * Register the interface.
	 */
	if_register (nif);

	mac = nif->hwlocal.adr.bytes;
	ksprintf (message, "NetUSBee/EtherNEC driver v1.0 (%s%d): %s at 0x%lx, %02x:%02x:%02x:%02x:%02x:%02x\r\n",
		nif->name, nif->unit, ei_local->name, (unsigned long) nif->base_addr,
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	c_conws (message);

	return 0;
}
