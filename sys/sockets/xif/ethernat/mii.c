/*
 * MII interface library: drivers/net/mii.c (Linux 7.3), the part
 * smc91x uses. Names are kept as in Linux so that fixes can be carried
 * over.
 *
 * This file belongs to FreeMiNT. It's not in the original MiNT 1.12
 * distribution. See the file CHANGES for a detailed log of changes.
 *
 * The kernel is built with -mshort, so Linux "int" is "long" here.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

/*

	mii.c: MII interface library

	Maintained by Jeff Garzik <jgarzik@pobox.com>
	Copyright 2001,2002 Jeff Garzik

	Various code came from myson803.c and other files by
	Donald Becker.  Copyright:

		Written 1998-2002 by Donald Becker.

		This software may be used and distributed according
		to the terms of the GNU General Public License (GPL),
		incorporated herein by reference.  Drivers based on
		or derived from this code fall under the GPL and must
		retain the authorship, copyright and license notice.
		This file is not a complete program and may only be
		used when the entire operating system is licensed
		under the GPL.

		The author may be reached as becker@scyld.com, or C/O
		Scyld Computing Corporation
		410 Severn Ave., Suite 210
		Annapolis MD 21403


 */

# include "global.h"

# include "inet4/if.h"

# include "91c111.h"
# include "mii.h"

# include <stdarg.h>


/*
 * netdev_info() from mii_check_media() may run in interrupt context,
 * where nothing may print on FreeMiNT: the message is stored for the
 * driver to print from its slow timer.
 */
void mii_netdev_info(struct mii_if_info *mii, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	kvsprintf(mii->link_msg, sizeof(mii->link_msg), fmt, args);
	va_end(args);
	mii->link_msg_pending = 1;
}

# define netdev_info(dev, fmt, ...)	mii_netdev_info(mii, fmt, ##__VA_ARGS__)

/**
 * mii_link_ok - is link status up/ok
 * @mii: the MII interface
 *
 * Returns 1 if the MII reports link status up/ok, 0 otherwise.
 */
long mii_link_ok (struct mii_if_info *mii)
{
	/* first, a dummy read, needed to latch some MII phys */
	mii->mdio_read(mii->dev, mii->phy_id, MII_BMSR);
	if (mii->mdio_read(mii->dev, mii->phy_id, MII_BMSR) & BMSR_LSTATUS)
		return 1;
	return 0;
}

/**
 * mii_check_media - check the MII interface for a carrier/speed/duplex change
 * @mii: the MII interface
 * @ok_to_print: OK to print link up/down messages
 * @init_media: OK to save duplex mode in @mii
 *
 * Returns 1 if the duplex mode changed, 0 if not.
 * If the media type is forced, always returns 0.
 */
u32 mii_check_media (struct mii_if_info *mii,
		     u32 ok_to_print,
		     u32 init_media)
{
	u32 old_carrier, new_carrier;
	long advertise, lpa, media, duplex;
	long lpa2 = 0;

	/* check current and old link status */
	old_carrier = netif_carrier_ok(mii->dev) ? 1 : 0;
	new_carrier = (u32) mii_link_ok(mii);

	/* if carrier state did not change, this is a "bounce",
	 * just exit as everything is already set correctly
	 */
	if ((!init_media) && (old_carrier == new_carrier))
		return 0; /* duplex did not change */

	/* no carrier, nothing much to do */
	if (!new_carrier) {
		netif_carrier_off(mii->dev);
		if (ok_to_print)
			netdev_info(mii->dev, "link down");
		return 0; /* duplex did not change */
	}

	/*
	 * we have carrier, see who's on the other end
	 */
	netif_carrier_on(mii->dev);

	if (mii->force_media) {
		if (ok_to_print)
			netdev_info(mii->dev, "link up");
		return 0; /* duplex did not change */
	}

	/* get MII advertise and LPA values */
	if ((!init_media) && (mii->advertising))
		advertise = mii->advertising;
	else {
		advertise = mii->mdio_read(mii->dev, mii->phy_id, MII_ADVERTISE);
		mii->advertising = advertise;
	}
	lpa = mii->mdio_read(mii->dev, mii->phy_id, MII_LPA);

	/* figure out media and duplex from advertise and LPA values */
	media = mii_nway_result(lpa & advertise);
	duplex = (media & ADVERTISE_FULL) ? 1 : 0;

	if (ok_to_print)
		netdev_info(mii->dev, "link up, %ldMbps, %s-duplex, lpa 0x%04lX",
			    lpa2 & 0 ? 1000L :
			    media & (ADVERTISE_100FULL | ADVERTISE_100HALF) ?
			    100L : 10L,
			    duplex ? "full" : "half",
			    lpa);

	if ((init_media) || (mii->full_duplex != duplex)) {
		mii->full_duplex = duplex;
		return 1; /* duplex changed */
	}

	return 0; /* duplex did not change */
}
