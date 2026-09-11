/*
 * MII interface library: include/linux/mii.h and include/uapi/linux/mii.h
 * (Linux 7.3) as far as smc91x needs them. See mii.c.
 *
 * This file belongs to FreeMiNT. It's not in the original MiNT 1.12
 * distribution. See the file CHANGES for a detailed log of changes.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* include/linux/mii.h */
/* SPDX-License-Identifier: GPL-2.0 */
/*
 * linux/mii.h: definitions for MII-compatible transceivers
 * Originally drivers/net/sunhme.h.
 *
 * Copyright (C) 1996, 1999, 2001 David S. Miller (davem@redhat.com)
 */

/* include/uapi/linux/mii.h */
/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * linux/mii.h: definitions for MII-compatible transceivers
 * Originally drivers/net/sunhme.h.
 *
 * Copyright (C) 1996, 1999, 2001 David S. Miller (davem@redhat.com)
 */


#ifndef _mii_h
#define _mii_h

/* Generic MII registers. */
#define MII_BMCR		0x00	/* Basic mode control register */
#define MII_BMSR		0x01	/* Basic mode status register  */
#define MII_PHYSID1		0x02	/* PHYS ID 1                   */
#define MII_PHYSID2		0x03	/* PHYS ID 2                   */
#define MII_ADVERTISE		0x04	/* Advertisement control reg   */
#define MII_LPA			0x05	/* Link partner ability reg    */

/* Basic mode control register. */
#define BMCR_FULLDPLX		0x0100	/* Full duplex                 */
#define BMCR_ANRESTART		0x0200	/* Auto negotiation restart    */
#define BMCR_PDOWN		0x0800	/* Enable low power state      */
#define BMCR_ANENABLE		0x1000	/* Enable auto negotiation     */
#define BMCR_SPEED100		0x2000	/* Select 100Mbps              */
#define BMCR_RESET		0x8000	/* Reset to default state      */

/* Basic mode status register. */
#define BMSR_LSTATUS		0x0004	/* Link status                 */
#define BMSR_ANEGCAPABLE	0x0008	/* Able to do auto-negotiation */
#define BMSR_10HALF		0x0800	/* Can do 10mbps, half-duplex  */
#define BMSR_10FULL		0x1000	/* Can do 10mbps, full-duplex  */
#define BMSR_100HALF		0x2000	/* Can do 100mbps, half-duplex */
#define BMSR_100FULL		0x4000	/* Can do 100mbps, full-duplex */
#define BMSR_100BASE4		0x8000	/* Can do 100mbps, 4k packets  */

/* Advertisement control register. */
#define ADVERTISE_CSMA		0x0001	/* Only selector supported     */
#define ADVERTISE_10HALF	0x0020	/* Try for 10mbps half-duplex  */
#define ADVERTISE_10FULL	0x0040	/* Try for 10mbps full-duplex  */
#define ADVERTISE_100HALF	0x0080	/* Try for 100mbps half-duplex */
#define ADVERTISE_100FULL	0x0100	/* Try for 100mbps full-duplex */
#define ADVERTISE_100BASE4	0x0200	/* Try for 100mbps 4k packets  */

#define ADVERTISE_FULL		(ADVERTISE_100FULL | ADVERTISE_10FULL | \
				  ADVERTISE_CSMA)

/* Link partner ability register. */
#define LPA_10HALF		0x0020	/* Can do 10mbps half-duplex   */
#define LPA_10FULL		0x0040	/* Can do 10mbps full-duplex   */
#define LPA_100HALF		0x0080	/* Can do 100mbps half-duplex  */
#define LPA_100FULL		0x0100	/* Can do 100mbps full-duplex  */
#define LPA_100BASE4		0x0200	/* Can do 100mbps 4k packets   */

/* what smc91x uses of struct mii_if_info */
struct mii_if_info {
	long phy_id;
	long advertising;
	long phy_id_mask;
	long reg_num_mask;

	unsigned long full_duplex : 1;	/* is full duplex? */
	unsigned long force_media : 1;	/* is autoneg. disabled? */
	unsigned long supports_gmii : 1; /* are GMII registers supported? */

	struct netif *dev;
	long (*mdio_read) (struct netif *dev, long phy_id, long location);
	void (*mdio_write) (struct netif *dev, long phy_id, long location, long val);

	/* netdev_info() of the link state, which may be found from
	 * interrupt context where nothing may print: kept here for the
	 * driver to print later
	 */
	char link_msg[96];
	short link_msg_pending;
};

/* the carrier state, kept by the driver (net_device on Linux) */
long netif_carrier_ok(struct netif *dev);
void netif_carrier_on(struct netif *dev);
void netif_carrier_off(struct netif *dev);

/* the deferred netdev_info() */
void mii_netdev_info(struct mii_if_info *mii, const char *fmt, ...)
	__attribute__((format(printf, 2, 3)));

long mii_link_ok (struct mii_if_info *mii);
u32 mii_check_media (struct mii_if_info *mii,
		     u32 ok_to_print,
		     u32 init_media);

/**
 * mii_nway_result
 * @negotiated: value of MII ANAR and'd with ANLPAR
 *
 * Given a set of MII abilities, check to see which bits wins.
 */
static inline u32 mii_nway_result (u32 negotiated)
{
	u32 ret;

	if (negotiated & LPA_100FULL)
		ret = LPA_100FULL;
	else if (negotiated & LPA_100BASE4)
		ret = LPA_100BASE4;
	else if (negotiated & LPA_100HALF)
		ret = LPA_100HALF;
	else if (negotiated & LPA_10FULL)
		ret = LPA_10FULL;
	else
		ret = LPA_10HALF;

	return ret;
}

#endif /* _mii_h */
