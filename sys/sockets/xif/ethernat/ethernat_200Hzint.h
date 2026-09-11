/*
 * Low level routines for the EtherNAT driver, see ethernat_200Hzint.S.
 *
 * This file belongs to FreeMiNT. It's not in the original MiNT 1.12
 * distribution. See the file CHANGES for a detailed log of changes.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#ifndef _ethernat_200Hzint_h
#define _ethernat_200Hzint_h

/* handler that was installed on the vector before ours */
extern void (*ethernat_old_vector)(void);

/* the vector entry itself */
void ethernat_interrupt (void);

/* the C handler it calls */
long _cdecl smc_interrupt (void);

/* bus error protected read: 1 if the byte at addr can be read */
long hwreg_present (const volatile void *addr);

#endif /* _ethernat_200Hzint_h */
