/*
 * Low level routines for the NetUSBee/EtherNEC driver, see netusbee_int.S.
 *
 * This file belongs to FreeMiNT. It's not in the original MiNT 1.12
 * distribution. See the file CHANGES for a detailed log of changes.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#ifndef _netusbee_int_h
#define _netusbee_int_h

/* handler that was on the 200 Hz vector before ours */
extern void (*netusbee_old_vector)(void);

/* the vector entry itself */
void netusbee_interrupt (void);

/* the C poll it calls */
void _cdecl ne_poll (void);

/* bus error protected read: 1 if the byte at addr can be read */
long hwreg_present (const volatile void *addr);

#endif /* _netusbee_int_h */
