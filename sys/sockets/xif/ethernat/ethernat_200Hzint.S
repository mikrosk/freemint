|
| Low level routines for the EtherNAT driver.
|
| This file belongs to FreeMiNT. It's not in the original MiNT 1.12
| distribution. See the file CHANGES for a detailed log of changes.
|
| ethernat_interrupt is the entry for vector 0xc4 (Linux IRQ 140): it
| calls smc_interrupt() with the scratch registers saved. smc_interrupt()
| claims every interrupt, as on Linux, so nothing is passed on.
|
| hwreg_present() is the bus error protected read the Linux platform
| code uses to see whether the EtherNAT is there; the same code as the
| kernel's own test_byte_rd() in sys/arch/detect.S.
|
| This file is free software; you can redistribute it and/or modify
| it under the terms of the GNU General Public License as published by
| the Free Software Foundation; either version 2, or (at your option)
| any later version.
|

#include "mint/asmdefs.h"
#include "mint/xbra.h"

	.globl	SYM(ethernat_old_vector)
	.globl	SYM(ethernat_interrupt)
	.globl	SYM(hwreg_present)
	.extern	SYM(smc_interrupt)

	.text

	dc.l	XBRA_MAGIC
	dc.l	MINT_MAGIC
SYM(ethernat_old_vector):
	ds.l	1
SYM(ethernat_interrupt):
	movem.l	d0-d2/a0-a2,-(sp)
	jsr	SYM(smc_interrupt)
	movem.l	(sp)+,d0-d2/a0-a2
	rte

| long hwreg_present(const volatile void *addr): 1 if a byte can be read
SYM(hwreg_present):
	move.l	a2,-(sp)
	move.l	sp,a0
	move.l	(8).w,a1
	lea	berr(pc),a2
	move.l	a2,(8).w
	nop

	clr.l	d0
	move.l	8(sp),a2
	tst.b	(a2)

	moveq	#0x01,d0	| passed

berr:	move.l	a1,(8).w
	move.l	a0,sp
	nop
	move.l	(sp)+,a2
	rts
