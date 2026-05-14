| Fake an IKBD ACIA interrupt by vectoring through $118.
|
| Used by drivers that hold the CPU at IPL >= 6 long enough for the
| ACIA's UART to drop characters.  Polls the ACIA RDRF bit (handled
| by the caller via ikbd_int_pending()) and, when set, invokes the
| installed IKBD handler so the byte is read out before it is lost.
|
| Originally lifted from the Lightning VME/ST USB driver by
| Ingo Uhlemann / Christian Zietz; ported to the NetUSBee/EtherNAT
| USB drivers by Roger Burrows and to Svethlana by Joni Valtanen.

#include "mint/asmdefs.h"

#ifndef __mcoldfire__

	.globl	SYM(fake_ikbd_int)

	.text

SYM(fake_ikbd_int):
	tst.w	0x59e.w			| handle 68000/68030 stack-frame difference
	beq.b	1f
	move.w	#0x118,-(sp)		| push vector offset
1:
	pea	2f(pc)			| return address
	move	sr,-(sp)		|   and SR
	move.l	0x118,a0
	jmp	(a0)			| go to interrupt handler
2:
	rts

#endif /* !__mcoldfire__ */
