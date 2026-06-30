// SPDX-License-Identifier: GPL-2.0
// Y Ddraig low-level DUART I/O.

#include <asm/mackerel.h>

/*
 * Re-initialise DUART channel A to 38400 8-N-1.
 * Called from config_BSP so the kernel never relies on the bootloader
 * having left the DUART in any particular state.
 */
void duart_init(void)
{
	MEM(DUART1_CRA)  = 0x20;  /* reset receiver A */
	MEM(DUART1_CRA)  = 0x10;  /* reset MR pointer to MR1A */
	MEM(DUART1_ACR)  = 0x70;  /* BRG set 1 */
	MEM(DUART1_MR1A) = 0x13;  /* 8-bit, no parity (pointer auto-advances to MR2A) */
	MEM(DUART1_MR2A) = 0x07;  /* 1 stop bit */
	MEM(DUART1_CSRA) = 0xCC;  /* 38400 baud TX and RX */
	MEM(DUART1_CRA)  = 0x05;  /* enable TX and RX */
}

void duart_putc(char c)
{
	while ((MEM(DUART1_SRA) & 0x04) == 0)
		;
	MEM(DUART1_TBA) = c;
	if (c == '\n')
		duart_putc('\r');
}

char duart_getc(void)
{
	while ((MEM(DUART1_SRA) & 0x01) == 0)
		;
	return MEM(DUART1_RBA);
}
