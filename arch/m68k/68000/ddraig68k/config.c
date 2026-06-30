// SPDX-License-Identifier: GPL-2.0
// Y Ddraig (ddraig68k) board init and early console.

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/console.h>
#include <linux/platform_device.h>
#include <asm/machdep.h>
#include <asm/mackerel.h>
#include <asm/traps.h>
#include <linux/platform_data/pata_mackerel.h>

extern void legacy_timer_tick(unsigned long ticks);

static irqreturn_t ddraig_tick(int irq, void *dummy)
{
	PIT_WRITE(PIT_TSR, 0x01);
	legacy_timer_tick(1);
	return IRQ_HANDLED;
}

static void ddraig_sched_init(void)
{
	int ret;
	unsigned long reload = PIT_CLK_HZ / PIT_PRESCALER / HZ;

	ret = request_irq(IRQ_NUM_TIMER, ddraig_tick, IRQF_TIMER, "timer", NULL);
	if (ret) {
		pr_err("Y Ddraig: cannot get timer IRQ: %d\n", ret);
		return;
	}

	/* MC68230 PIT: CLK/32 mode, zero-detect interrupt enabled */
	PIT_WRITE(PIT_PGCR, 0x30);
	PIT_WRITE(PIT_CPRH, (reload >> 16) & 0xFF);
	PIT_WRITE(PIT_CPRM, (reload >>  8) & 0xFF);
	PIT_WRITE(PIT_CPRL, (reload      ) & 0xFF);
	PIT_WRITE(PIT_TSR,  0x01);		// clear any stale zero-detect status
	PIT_WRITE(PIT_TCR,  0xE1);		// CLK/32, ZDE, TE
}

static void ddraig_console_write(struct console *co, const char *str,
				 unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count && str[i]; i++)
		duart_putc(str[i]);
}

static struct console ddraig_console_driver = {
	.name  = "mackconsole",
	.flags = CON_PRINTBUFFER | CON_BOOT,
	.index = -1,
	.write = ddraig_console_write,
};

static void ddraig_reset(void)
{
	local_irq_disable();
}

static struct resource uart_res[] = {
	{
		.start = DUART1_BASE,
		.end   = DUART1_BASE + 31,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = IRQ_NUM_DUART,
		.end   = IRQ_NUM_DUART,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device xr68c681_device = {
	.name          = "uart-xr68c681",
	.id            = 0,
	.num_resources = ARRAY_SIZE(uart_res),
	.resource      = uart_res,
};

#if 0
static struct resource ide_res[] = {
	{
		/* Command block: 8 regs at 2-byte stride (A1 selects register) */
		.start = IDE_BASE,
		.end   = IDE_BASE + 0x0F,
		.flags = IORESOURCE_MEM,
	},
	{
		/* Control block: alt status / device control */
		.start = IDE_CTL_BASE,
		.end   = IDE_CTL_BASE + 0x01,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = IRQ_NUM_IDE,
		.end   = IRQ_NUM_IDE,
		.flags = IORESOURCE_IRQ,
	},
};

static struct pata_mackerel_pdata ide_pdata = {
	.low_byte_bitrev = false,
};

static struct platform_device ide_device = {
	.name          = "pata-mackerel",
	.id            = -1,
	.num_resources = ARRAY_SIZE(ide_res),
	.resource      = ide_res,
	.dev = {
		.platform_data = &ide_pdata,
	},
};
#endif

extern void mackerel_addr_err(void);
extern void mackerel_bus_err(void);

void __init config_BSP(char *command, int len)
{
	/* Stop the PIT timer left running by DdraigOS before it fires as IRQ3 */
	PIT_WRITE(PIT_TCR, 0xE0);

	pr_info(MACKEREL_BOARD_NAME ": Y Ddraig 68000 computer\n");

	_ramvec[2] = (e_vector)mackerel_bus_err;
	_ramvec[3] = (e_vector)mackerel_addr_err;

	/* Bring up DUART channel A to 38400 8-N-1, independent of bootloader state */
	duart_init();
	/* Disable DUART interrupts; let the xr68c681 driver re-enable them */
	MEM(DUART1_IMR) = 0;

	mach_reset      = ddraig_reset;
	mach_sched_init = ddraig_sched_init;

	register_console(&ddraig_console_driver);
}

static int __init ddraig_platform_init(void)
{
	if (platform_device_register(&xr68c681_device))
		panic("Could not register DUART device");

	/* IDE probe deferred: async_synchronize_full() blocks ~30 s waiting
	 * for the libata timeout when no drive is connected.
	 * Re-enable once an IDE drive is wired up.
	 */
#if 0
	if (platform_device_register(&ide_device))
		pr_err("Y Ddraig: could not register IDE device\n");
#endif

	return 0;
}
arch_initcall(ddraig_platform_init);
