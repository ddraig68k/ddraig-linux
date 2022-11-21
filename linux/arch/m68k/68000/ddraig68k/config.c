#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <asm/traps.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/siginfo.h>
#include <linux/kallsyms.h>
#include <linux/signal.h>
#include <linux/ptrace.h>
#include <linux/console.h>
#include <asm/irq.h>
#include "ddraig68k.h"

static u32 ddraig_tick_count;
static irq_handler_t timer_interrupt;

static void ddraig_console_write(struct console *co, const char *str, unsigned int count)
{
	unsigned i = 0;

	while (str[i] != 0 && i < count)
	{
		ddraig_putc(str[i]);
		i++;
	}
}

static struct console ddraig_console_driver = {
	.name = "ddraigconsole",
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.write = ddraig_console_write
};

static irqreturn_t hw_tick(int irq, void *dummy)
{
	ddraig_tick_count += 10;
	return timer_interrupt(irq, dummy);
}

static struct irqaction ddraig_timer_irq = {
	.name = "timer",
	.flags = /*IRQF_DISABLED | */ IRQF_TIMER,
	.handler = hw_tick,
};

void ddraig_reset(void)
{
	local_irq_disable();
}

static cycle_t ddraig_read_clk(struct clocksource *cs)
{
	unsigned long flags;
	u32 cycles;

	local_irq_save(flags);
	cycles = ddraig_tick_count + 100; // TODO: This is definitely not the right value
	local_irq_restore(flags);

	return cycles;
}

static struct clocksource ddraig_clk = {
	.name = "timer",
	.rating = 250,
	.read = ddraig_read_clk,
	.mask = CLOCKSOURCE_MASK(32),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

void ddraig_sched_init(irq_handler_t handler)
{
	setup_irq(1, &ddraig_timer_irq);

	// Setup DUART timer as 50 Hz interrupt
	MEM(DUART_IVR) = 0x40;		 // Interrupt base register
	MEM(DUART_ACR) = 0xF0;		 // Set timer mode X/16
	MEM(DUART_IMR) = 0b00001000; // Unmask counter interrupt
	MEM(DUART_CUR) = 0x09;		 // Counter upper byte, (3.6864MHz / 2 / 16 / 0x900) = 50 Hz
	MEM(DUART_CLR) = 0x00;		 // Counter lower byte
	MEM(DUART_OPR);				 // Start counter

	clocksource_register_hz(&ddraig_clk, 10 * 100); // TODO: this should be calculated properly from the interrupt rate and CPU speed and all that

	timer_interrupt = handler;
}

void __init config_BSP(char *command, int len)
{
	printk(KERN_INFO "Ddraig68k support by Stephen Moody <steve@ddraig68k.com>\n");

	mach_reset = ddraig_reset;
	mach_sched_init = ddraig_sched_init;

	register_console(&ddraig_console_driver);
}
