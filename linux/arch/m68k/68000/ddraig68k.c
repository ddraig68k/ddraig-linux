/***************************************************************************/

/*
 *  m68EZ328.c - 68EZ328 specific config
 *
 *  Copyright (C) 1993 Hamish Macdonald
 *  Copyright (C) 1999 D. Jeff Dionne
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

/***************************************************************************/

#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/rtc.h>
#include <asm/pgtable.h>
#include <asm/machdep.h>

#include <asm/irq.h>

/***************************************************************************/
static irq_handler_t timer_interrupt;


#define RTC_BASE        0x00F7F400       // Base address of the RTC72421 clock controller

#define RTC_READ(x)         (*((volatile uint8_t *) RTC_BASE + (x) ))
#define RTC_WRITE(x, y)     (*((uint8_t *) RTC_BASE + (x) ) = (y))

#define RTC_SECOND1         0
#define RTC_SECOND10        2
#define RTC_MINUTE1         4
#define RTC_MINUTE10        6
#define RTC_HOUR1           8
#define RTC_HOUR10          10
#define RTC_DAY1            12
#define RTC_DAY10           14
#define RTC_MONTH1          16
#define RTC_MONTH10         18
#define RTC_YEAR1           20
#define RTC_YEAR10          22
#define RTC_DOW             24 /* Day of week */
#define RTC_CONTROL_D       26 /* Control register D */
#define RTC_CONTROL_E       28 /* Control register E */
#define RTC_CONTROL_F       30 /* Control register F */



static void rtc_delay(unsigned long d)
{
    volatile unsigned long wait = d;
    while (wait--);
}

void rtc_wait_busy(void)
{
    RTC_WRITE(RTC_CONTROL_D, 1); // No interrupts, HOLD Bit=1

    uint8_t status = RTC_READ(RTC_CONTROL_D);

    while (status & 2)
    {
        RTC_WRITE(RTC_CONTROL_D, 0); // No interrupts, HOLD Bit=0
        rtc_delay(10);
        RTC_WRITE(RTC_CONTROL_D, 1); // No interrupts, HOLD Bit=1
        status = RTC_READ(RTC_CONTROL_D);
    }
    RTC_WRITE(RTC_CONTROL_D, 0);
}

uint8_t read_rtc_register(uint32_t reg)
{
    RTC_WRITE(RTC_CONTROL_D, 1); // No interrupts, HOLD Bit=1
    uint8_t status = RTC_READ(RTC_CONTROL_D);
    uint8_t data = 0;

    while (status & 2)
    {
        RTC_WRITE(RTC_CONTROL_D, 0); // No interrupts, HOLD Bit=0
        rtc_delay(10);
        RTC_WRITE(RTC_CONTROL_D, 1); // No interrupts, HOLD Bit=1
        status = RTC_READ(RTC_CONTROL_D);
    }
    data = RTC_READ(reg);
    RTC_WRITE(RTC_CONTROL_D, 0);

    return data;
}

int ddraig68k_hwclk(int set, struct rtc_time *t)
{
	if (!set){
        uint8_t hr, min, sec, day, mon;
        uint16_t yr;

        // get current date and time from RTC
        hr   =  read_rtc_register(RTC_HOUR1) & 0x0F;
        hr  += (read_rtc_register(RTC_HOUR10) & 0x0F) * 10;
        min  =  read_rtc_register(RTC_MINUTE1) & 0x0F;
        min += (read_rtc_register(RTC_MINUTE10) & 0x0F) * 10;
        sec  =  read_rtc_register(RTC_SECOND1) & 0x0F;
        sec += (read_rtc_register(RTC_SECOND10) & 0x0F) * 10;
        day  =  read_rtc_register(RTC_DAY1) & 0x0F;
        day += (read_rtc_register(RTC_DAY10) & 0x0F) * 10;
        mon  =  read_rtc_register(RTC_MONTH1) & 0x0F;
        mon += (read_rtc_register(RTC_MONTH10) & 0x0F) * 10;
        yr   =  read_rtc_register(RTC_YEAR1) & 0x0F;
        yr  += (read_rtc_register(RTC_YEAR10) & 0x0F) * 10;

        if (yr > 70)
            yr += 2000;
        else
            yr += 1900;

		t->tm_year = yr;
        t->tm_mon = mon;
        t->tm_mday = day;
		t->tm_hour = hr;
		t->tm_min = min;
		t->tm_sec = sec;
	}

	return 0;    
}

/***************************************************************************/

void ddraig68k_reset(void)
{
  local_irq_disable();
  asm volatile (
    "move.l 0xf7f800, %a0;\n"
    "move.l 0xf7f804, %sp;\n"
    "jmp (%a0);\n"
    );
}

static volatile uint8_t *uart_base = (volatile uint8_t *)0x00F7F000;

#define DUART_GET(x)		(uart_base[(x)])
#define DUART_PUT(x, y)		uart_base[(x)] = (y)

#define	DUART_MR1A      0x00     // Mode Register A
#define DUART_MR2A      0x00     // Mode Register A
#define DUART_SRA       0x02     // Status Register A
#define DUART_CSRA      0x02     // Clock-Select Register A
#define DUART_CRA       0x04     // Command Register A
#define DUART_RBA       0x06     // Receive Buffer A
#define DUART_TBA       0x06     // Transmit Buffer A
#define DUART_IPCR      0x08     // Input Port Change Register
#define DUART_ACR       0x08     // Auxiliary Control Register
#define DUART_ISR       0x0A     // Interrupt Status Register
#define DUART_IMR       0x0A     // Interrupt Mask Register
#define DUART_CUR       0x0C     // Counter Mode: current MSB
#define DUART_CTUR      0x0C     // Counter/Timer upper reg
#define DUART_CLR       0x0E     // Counter Mode: current LSB
#define DUART_CTLR      0x0E     // Counter/Timer lower reg
#define DUART_MR1B      0x10     // Mode Register B
#define DUART_MR2B      0x10     // Mode Register B
#define DUART_SRB       0x12     // Status Register B
#define DUART_CSRB      0x12     // Clock-Select Register B
#define DUART_CRB       0x14     // Command Register B
#define DUART_RBB       0x16     // Receive Buffer B
#define DUART_TBB       0x16     // Transmit Buffer A
#define DUART_IVR       0x18     // Interrupt Vector Register
#define DUART_IP        0x1A     // Input Port
#define DUART_OPCR      0x1A     // Output Port Configuration Reg.
#define DUART_STRTCC    0x1C     // Start-Counter command
#define DUART_OPRSET    0x1C     // Output Port Reg,SET bits
#define DUART_STOPCC    0x1E     // Stop-Counter command
#define DUART_OPRRST    0x1E     // Output Port Reg,ReSeT bits

static irqreturn_t ddraig68k_tick(int irq, void *dummy)
{

	uint8_t r = DUART_GET(DUART_ISR);
	static uint8_t c = 0x12;
	static uint8_t ct;
	timer_interrupt(irq, dummy);

	if (r & 0x08) {
		/* Ack the interrupt */
		DUART_GET(DUART_STOPCC);
		if (++ct == 20) {
			ct = 0;
			DUART_PUT(DUART_OPRSET, 0xFF);
			c <<= 1;
			c &= 0x7F;
			if (c == 0x10)
				c |= 0x02;
			DUART_PUT(DUART_OPRRST, c);
		}
	}

    return IRQ_HANDLED;
}


static struct irqaction ddraig_timer_irq = {
	.name	 = "timer",
	.flags	 = IRQF_TIMER,
	.handler = ddraig68k_tick,
};

void ddraig68k_timer_init(irq_handler_t handler)
{
    timer_interrupt = handler;

	setup_irq(4, &ddraig_timer_irq);

    DUART_PUT(DUART_CTLR, 0x80);
    DUART_PUT(DUART_CTUR, 0x04);
    DUART_PUT(DUART_IMR, 0x2A);
}

/***************************************************************************/

static int errno;

void __init config_BSP(char *command, int len)
{
  unsigned char *p;

  printk(KERN_INFO "\nY Ddraig 68000 support (C) 2022 Stephen Moody\n");

  mach_sched_init = ddraig68k_timer_init;
  mach_hwclk = ddraig68k_hwclk;
  mach_reset = ddraig68k_reset;
}

/***************************************************************************/
