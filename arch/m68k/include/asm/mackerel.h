#ifndef _MACKEREL_H
#define _MACKEREL_H

#define MEM(address) (*(volatile unsigned char *)(address))

#ifdef CONFIG_MACKEREL08
#define MACKEREL_BOARD_NAME "Mackerel-08"
#define IRQ_NUM_DUART 1		// DUART on IPL1
#define DUART1_BASE 0x3FC000

#elif defined(CONFIG_MACKEREL10)
#define MACKEREL_BOARD_NAME "Mackerel-10"
#define IRQ_NUM_IDE   3
#define IRQ_NUM_DUART 5
#define IRQ_NUM_TIMER 6
#define DUART1_BASE 0xFF8000
#define IDE_BASE     0xFFC000	// command block (CS0)
#define IDE_CTL_BASE 0xFF400C	// control block (CS1): alt status / device control

#elif defined(CONFIG_DDRAIG68K)
#define MACKEREL_BOARD_NAME "Y Ddraig"
#define IRQ_NUM_IDE   2		// MC68230 PIT raises IPL3, IDE raises IPL2
#define IRQ_NUM_TIMER 3
#define IRQ_NUM_DUART 4
#define DUART1_BASE 0xF7F000
#define PIT_BASE    0xF7F100
#define IDE_BASE    0xF7F300
#define IDE_CTL_BASE (IDE_BASE + 0x8C)	// alt status / device control (CS1)

// MC68230 PIT register offsets (byte stride 2 on 16-bit bus)
#define PIT_PGCR  0x00		// Port General Control Register
#define PIT_TCR   0x20		// Timer Control Register
#define PIT_TIVR  0x22		// Timer Interrupt Vector Register
#define PIT_CPRH  0x26		// Counter Preload Register (high byte)
#define PIT_CPRM  0x28		// Counter Preload Register (mid byte)
#define PIT_CPRL  0x2A		// Counter Preload Register (low byte)
#define PIT_TSR   0x34		// Timer Status Register (write 0x01 to clear ZDS)
#define PIT_WRITE(reg, val) (MEM(PIT_BASE + (reg)) = (val))

// System clock 10 MHz, /32 prescaler -> reload = CLK/32/HZ
#define PIT_CLK_HZ    10000000UL
#define PIT_PRESCALER 32UL

#elif defined(CONFIG_MACKERELF)
#define MACKEREL_BOARD_NAME "Mackerel-F"
#define GPIO_BASE  0xFFF800
#define UART_BASE  0xFFF900
#define TIMER_BASE 0xFFFA00
#define SPI_BASE   0xFFFB00
#define SPI2_BASE  0xFFFC00
#define INTC_BASE  0xFFFD00	// per-level IRQ enable/mask register
#define IRQ_NUM_NIC   4
#define IRQ_NUM_UART  5
#define IRQ_NUM_TIMER 6

// Programmable Timer Registers
#define TIMER_CTRL   (TIMER_BASE + 0)
#define TIMER_STATUS (TIMER_BASE + 2)
#define TIMER_ENABLE_10HZ  (0x01 | (0 << 4))
#define TIMER_ENABLE_25HZ  (0x01 | (1 << 4))
#define TIMER_ENABLE_50HZ  (0x01 | (2 << 4))
#define TIMER_ENABLE_100HZ (0x01 | (3 << 4))

// 16550 Registers (helpers for the boot console only)
#define MF_UART_THR (UART_BASE + 0)
#define MF_UART_RBR (UART_BASE + 0)
#define MF_UART_DLL (UART_BASE + 0)
#define MF_UART_IER (UART_BASE + 2)
#define MF_UART_DLM (UART_BASE + 2)
#define MF_UART_IIR (UART_BASE + 4)
#define MF_UART_FCR (UART_BASE + 4)
#define MF_UART_LCR (UART_BASE + 6)
#define MF_UART_LSR (UART_BASE + 10)
#define MF_LSR_DR   0x01
#define MF_LSR_THRE 0x20

void uart16550_putc(char c);
char uart16550_getc(void);

#else   // Mackerel-30
#define MACKEREL_BOARD_NAME "Mackerel-30"
#define IRQ_NUM_DUART 5
// Mackerel-30 Timer
#define TIMER_BASE   0xF0030000
#define TIMER_ENABLE  (TIMER_BASE + 0x00)
#define TIMER_DISABLE (TIMER_BASE + 0x01)
#define DUART1_BASE 0xF0000000
#define IDE_BASE     0xF0010000
#define IDE_CTL_BASE 0xF0020000
#endif

#ifndef CONFIG_MACKERELF
/*
 * XR68C681/SCC68681 DUART register offsets.
 * The chip has 16 internal registers on a 4-bit address bus, stride-2 byte
 * access on a 16-bit data bus. Which byte lane depends on A0 wiring:
 *   Y Ddraig:       DUART A0 = CPU A1 → even byte offsets (DUART_BYTE_OFF = 0)
 *   Mackerel-10/08: DUART A0 = CPU A0 → odd byte offsets  (DUART_BYTE_OFF = 1)
 */
#ifdef CONFIG_DDRAIG68K
#define DUART_BYTE_OFF            0
#define DUART_CONSOLE_BAUD        38400
#define DUART_CONSOLE_BAUD_CFLAG  B38400
#define DUART_CONSOLE_IS_CHAN_A   1  /* Y Ddraig: console on Channel A (matches DdraigOS) */
#else
#define DUART_BYTE_OFF            1
#define DUART_CONSOLE_BAUD        115200
#define DUART_CONSOLE_BAUD_CFLAG  B115200
/* Mackerel-10/08: console on Channel B */
#endif

/* Register offsets from DUART base (internal address × 2 + DUART_BYTE_OFF) */
#define DUART_OFF_MR_A      (0x00 + DUART_BYTE_OFF)  /* MR1A / MR2A (auto-advance on write) */
#define DUART_OFF_SR_A      (0x02 + DUART_BYTE_OFF)  /* SRA (r) / CSRA (w) */
#define DUART_OFF_CR_A      (0x04 + DUART_BYTE_OFF)  /* MISR (r) / CRA (w) / BRGTEST (r, SCC68681: read to enable extended baud table) */
#define DUART_OFF_RB_A      (0x06 + DUART_BYTE_OFF)  /* RBA (r) / TBA (w) */
#define DUART_OFF_ACR       (0x08 + DUART_BYTE_OFF)  /* IPCR (r) / ACR (w) */
#define DUART_OFF_IMR       (0x0A + DUART_BYTE_OFF)  /* ISR (r) / IMR (w) */
#define DUART_OFF_CTUR      (0x0C + DUART_BYTE_OFF)  /* CUR (r) / CTUR (w) */
#define DUART_OFF_CTLR      (0x0E + DUART_BYTE_OFF)  /* CLR (r) / CTLR (w) */
#define DUART_OFF_MR_B      (0x10 + DUART_BYTE_OFF)  /* MR1B / MR2B (auto-advance on write) */
#define DUART_OFF_SR_B      (0x12 + DUART_BYTE_OFF)  /* SRB (r) / CSRB (w) */
#define DUART_OFF_CR_B      (0x14 + DUART_BYTE_OFF)  /* CRB (w) */
#define DUART_OFF_RB_B      (0x16 + DUART_BYTE_OFF)  /* RBB (r) / TBB (w) */
#define DUART_OFF_IVR       (0x18 + DUART_BYTE_OFF)  /* IVR */
#define DUART_OFF_IP        (0x1A + DUART_BYTE_OFF)  /* IP (r) / OPCR (w) */
#define DUART_OFF_OPR_SET   (0x1C + DUART_BYTE_OFF)  /* start counter (r) / set OPR bits (w) */
#define DUART_OFF_OPR_RESET (0x1E + DUART_BYTE_OFF)  /* stop counter (r) / reset OPR bits (w) */

/* Absolute addresses derived from DUART1_BASE + offset */
#define DUART1_MR1A      (DUART1_BASE + DUART_OFF_MR_A)
#define DUART1_MR2A      (DUART1_BASE + DUART_OFF_MR_A)
#define DUART1_SRA       (DUART1_BASE + DUART_OFF_SR_A)
#define DUART1_CSRA      (DUART1_BASE + DUART_OFF_SR_A)
#define DUART1_CRA       (DUART1_BASE + DUART_OFF_CR_A)
#define DUART1_MISR      (DUART1_BASE + DUART_OFF_CR_A)
#define DUART1_BRGTEST   (DUART1_BASE + DUART_OFF_CR_A)  /* SCC68681: read to enable extended baud table */
#define DUART1_RBA       (DUART1_BASE + DUART_OFF_RB_A)
#define DUART1_TBA       (DUART1_BASE + DUART_OFF_RB_A)
#define DUART1_IPCR      (DUART1_BASE + DUART_OFF_ACR)
#define DUART1_ACR       (DUART1_BASE + DUART_OFF_ACR)
#define DUART1_ISR       (DUART1_BASE + DUART_OFF_IMR)
#define DUART1_IMR       (DUART1_BASE + DUART_OFF_IMR)
#define DUART1_MR1B      (DUART1_BASE + DUART_OFF_MR_B)
#define DUART1_MR2B      (DUART1_BASE + DUART_OFF_MR_B)
#define DUART1_SRB       (DUART1_BASE + DUART_OFF_SR_B)
#define DUART1_CSRB      (DUART1_BASE + DUART_OFF_SR_B)
#define DUART1_CRB       (DUART1_BASE + DUART_OFF_CR_B)
#define DUART1_RBB       (DUART1_BASE + DUART_OFF_RB_B)
#define DUART1_TBB       (DUART1_BASE + DUART_OFF_RB_B)
#define DUART1_IVR       (DUART1_BASE + DUART_OFF_IVR)
#define DUART1_IP        (DUART1_BASE + DUART_OFF_IP)
#define DUART1_OPCR      (DUART1_BASE + DUART_OFF_IP)
#define DUART1_OPR       (DUART1_BASE + DUART_OFF_OPR_SET)
#define DUART1_OPR_RESET (DUART1_BASE + DUART_OFF_OPR_RESET)

// Interrupt bits
#define DUART_INTR_COUNTER 0b00001000
#define DUART_INTR_RXRDY 0b00100000

// Mackerel-08 serial interrupts and timer interrupts both come from the same DUART pin
// so we need to preserve IMR and ACR register bits during the timer tick
#ifdef CONFIG_MACKEREL08
#define DUART_IMR_RESERVED DUART_INTR_COUNTER
#define DUART_ACR_RESERVED 0x70

// The other Mackere boards have separate timer and serial interrupt lines, so not important
#else
#define DUART_IMR_RESERVED 0
#define DUART_ACR_RESERVED 0
#endif

// DUART
void duart_init(void);
void duart_putc(char c);
void duart_puts(const char *s);
char duart_getc(void);
#endif /* !CONFIG_MACKERELF */

#endif /* _MACKEREL_H */
