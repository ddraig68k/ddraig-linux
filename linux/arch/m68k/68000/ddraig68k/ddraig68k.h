#ifndef _DDRAIG68K_H
#define _DDRAIG68K_H

// Ddraig hardware addresses
#define SRAM_START      0x00000000       // Start of Static RAM
#define SRAM_END        0x000FFFFF       // End of Static RAM
#define DRAM_START      0x00100000       // Start of Dynamic RAM
#define DRAM_END        0x008FFFFF       // End of Dynamic RAM
#define HEAP_START      0x00020000       // Heap memory start
#define HEAP_END        0x008FFFFF       // Heap memory end

#define DUART_BASE      0x00F7F000       // Base address of the Dual Asynchronous Receiver Transmitter
#define PIT_BASE        0x00F7F100       // Base address of the Peripheral Interface Timer
#define PS2_BASE        0x00F7F200       // Base address of the VT82C42 Keyboard/Mouse controller
#define IDE_BASE        0x00F7F300       // Base address of the IDE controller
#define RTC_BASE        0x00F7F400       // Base address of the RTC72421 clock controller
#define EXP1_BASE       0x00F7F500       // Base address of the Expansion slot ID
#define EXP2_BASE       0x00F7F600       // Base address of the Expansion slot ID
#define EXP3_BASE       0x00F7F700       // Base address of the Expansion slot ID
#define EXP4_BASE       0x00F7F800       // Base address of the Expansion slot ID

#define EXP1_DATA       0x00A00000       // Base address of the Expansion slot data
#define EXP2_DATA       0x00B00000       // Base address of the Expansion slot data
#define EXP3_DATA       0x00C00000       // Base address of the Expansion slot data
#define EXP4_DATA       0x00D00000       // Base address of the Expansion slot data

#define ROM_START       0x00F80000       // Start of ROM
#define ROM_END         0x00FFFFFF       // End of ROM

#define STACK_SIZE      0x8000           // 32K for the stack

#define ISR_VECT_EXP1(vec)      (*((long *)0x64) = (long)vec)
#define ISR_VECT_IDE(vec)       (*((long *)0x68) = (long)vec)
#define ISR_VECT_PIT(vec)       (*((long *)0x6C) = (long)vec)
#define ISR_VECT_DUART(vec)     (*((long *)0x70) = (long)vec)
#define ISR_VECT_PS2(vec)       (*((long *)0x74) = (long)vec)
#define ISR_VECT_EXP2(vec)      (*((long *)0x78) = (long)vec)
#define ISR_VECT_EXP3(vec)      (*((long *)0x7C) = (long)vec)

#define DUART_MR1A          (DUART_BASE + 0x00)
#define DUART_MR2A          (DUART_BASE + 0x00)
#define DUART_SRA           (DUART_BASE + 0x02)
#define DUART_CSRA          (DUART_BASE + 0x02)
#define DUART_CRA           (DUART_BASE + 0x04)
#define DUART_RBA           (DUART_BASE + 0x06)
#define DUART_TBA           (DUART_BASE + 0x06)
#define DUART_ACR           (DUART_BASE + 0x08)
#define DUART_ISR           (DUART_BASE + 0x0A)
#define DUART_IMR           (DUART_BASE + 0x0A)
#define DUART_CUR           (DUART_BASE + 0x0C)
#define DUART_CLR           (DUART_BASE + 0x0E)

#define DUART_MR1B          (DUART_BASE + 0x10)
#define DUART_MR2B          (DUART_BASE + 0x10)
#define DUART_SRB           (DUART_BASE + 0x12)
#define DUART_CSRB          (DUART_BASE + 0x12)
#define DUART_CRB           (DUART_BASE + 0x14)
#define DUART_RBB           (DUART_BASE + 0x16)
#define DUART_TBB           (DUART_BASE + 0x16)

#define DUART_IVR           (DUART_BASE + 0x18)
#define DUART_OPCR          (DUART_BASE + 0x1A)
#define DUART_OPR           (DUART_BASE + 0x1C)
#define DUART_OPR_RESET     (DUART_BASE + 0x1E)

// Get the value at a memory address
#define MEM(address) (*(volatile unsigned char *)(address))

void ddraig_putc(char c);
char ddraig_getc(void);

#endif
