#include "ddraig68k.h"

void ddraig_putc(char c)
{
    while ((MEM(DUART_SRB) & 0b00000100) == 0)
    {
    }

    MEM(DUART_TBB) = c;

    // Always print a carriage return after a line feed
    if (c == 0x0A)
    {
        ddraig_putc(0x0D);
    }
}

char ddraig_getc(void)
{
    while ((MEM(DUART_SRB) & 0b00000001) == 0)
    {
    }

    return MEM(DUART_RBB);
}
