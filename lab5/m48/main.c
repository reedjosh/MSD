#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include "uart_functions_m48.h"

int main(void)
{
    uart_init();
    int i = 0;
    char stuff [40] = {"Hello World!"};
    while(1)
    { 
        if (uart_getc() == 'a') 
        {
            for (i=0; i<12; i++)
            {
                uart_putc(stuff[i]);
            }
        }
    }
}
