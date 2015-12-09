#include "uart_functions_m48.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "../libs/lm73_functions.h"
#include "../libs/twi_master.h"

uint8_t send_str(char * arr);
void spi_init(void);

// TWI buffers
extern uint8_t lm73_wr_buf[2]; 
extern uint8_t lm73_rd_buf[2];

int main(void)
{
    spi_init();
    init_twi();
    uart_init();
    sei();
    lm73_wr_buf[0] = LM73_PTR_TEMP; // load lm73_wr_buf[0] with temperature pointer address
    twi_start_wr(LM73_WRITE, lm73_wr_buf, 1);   //start the TWI write process (twi_start_wr())
        
    uint16_t lm73_temp;
    char stuff [2];
    while(1)
    { 
        // listen for transmit request
        if(uart_getc() == 'a')
        {
            uart_putc(stuff[0]);
            uart_putc(stuff[1]);
        }
        twi_start_rd(LM73_READ, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes)  (twi_start_rd())
        _delay_us(50); // wait for it to finish
        lm73_temp = (lm73_rd_buf[0]<<8);   //save high temperature byte into lm73_temp
        lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
        lm73_temp = (lm73_temp>>7);
        itoa(lm73_temp, stuff, 10); //convert to string in array with itoa() from avr-libc
    }
}

void spi_init(void) 
{
    DDRB |=  0b11110111; 
    SPCR = (1<<SPE) | (1<<MSTR); 
    SPSR = (1<<SPI2X); // SPI at 2x speed (8 MHz)  
}
