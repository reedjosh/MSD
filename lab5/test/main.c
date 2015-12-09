
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>
#include "uart_functions.h"
#include "lcd_functions.h"

void spi_init(void){
  DDRB |=  0x07;  //Turn on SS, MOSI, SCLK
  //mstr mode, sck=clk/2, cycle 1/2 phase, low polarity, MSB 1st, 
  //no interrupts, enable SPI, clk low initially, rising edge sample
  SPCR=(1<<SPE) | (1<<MSTR); 
  SPSR=(1<<SPI2X); //SPI at 2x speed (8 MHz)  
 }//spi_init

int main(void)
{
    spi_init();
    lcd_init();
    uart_init();
    char stuff[40];
    //cursor_off();
    int i = 0;
    while(1)
    { 
        uart_putc('a');
        for(i=0; i<12; i++)
        {
            stuff[i] = uart_getc();
        }
        for(i=0; i<12; i++)
        {
            char2lcd(stuff[i]);
        }
        cursor_home();
        _delay_ms(200);
    }
}
