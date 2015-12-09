//UART Functions for Mega 48 only
//Roger Traylor 12.7.15
//For controlling the UART and sending debug data to a terminal as an aid 
//in debugging. Note that RX and TX lines are marked relative to the device
//they are located on.

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>

#define USART_BAUDRATE 4800  
#define BAUDVALUE  ((F_CPU/(USART_BAUDRATE * 16UL)) - 1 )


//******************************************************************
//                        uart_putc
//
// Takes a character and sends it to USART0
//******************************************************************
void uart_putc(char data) 
{
    while (!(UCSR0A&(1<<UDRE0))); // Wait for previous transmissions
    UDR0 = data; // Send data byte
}

//******************************************************************
//                        uart_puts
// Takes a string and sends each charater to be sent to USART0
//******************************************************************
void uart_puts(char *str) 
{
    int i = 0;               
    // Loop through string, sending each character
    while(str[i] != '\0') 
    { 
        uart_putc(str[i]);
        i++;
    }
    uart_putc('\0');
}

//******************************************************************
//                            uart_init
//
//RXD is PORT D bit 0
//TXD is PORT D bit 1
//******************************************************************
void uart_init(){
//rx and tx enable, receive interrupt enabled, 8 bit characters
//UCSR0B |= (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0); //INTERRUPTS ENABLED
  UCSR0B |= (1<<RXEN0) | (1<<TXEN0);               //INTERRUPS DISABLED

//async operation, no parity,  one stop bit, 8-bit characters
  UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
  UBRR0H = (BAUDVALUE >>8 ); //load upper byte of the baud rate into UBRR 
  UBRR0L =  BAUDVALUE;       //load lower byte of the baud rate into UBRR 

}


//******************************************************************
//                             uart_getc
//Modified to not block indefinately in the case of a lost byte
//******************************************************************
char uart_getc(void) 
{
    uint16_t timer = 0;

    while (!(UCSR0A & (1<<RXC0))) 
    {
        timer++;
        if(timer >= 160000){ return(0);}
    } 

    return(UDR0); //return the received data
}

