// bar_graph_demo_skel.c 
// R. Traylor
// 10.22.14
// demos interrups, counter timer and SPI

// !!! YOU MUST RENAME THIS FILE TO MATCH YOUR MAKEFILE   !!!
// !!! YOU MUST FILL IN THE AREAS MARKED WITH "@" SIGNS   !!!
// !!! DISCONNECT ALL OTHER CONNECTIONS TO YOUR AVR BOARD !!!

// This code implements a timer interrupt to update the bar graph display
// at 0.5 second intervals. Every half second, a new data value is sent to 
// the bargraph via SPI. The value is displayed as single led in a climbing
// pattern.

// Expected Connections:
// Bargraph board           Mega128 board 
// --------------      ----------------------    
//     reglck            PORTB bit 0 (ss_n)                      
//     srclk             PORTB bit 1 (sclk)
//     sdin              PORTB bit 2 (mosi)
//     oe_n                   ground
//     gnd2                   ground
//     vdd2                     vcc
//     sd_out               no connect

#include <avr/io.h>
#include <avr/interrupt.h>

/***********************************************************************/
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further   
//external device specific initalizations.  Sets up SPI to be:                        
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void)
{
    // Turn on slave select, master out slave in, serial clk
    DDRB  |=   0x07;          

    // SPI enabled, master mode, low polarity, MSB 1st, interrupts disabled, clk/2
    SPCR |= (1<<SPE) | (1<<MSTR); //spi control register
    
    // Run at clk/2
    SPSR = (1<<SPI2X); //spi status register
}



/***********************************************************************/
//                              tcnt0_init                             
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
//
void tcnt0_init(void)
{
    // Set to external ocillator
    ASSR   |=  (1 << AS0); // Asynchronous status register 

    // Enable Timer/Counter0 overflow interrupt
    TIMSK  |=  (1 << TOIE0); // Timer mask 

    // Normal mode, No prescale
    TCCR0  |=  (1 << CS00); // Timer counter control register
}

/*************************************************************************/
//                           timer/counter0 ISR                          
//When the TCNT0 overflow interrupt occurs, the count_7ms variable is    
//incremented.  Every 7680 interrupts the minutes counter is incremented.
//tcnt0 interrupts come at 7.8125ms internals.
// 1/32768         = 30.517578uS
//(1/32768)*256    = 7.8125ms
//(1/32768)*256*64 = 500mS
/*************************************************************************/
ISR(TIMER0_OVF_vect) // using the timer overflow indicator
{ 
    // Holds 7ms tick count in binary
    static uint8_t count_7ms = 0;        
    
    // Holds count for display 
    static uint8_t display_count = 0x01; 
   
    // Increment count every 7.8125 ms
    count_7ms++;                 
    

    // Interrupts equals one half second 
    if ((count_7ms % 64)==0){         
        
        // Send to display
        SPDR = display_count;               
        
        // Wait until 8 clock cycles are done
        while (bit_is_clear(SPSR,SPIF)){} // SPI Status register // SPI flag
        
        // Strobe output data reg in HC595
        PORTB |=  0x01;          
        
        // Falling edge
        PORTB &=  0x00; 

        // Shift display bit for next time
        display_count = (display_count << 1);  
    }

    // Display back to first position
    if (display_count == 0x00){display_count=0x01;} 
}

/***********************************************************************/
//                                main                                 
/***********************************************************************/
int main()
{     
    tcnt0_init();  //initalize counter timer zero
    spi_init();    //initalize SPI port
    sei();         //enable interrupts before entering loop
    while(1){}     //empty main while loop

} //main


















