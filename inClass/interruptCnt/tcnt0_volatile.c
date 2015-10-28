// Joshua Reed 
// Oregon State University
// tcnt0_volatile.c
// setup TCNT0 in compare mode and create counting pattern on port B
// count update is every (32668)*(128)*(256) = 0.999975168 sec

#include <avr/io.h>
#include <avr/interrupt.h>

// Declare 8-bit variable that is outside scope of main
volatile uint8_t tick;

//***********************************************************************
//                     ISR for timer counter zero
//***********************************************************************
ISR(TIMER0_COMP_vect) { tick++; }

//***********************************************************************
//                           init_tcnt0
// Initalize timer/counter zero to CTC mode
//***********************************************************************
void init_tcnt0()
{
    sei(); // Enable global interrupts
    // asynchronous status register
    ASSR  |=  (1<<AS0); // Run off external 32khz osc (TOSC)

    // timer mask
    TIMSK |=  (1<<OCIE0); // Enable interrupts for output compare match 0
 
    // Timer Counter Control Register
    // CTC--clear timer on compare match
    TCCR0 |=  (1<<WGM01) | (1<<CS00); // CTC mode, no prescale 

    OCR0  |=  0x07f; // Compare at 128
}

//***********************************************************************
//                              main
//***********************************************************************
int main() 
{
    DDRB = 0xFF; // Set all port B pins to output
    init_tcnt0(); // Initalize timer counter zero
    while(1)
    {
        if (tick == 255) 
        { 
              PORTB++;
              tick = 0; // PORTB will increment until rollover if not performed manually.
        }
    }
}


