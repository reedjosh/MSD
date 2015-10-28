//heartint.c
//setup TCNT1 in pwm mode, TCNT3 in normal mode 
//set OC1A (PB5) as pwm output 
//pwm frequency:  (16,000,000)/(1 * (61440 + 1)) = 260hz
//
//Timer TCNT3 is set to interrupt the processor at a rate of 30 times a second.
//When the interrupt occurs, the ISR for TCNTR3 changes the duty cycle of timer 
//TCNT1 to affect the brightness of the LED connected to pin PORTB bit 5.
//
//
// Joshua Reed
// Keaton Sheible
// Oliver Poei
//
////////////////////////////////
#include <avr/io.h>
#include <avr/interrupt.h>

ISR( TIMER3_OVF_vect ) {                                     
    static uint16_t brightness[20] = {0, 10, 50, 70, 80, 100, 500, 1000, 10000, 60000, 60000, 10000, 1000, 500, 100, 80, 70, 50, 10, 0};
    static uint8_t temp = 0; 
    OCR1A=brightness[temp%20]; // rotate through brightness levels
    temp++; // increment to overflow
    }

int main() {

//setup timer counter 1 as pwm source 

    // set port B bit five to output
    DDRB = 0b00100000;                  

    // fast pwm, set on match, clear@bottom, 
    // (inverting mode) ICR1 holds TOP          
    TCCR1A |=  (1<<COM1A1) | (1<<WGM11); 
                                         

    // use ICR1 as source for TOP, use clk/1
    TCCR1B |=  (1<<WGM13) | (1<<WGM12) | (1<<CS10); 
    
    // no forced compare 
    TCCR1C  = 0b00000000;                 

    // clear at 0xF000
    ICR1    = 0b11110000;                                                
  
// set timer counter 3 as interrupt source
// 30 interrupts/sec--(16,000,000)/(8 * 2^16) = 30 cycles/sec
  
  TCCR3A = 0b00000000; // normal mode

  TCCR3B |= (1<<CS11); // use clk/8 (15hz)

  TCCR3C = 0b00000000; // no forced compare 

  ETIMSK = (1<<TOIE3); // enable timer 3 interrupt on TOV
  sei(); // set GIE to enable interrupts

// loop forever
    while(1) {  
        } 
    } 












