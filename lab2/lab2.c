// OregonState EECS
// Microcontroller System Design
// lab2_code.c 
// Joshua Reed
// Sep. 29, 2015

// This program represents S0 button presses as a BCD counter on the
// PORTB LEDs  
#include <avr/io.h>
#include <util/delay.h>

//*******************************************************************************
//                            debounce_switch                                  
// Check pushbuttons on PORTA. 
// Returns which button was pressed 0-7.
//*******************************************************************************
uint8_t debounce_switch() 
{
    // Debounce 8ms  
    _delay_ms(2); 
    PORTA = 0xFF; // Display Off
    DDRA = 0x00; // DDRA to input
    PORTB = (7<<4); // Enable tristate for button board usage
    // Button press shift register
    static uint16_t SR[8] = {0,0,0,0,0,0,0,0}; 
    uint8_t i = 0;
    uint8_t ret_val = 9;
    for (i=0; i<8; i++) 
    {
        // bit_is_clear() returns a one when button pushed
        SR[i] = (SR[i] << 1) | bit_is_clear(PINA, i);
        if (SR[i] == 0x000F) { ret_val = i; }
    }
    DDRA = 0xFF; // set PORTA for output
    return ret_val;
}

//*******************************************************************************
//                           to_digs 
// Returns an array pointer
// The array is a digit wise separation of numbers 0 to 3.
// For example, passing num as 1234 will result in digs[0] = 4, digs[1] = 2, etc...
//*******************************************************************************
uint8_t * to_digs(uint16_t num) 
{
    static uint8_t digs [4];
    static uint8_t sev_seg[11] = { 0b11000000,   // 0
                            0b11111001,   // 1
                            0b10100100,   // 2
                            0b10110000,   // 3
                            0b10011001,   // 4
                            0b10010010,   // 5
                            0b10000010,   // 6
                            0b11111000,   // 7
                            0b10000000,   // 8
                            0b10010000,   // 9
                            0b11111111 }; // off
    digs[0] = sev_seg[num       % 10];
    digs[1] = sev_seg[(num/10)  % 10];
    digs[2] = sev_seg[(num/100) % 10];
    digs[3] = sev_seg[(num/1000)     ];
    return digs;
}

//*******************************************************************************
//                          set_disp 
// Set which digit is displayed. 
//*******************************************************************************
void set_disp(uint8_t disp)
{
    static uint8_t decode[5] = { 0,   // 000 disp 0
                                 1,   // 001 disp 1
                                 3,   // 011 colon 
                                 4,   // 100 disp 2
                                 2 }; // 101 disp 3
    PORTB = (decode[disp]<<4); 
}

//*******************************************************************************
//                            main 
// Check active low switches on PORTB.  
// If low for 4 passes of debounc_switch() increment counter.
// Display number on all four digits of the LED display board.
//*******************************************************************************
int main()
{
    DDRB = 0xFF; // Set to output
    DDRA = 0xFF; // Set to output

    PORTA = 0xFF; // Turn of LEDs

    uint16_t cnt = 0;
    uint8_t state = 0;
    while(1) // loop forever
    { 
        // rotate state
        state++;
        state %= 4;
        
        // display one digit per cycle
        set_disp(state);
        PORTA = to_digs(cnt)[state];
    
        // check button board
        if (debounce_switch()<8)   { cnt++; }
    }  
} 






