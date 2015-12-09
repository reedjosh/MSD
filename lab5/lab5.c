// OregonState EECS
// Microcontroller System Design
// lab4_code.c 
// Joshua Reed
// Dec. 8, 2015
//
// Pin Setup
// ------------------------------------------------------------ 
// Mega128                                 
// ------------------------------------------------------------ 
//                          Sev Seg             Button Board
// ------------------------------------------------------------ 
// PORTA(0)                 A
// PORTA(1)                 B
// PORTA(2)                 C
// PORTA(3)                 D
// PORTA(4)                 E
// PORTA(5)                 F
// PORTA(6)                 G
// PORTA(7)                 DP
//
// PORTB(4)                 SEL0
// PORTB(5)                 SEL1
// PORTB(6)                 SEL2
// PORTB(7)                 PWM
//
//                          DEC7                COM_EN
// Vdd                      EN, Vdd             Vdd             
// GND                      EN_N, GND           GND
//                          A_2-DP_2            S1-S8
// ------------------------------------------------------------ 
//                          Bar Graph             
// ------------------------------------------------------------ 
// PORTB(0)                 REGCLK
// PORTB(1)                 SCLK
// PORTB(2)                 DIN
//

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "libs/lcd_functions.h"
#include "libs/lm73_functions.h"
#include "libs/twi_master.h"
#include "libs/uart_functions.h"

// Global status variables
uint8_t flag = 0;
uint8_t half_sec = 0;
uint8_t sec = 0;
uint8_t quart_sec = 0;
uint8_t mode = 0;
uint8_t alarm_set = 0;
uint8_t alarm_trg = 0;
uint8_t snooze_timer = 0;
uint8_t lcd_lock = 0;

// Seven segment decoder array
volatile uint8_t sev_seg[11] = { 0b11000000,   // 0 // Seven segment decoder array
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

// Time struct {hour, minute, second}
typedef struct 
{
    int8_t hr;
    int8_t min;
    int8_t sec; 
} time;

// Global time variables
time zero = {0,0,0};
time min_plus = {0,1,0};
time min_less = {0,-1,0};
time hr_plus = {1,0,0};
time hr_less = {-1,0,0};
time curr_time  = {23,59,56}; 
time alarm_time = {23,59,56};

int8_t _enc();
int8_t comp_times(time a, time b);
uint8_t check_buttons();
uint8_t spi_cycle(uint8_t write_val) ;
void adc_init();
void play_alarm();
void stop_alarm();
void disp_time();
void tcntr_setup();
void spi_setup();
time inc_time(time a);
time add_times(time a, time b);


volatile uint8_t time_cnt = 0;
//***********************************************************************
//                                                          ISR0
//
//***********************************************************************
ISR(TIMER0_OVF_vect) 
{ 
    time_cnt++;
    if ((time_cnt%16) == 0) { sec = !sec; }
    if ((time_cnt%8) == 0) 
    { 
        half_sec = !half_sec; 
    }
    if ((time_cnt%4) == 0) { quart_sec = !quart_sec && half_sec; }
        if (half_sec && alarm_trg && alarm_set) { play_alarm(); }
        else { stop_alarm(); }
}

//***********************************************************************
//                                                          ISR1
//
//***********************************************************************
ISR(TIMER1_COMPA_vect) 
{
    PORTD ^= 1<<PD7;
}

volatile uint8_t cnt2 = 0;
volatile uint8_t t2 = 0;
volatile uint16_t volume = 128;
//***********************************************************************
//                                                          ISR2
//
//***********************************************************************
ISR(TIMER2_OVF_vect) 
{
    cnt2++;
    if (((cnt2 % 10) == 0) && (lcd_lock == 0)) 
    {   
        t2 = check_buttons();

        if ((mode == 1) && (t2 == 0)) { mode = 0; }
        else if (t2 == 0) { mode = 1; }

        if ((mode == 2) && (t2 == 1)) { mode = 0; }
        else if (t2 == 1) { mode = 2; }
        
        if (t2 == 5) { 
            snooze_timer = 10;
            stop_alarm(); }
        if (t2 == 7) { alarm_set = !alarm_set; }
        if (!alarm_set) { alarm_trg = 0; }
         
    
        int8_t temp = _enc();
        if (mode == 1) 
        {
            if (temp ==  1) {curr_time = add_times(curr_time, hr_plus);}
            if (temp == -1) { curr_time = add_times(curr_time, hr_less);}
            if (temp ==  2) { curr_time = add_times(curr_time, min_plus);}
            if (temp == -2) { curr_time = add_times(curr_time, min_less);}
        }
        else if (mode == 2) 
        {
            if (temp ==  1) {alarm_time = add_times(alarm_time, hr_plus);}
            if (temp == -1) {alarm_time = add_times(alarm_time, hr_less);}
            if (temp ==  2) {alarm_time = add_times(alarm_time, min_plus);}
            if (temp == -2) {alarm_time = add_times(alarm_time, min_less);}
        }
        else
        {
            if (temp ==  2) { volume+=4;}
            if (temp == -2) { volume-=4;}
            if (volume < 31) {volume = 31;}
            if (volume > 255) {volume = 255;}
        }
    }
}

//***********************************************************************
//                                                          ISRADC
//
//***********************************************************************
ISR(ADC_vect) 
{
    int16_t temp = 0;
    temp = 255-2*ADCH; 
    if (temp < 30) {temp = 30;}
    OCR2 = temp;
}

//*******************************************************************************
//                                                          main 
//*******************************************************************************
int main()
{
    uint8_t sec_trg = 0;
    uint8_t al_set_trg = 0;
    DDRB = 0b11110111; // Set to output accept input on spi pin 3
    DDRE = 0xFF; // Set to output
    DDRA = 0xFF; // Set to output
    DDRF |= 1<<PF3;
    tcntr_setup();
    spi_setup();
    adc_init();
    uart_init();
    sei();
    cursor_off();

    while(1) // Loop forever
    { 
        disp_time();
            
            if (sec_trg != sec) 
            {
                // Track the second clock for edges 
                sec_trg = sec;
                if (snooze_timer > 0) { snooze_timer--; }  
                lcd_lock = 1;
                char2lcd(uart_getc());
                lcd_lock = 0;
                // Increment the current time by one second
                curr_time = inc_time(curr_time);
                if (alarm_set && comp_times(curr_time, alarm_time)) { alarm_trg = 1; }
                if (alarm_set && alarm_trg) { OCR3A = volume; }
                else { OCR3A = 1; }
                if (al_set_trg != alarm_set)
                {   
                    lcd_lock = 1;
                    lcd_init();
                    clear_display();
                    al_set_trg = alarm_set;
                    //if (alarm_set) { char2lcd(uart_getc()); }
                    //if (alarm_set) { string2lcd( "Alarm Set"); }
                    //else { clear_display(); }
                    spi_setup();
                    lcd_lock = 0;
                }
            }

    }  
} 

//*******************************************************************************
//                                                          _enc 
//
// returns  0 if none of the encoders have been rotated
// returns  1 if left  encoder has been rotated clockwise
// returns  2 if right encoder has been rotated clockwise
// returns -1 if left  encoder has been rotated counter-clockwise
// returns -2 if right encoder has been rotated counter-clockwise
//*******************************************************************************
int8_t _enc()
{
    static uint8_t ec_state = 0; // state of encoders
    uint8_t temp = spi_cycle(volume);
    ec_state = (ec_state << 4); // state of encoders
    ec_state |= (0b00001111 & temp);
    uint8_t ec1_prev = (ec_state & 0b00110000)>>4;
    uint8_t ec1_curr =  ec_state & 0b00000011;
    uint8_t ec2_prev = (ec_state & 0b11000000)>>6;
    uint8_t ec2_curr = (ec_state & 0b00001100)>>2;
    int8_t state = 0;
    if (ec1_prev != ec1_curr) 
    {
        if ((ec1_prev == 0b11) && (ec1_curr == 0b01)) 
        { // C turn  
             state = 2; 
        }
        else if ((ec1_prev == 0b11) && (ec1_curr == 0b10)) 
        { // CC turn   
             state = -2; 
        }
    }
    if (ec2_prev != ec2_curr) 
    {
        if ((ec2_prev == 0b11) && (ec2_curr == 0b01)) 
        { // C turn  
            state = 1; 
        }
        else if ((ec2_prev == 0b11) && (ec2_curr == 0b10)) 
        { // CC turn 
             state = -1; 
        }
    }
    return state;
}

//*******************************************************************************
//                                                          check_buttons               
// Check pushbuttons on PORTA. 
// Returns which button was pressed 0-7 or 8 if none.
// Maintains its own state.
//*******************************************************************************
uint8_t check_buttons() 
{
    // Button press shift register
    static uint16_t SReg[8] = {0}; 
    // State Variable
    static uint8_t button = 0;

    uint8_t ret_val = 8;
    
    // DDRA to input
    DDRA = 0x00;
    PORTA = 0x00;
    // Enable button board 
    PORTB = (7<<4) | (PORTB & 0b10001111); 
    
    // bit_is_clear() returns one when button pressed
    // Shift in the current button state
    SReg[button] = (SReg[button] << 1) | bit_is_clear(PINA, button);
    // If the button has been pressed after a period of no presses 
    if (SReg[button] == 0x000F) 
    { // Change the return value to the button being pressed 
        ret_val = button; 
    }

    // Restore values
    DDRA  = 0xFF; 
    PORTB = PORTB & 0b11101111; 

    // Increment State (0-7)
    button++;
    button = button % 8;
    return ret_val;
}

//*******************************************************************************
//                                                          play_alarm
//*******************************************************************************
void play_alarm()
{
    if (snooze_timer == 0) {TCCR1B |= (1<<CS11) | (1<<CS10);}
}

//*******************************************************************************
//                                                          stop_alarm
//*******************************************************************************
void stop_alarm()
{
    TCCR1B &= ~((1<<CS11) | (1<<CS10));
}

//*******************************************************************************
//                                                          disp_time 
//
//*******************************************************************************
void disp_time() 
{
    uint8_t digit; // Digit place holder
    static uint8_t disp_state = 0;
    time t;
    if (mode == 2) { t = alarm_time; }
    else { t = curr_time; }
    // Parse and decode digits
    switch( disp_state )
    {
        case 0:
            digit = sev_seg[t.min % 10];
            break;
        case 1:
            digit = sev_seg[t.min / 10];
            break;
        case 2:
            if (half_sec) {digit = 0b11111100;}
            else          {digit = 0b11111111;}
            if (alarm_set) { digit &= 0b11111011; }
            break;
        case 3:
            digit = sev_seg[t.hr % 10];
            break;
        case 4:
            digit = sev_seg[t.hr / 10];
            break;
        default:
            digit = 0b11111111;
            break;
    }
    if (quart_sec && ((mode == 1) || (mode == 2))) {digit = 0b11111111;}
    PORTB = (disp_state<<4) | (PORTB & 0b10001111); // Sel Bits
    PORTA = digit;
    _delay_us(10); // delay with the leds on
    PORTA = 0xFF; // turn leds off
    PORTB = (6<<4) | (PORTB & 0b10001111); // Sel Bits
    disp_state++;
    disp_state %= 5;
}

//*******************************************************************************
//                                                          tcntr_setup
//
// TCNT0 - realtime clock, 32khz, seconds tick
// 
// TCNT1 - alarm sound
//       - slow 1khz
//
// TCNT2 - pwm source to dim LED's
//       - use fast pwm
//
// TCNT3 - pwm source for alarm volume
//*******************************************************************************
void tcntr_setup()
{
    // TCNT0 -- 1/2 second overflow 
    ASSR = 1<<AS0; // Use external oscillator
    TIMSK |= 1<<TOIE0; // Overflow Interrupt Enable
    TCCR0 |= (1<<CS01); // Prescale by 64

    //// TCNT1 - alarm 
    TIMSK |= 1<<OCIE1A; // Compare Match Interrupt Enable
    TCCR1A = 0;
    TCCR1B |= 1<<WGM12; // CTC
    OCR1A = 0x00FF; // Compare register
    DDRD |= 1<<PD7; // Set pin D7 for output

    // TCNT2 - LED dimming PWM
    TIMSK |= (1<<TOIE2); // Overflow Interrupt Enable
    OCR2 = 8; // Full Duty Cycle
    TCCR2 |= (1<<COM21) | (1<<COM20) | // Fast PWM
             (1<<WGM21) | (1<<WGM20) | // Compare Match
             (1<<CS21); // prescale 8 

    //// TCNT3 - AMP volume pwm
    OCR3A = 1; // Volume Muted
    TCCR3A = (1<<WGM30) | // Fast PWM
             (1<<COM3A1); // Compare Match 
    TCCR3B = (1<<WGM32) | 
             (1<<CS30); // Prescale by 1
    DDRE |= 1<<PE3;
}

//*******************************************************************************
//                                                          spi_cycle
//
// Writes input "write_val" to SPI out and returns input from read in.
//*******************************************************************************
uint8_t spi_cycle(uint8_t write_val) 
{
    PORTE &= 0b01111111; // load data
    PORTE |= 0b10000000; // set as shift reg and enable the clk 
    SPDR = write_val; // Send data
    while (bit_is_clear(SPSR, SPIF)){} // SPI sreg, sflag
    PORTB |= 0b00000001; // Strobe high
    PORTB &= 0b11111110; // Strobe low
    return SPDR;
}

//*******************************************************************************
//                                                          spi_setup
//
//*******************************************************************************
void spi_setup()
{
    SPCR |= (1 << SPE) | (1 << MSTR); // SPI ctr reg -- SPI enable, MSTR
    SPSR |= (1 << SPI2X); // SPI status reg -- set clk/2
}

//*******************************************************************************
//                                                          adc_init
// Pertinent Control Registers
// -------------------------------------------------------------------
// ADMUX (A/D Multiplexer Select Register)
// -------------------------------------------------------------------
// | REFS1 | REFS0 | ADLAR | MUX4 | MUX3 | MUX2 | MUX1 | MUX0 |
// -------------------------------------------------------------------
//      | REFS1 | REFS0 |      VRef Selection         
//      | ADLAR | ADC Result Left(1) Right(0) adjuxt
//      | MUX(4-0) | Input and Gain Selection
// ADCSRA (A/D Control and Status Register)
//*******************************************************************************
void adc_init()
{
    DDRF  &= 0b01111111;
    PORTF &= 0b01111111;
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) |
             (1<<ADFR) | (1<<ADIE)  | (1<<ADSC); // Enable ADC
    ADMUX = (1<<MUX2)  | (1<<MUX1)  | (1<<MUX0) | // Use PD7 
            (1<<ADLAR) | // Adjust Left
            (1<<REFS0); // Internal 2.56Vref
}

//*******************************************************************************
//                                                          comp_times
//
//*******************************************************************************
int8_t comp_times(time a, time b)
{
    int8_t ret_val = 1;
    if (a.min != b.min) {ret_val = 0;}
    if (a.hr != b.hr) {ret_val = 0;}
    return ret_val;
}

//*******************************************************************************
//                                                          inc_time
// Add a second to the current time
//*******************************************************************************
time inc_time(time a)
{
    time second = {0,0,1};
    return add_times(a, second);
}

//*******************************************************************************
//                                                          add_times 
//
// Because times are stored in 8bit signed integers (-128,127),
// two time digits (-60,60) added together can't cause rollover. 
//*******************************************************************************
time add_times(time a, time b)
{
    a.sec += b.sec;
    a.min += b.min;
    a.hr += b.hr;
    if (a.sec > 59) 
    {
        a.min++;
        a.sec-=60;
    }
    else if (a.sec < 0)
    {
        a.min--;
        a.sec+=60;
    }
    if (a.min > 59) 
    {
        a.hr++;
        a.min-=60;
    }
    else if (a.min < 0)
    {
        a.hr--;
        a.min+=60;
    }
    if (a.hr<0) {a = zero;}
    a.hr %= 24;
    return a;
}
