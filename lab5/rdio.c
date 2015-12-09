// OregonState EECS
// Microcontroller System Design
// lab4_code.c 
// Joshua Reed
// Dec. 8, 2015
//
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "libs/lcd_functions.h"
#include "libs/lcd_functions.h"
#include "libs/uart_functions.h"
#include "libs/si4734.h"
#include "libs/twi_master.h"

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
char remote_temp[2];

// set to KRKT Albany at 99.9Mhz
extern uint16_t current_fm_freq = 9990; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
extern uint8_t si4734_wr_buf[9];
extern uint8_t si4734_rd_buf[9];
extern uint8_t si4734_tune_status_buf[8];
extern volatile uint8_t STC_interrupt; //indicates tune or seek is done

//******************************************************************************
//                          External Interrupt 7 ISR
// Handles the interrupts from the radio that tells us when a command is done.
// The interrupt can come from either a "clear to send" (CTS) following most
// commands or a "seek tune complete" interrupt (STC) when a scan or tune command
// like fm_tune_freq is issued. The GPIO2/INT pin on the Si4734 emits a low
// pulse to indicate the interrupt. I have measured but the datasheet does not
// confirm a width of 3uS for CTS and 1.5uS for STC interrupts.                 
//
// I am presently using the Si4734 so that its only interrupting when the
// scan_tune_complete is pulsing. Seems to work fine. (12.2014)
//
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7. The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect){STC_interrupt = TRUE;}

//*******************************************************************************
//                                                          main 
//*******************************************************************************
int main()
{
    lcd_init();
    uint8_t rssi = 0;
    //Port E inital values and setup.  This may be different from yours for bits 0,1,6.
    //                           DDRE:  0 0 0 0 1 0 1 1
    //   (^ edge int from radio) bit 7--| | | | | | | |--bit 0 USART0 RX
    //(shift/load_n for 74HC165) bit 6----| | | | | |----bit 1 USART0 TX
    //                           bit 5------| | | |------bit 2 (new radio reset, active high)
    //                  (unused) bit 4--------| |--------bit 3 (TCNT3 PWM output for volume control)
    DDRE  |= 0x04; //Port E bit 2 is active high reset for radio
    DDRE  |= 0x40; //Port E bit 6 is shift/load_n for encoder 74HC165
    DDRE  |= 0x08; //Port E bit 3 is TCNT3 PWM output for volume
    PORTE |= 0x04; //radio reset is on at powerup (active high)
    PORTE |= 0x40; //pulse low to load switch values, else its in shift mode
    while(1) 
    {
    
        //Given the hardware setup reflected above, here is the radio reset sequence.
        //hardware reset of Si4734
        PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
        DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
        PORTE |=  (1<<PE2); //hardware reset Si4734
        _delay_us(200);     //hold for 200us, 100us by spec
        PORTE &= ~(1<<PE2); //release reset
        _delay_us(30);      //5us required because of my slow I2C translators I suspect
                            //Si code in "low" has 30us delay...no explaination in documentation
        DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt
        
        
        //Once its setup, you can tune the radio and get the received signal strength.
        while(twi_busy()){} //spin while TWI is busy
        fm_pwr_up();        //power up radio
        while(twi_busy()){} //spin while TWI is busy
        fm_tune_freq(9990);     //tune to frequency
        
        //retrive the receive strength and display on the bargraph display
        while(twi_busy()){}                //spin while TWI is busy
        fm_rsq_status();                   //get status of radio tuning operation
        rssi =  si4734_tune_status_buf[4]; //get tune status
        _delay_ms(100);
    }   
    //redefine rssi to be a thermometer code
    // if(rssi<= 8) {rssi = 0x00;} else
    // if(rssi<=16) {rssi = 0x01;} else
    // if(rssi<=24) {rssi = 0x03;} else
    // if(rssi<=32) {rssi = 0x07;} else
    // if(rssi<=40) {rssi = 0x0F;} else
    // if(rssi<=48) {rssi = 0x1F;} else
    // if(rssi<=56) {rssi = 0x3F;} else
    // if(rssi<=64) {rssi = 0x7F;} else
    // if(rssi>=64) {rssi = 0xFF;}
}
