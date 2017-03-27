/*
 * BlinksFirmware.c
 *
 * Created: 3/26/2017 8:39:50 PM
 * Author : josh.com
 */ 

#include <avr/io.h>

#define F_CPU 1000000

#include <util/delay.h>

// Timer1 for internal time keeping (mostly timing IR pulses) because it is 16 bit and its pins happen to fall on ports that are handy for other stuff
// Timer0 A=Red, B=Green. Both happen to be on handy pins
// Timer2B for Blue duty. Works out perfectly because we can use OCR2A as a variable TOP to change the frequency for the charge pump, which is better to change than duty. 

int main(void)
{
    
    DDRD |= _BV(6);
    
    /* Replace with your application code */
    while (1) 
    {
        PORTD |= _BV(6);
        _delay_ms(100);
        PORTD &= ~_BV(6);
        _delay_ms(100);
    }
}

