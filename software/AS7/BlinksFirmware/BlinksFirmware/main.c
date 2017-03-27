/*
 * BlinksFirmware.c
 *
 * Created: 3/26/2017 8:39:50 PM
 * Author : josh.com
 */ 

#include <avr/io.h>

#define F_CPU 1000000

#include <util/delay.h>

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

