/*
 * BlinksFirmware.c
 *
 * Created: 3/26/2017 8:39:50 PM
 * Author : josh.com
 */ 

#include <avr/io.h>

#define F_CPU 1000000           // Default fuses



#include <util/delay.h>

#define BUTTON_DOWN() (PIND & _BV(7))           // PCINT23

ISR(PCINT2_vect)
{
    //code
}

void commonActivate( uint8_t line ) {           // Also deactivates previous
    
    switch (line) {
        
        case 0:
            DDRD &= ~_BV(2);
            PORTD&= ~_BV(2);
            DDRB |= _BV(6);
            PORTB|= _BV(6);
            break;
            
        case 1:
            DDRB &= ~_BV(6);
            PORTB&= ~_BV(6);
            DDRD |= _BV(0);
            PORTD|= _BV(0);
            break;
        
        case 2:
            DDRD &= ~_BV(0);
            PORTD&= ~_BV(0);
            DDRB |= _BV(7);
            PORTB|= _BV(7);
            break;
            
        case 3:
            DDRB &= ~_BV(7);
            PORTB&= ~_BV(7);
            DDRD |= _BV(1);
            PORTD|= _BV(1);
            break;
        
        case 4:
            DDRD &= ~_BV(1);
            PORTD&= ~_BV(1);
            DDRD |= _BV(4);
            PORTD|= _BV(4);
            break;
            
        case 5:
            DDRD &= ~_BV(4);
            PORTD&= ~_BV(4);
            DDRD |= _BV(2);
            PORTD|= _BV(2);
            break;
                   
    }
    
}

// Timer1 for internal time keeping (mostly timing IR pulses) because it is 16 bit and its pins happen to fall on ports that are handy for other stuff
// Timer0 A=Red, B=Green. Both happen to be on handy pins
// Timer2B for Blue duty. Works out perfectly because we can use OCR2A as a variable TOP to change the frequency for the charge pump, which is better to change than duty. 

int main(void)
{
    
    DDRD |= _BV(3);
    
    while (1) {
        
        for(int i=0;i<10000;i++) {
            PORTD |= _BV(3);
            _delay_us(30);
            PORTD &= ~_BV(3);
            _delay_us(100);
        }            

        for(int i=0;i<10000;i++) {
            PORTD |= _BV(3);
            _delay_us(15);
            PORTD &= ~_BV(3);
            _delay_us(250);
        }
        
        _delay_ms(1000);
 
    }
    
    //DDRD |= _BV(3);     // BLUE
    
    while (1) {
    
    
        for( int countdown = 1500; countdown>0; countdown-=10) {
            for( uint8_t p=0; p<=5; p++ ) {
        
                commonActivate(p);
        
                for( int delay=countdown;delay;delay--) {
                   DDRD |= _BV(6);     // RED
                   _delay_us(5);
                   DDRD &= ~_BV(6);
                   _delay_us(25);

                }
            }
        }           
        
        for( int countdown = 1500; countdown>0; countdown-=10) {
            for( uint8_t p=0; p<=5; p++ ) {
                
                commonActivate(p);
                
                for( int delay=countdown;delay;delay--) {
                    DDRD |= _BV(5);     // green
                    
                    _delay_us(5);
                    DDRD &= ~_BV(5);
                    _delay_us(25);

                }
            }
        }

        for( int countdown = 1000; countdown>0; countdown-=1) {

            for( uint8_t p=0; p<=5; p++ ) {
            
                commonActivate(p);
            
                if (countdown & 0b01000 ) {
                    DDRD |= _BV(3);     // BLUE
                    
                }                    
                
                _delay_us(300);
                DDRD &= ~_BV(3);
                _delay_us(1);

                
            }
        }        
    }        
}

