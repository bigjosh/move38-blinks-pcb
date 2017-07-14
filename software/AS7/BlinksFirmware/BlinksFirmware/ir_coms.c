/*

    Talk to the 6 IR LEDs that are using for communication with adjecent tiles

*/


#include "blinks.h"


#include <avr/interrupt.h>
#include <util/delay.h>         // Must come after F_CPU definition

#include "ir_comms.h"
#include "utils.h"



// IR transceivers
// There are 6 IR LEDs - one for each face

#define IR_CATHODE_PORT PORTC
#define IR_CATHODE_DDR  DDRC
#define IR_CATHODE_PIN  PINC

#define IR_ANODE_PORT PORTB
#define IR_ANODE_DDR  DDRB
#define IR_ANODE_PIN  PINB


// LED 1-6
#define LED_BIT(b) (_BV(b-1))   // Map to bits 0-5

// TODO: Renumber and rename IR LEDs to be 0 based (IR0-IR5)


// Bits 0-5 represent IR leds 0-6
// Default state is waiting for pulse, so...
// Anode   : driven 0 - to give a ground to the capactior
// Cathode : as input no pullup  with capacatance charged

void ir_init(void) {
  IR_ANODE_DDR = LED_BIT(1) | LED_BIT(2) | LED_BIT(3) | LED_BIT(4) | LED_BIT(5) | LED_BIT(6) ;  // Set all ANODES to drive (and leave forever)
  // Everything else ok in default reset state    
}

// Send a pulse on all LEDs that have a 1 in bitmask
// bit 0= D1, bit 1= D2...

void ir_tx_pulse( uint8_t bitmask ) {

    // ANODE always driven

      IR_CATHODE_DDR |= bitmask ;   // Drive Cathode too (now driving low)
      
      IR_ANODE_PORT  |= bitmask;    // Blink!
      
      // TODO: Is this the right TX pulse with? Currently ~6us total width
      // Making too long wastes (a little?) battery and time
      // Making too short might not be enough light to trigger the RX on the other side
      // when TX voltage is low and RX voltage is high?
      // Also replace with a #define and _delay_ms()

      asm("nop");
      asm("nop");
      asm("nop");
            
      //_delay_us(1);
      IR_ANODE_PORT  ^= bitmask;   // un-Blink! Should be on for ~2us @1Mhz clock 


      IR_CATHODE_DDR ^= bitmask ;   // Cathode back to inpit too (now driving low)
      // TODO: Charge up cap whn all TX work done
     
}


// Blink LED D5 at about 100hz for testing.
// We pick that one because it shows up on the MOSI pin (pin #1 on ISP)
// so it is easy to easedrop on it. 

void blinkIr(void) {
    
    // Lets start with 0
    
    DEBUG_INIT();
    
    // RX on 0
       
    // ANode always drive, default to 0
    SBI( IR_ANODE_DDR , 0 );    

    cli();      //Pesky interrupts mess everything up!
    
    while (1) {
        
       
        // charge up receiver cathode
        SBI( IR_CATHODE_PORT , 0 );         // Pull-up on
     //   SBI( IR_CATHODE_DDR , 0 );         // Pull-up on
        
        //_delay_us(20);
        
        CBI( IR_CATHODE_DDR , 0 );
        
        
        CBI( IR_CATHODE_PORT , 0 );         // stop charging, start floating input again
        
        _delay_ms(1);
        
        //ir_tx_pulse( LED_BIT(5) );          // Blink D5
        
        //_delay_ms(30);

        DEBUG_1();                
        while (TBI( IR_CATHODE_PIN , 0 ));
        DEBUG_0();        
                        
        //_delay_ms(10);
    }        
    
}    

