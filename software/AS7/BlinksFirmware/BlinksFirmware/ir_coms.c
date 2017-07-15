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


// CONSTRAINTS:
// Note that all IR anodes must be on same port, as do all IR cathodes. 
// Cathodes must have pin change interrupt
//
// Each anode must be on the same bit as the corresponding anode. (this could be relaxed with extra code)


// All of the 6 GPIO bits used by IR pins. Also assumes these are the same bits in the pin change mask register.

#define IR_BITS     (_BV( 0 )|_BV( 0 )|_BV( 2 )|_BV( 3 )|_BV( 4 )|_BV( 5 ))

// We want a pin change interrupt on the CATHODES since these will get charged up 
// and then exposure will make them go low. 
// PORTC is connected to the cathodes, and they are on PCINT0-PCINT5
// which is controlled by PCIE0

/*
    PCICR
    Bit 1 – PCIE1:?Pin Change Interrupt Enable 1
    When the PCIE1 bit is set and the I-bit in the Status Register (SREG) is set, pin change interrupt 1 is
    enabled. Any change on any enabled PCINT[14:8] pin will cause an interrupt. The corresponding interrupt
    of Pin Change Interrupt Request is executed from the PCI1 Interrupt Vector. PCINT[14:8] pins are
    enabled individually by the PCMSK1 Register.
*/

/*
    PCMSK1
    Bits 0, 1, 2, 3, 4, 5, 6 – PCINT8, PCINT9, PCINT10, PCINT11, PCINT12, PCINT13, PCINT14:?Pin
    Change Enable Mask
    Each PCINT[15:8]-bit selects whether pin change interrupt is enabled on the corresponding I/O pin. If
    PCINT[15:8] is set and the PCIE1 bit in PCICR is set, pin change interrupt is enabled on the
    corresponding I/O pin. If PCINT[15:8] is cleared, pin change interrupt on the corresponding I/O pin is
    disabled.
*/


#define IR_PCI     PCIE1
#define IR_ISR     PCINT1_vect
#define IR_MASK    PCMSK1           // Each bit here corresponds to 1 pin
#define IR_PCINT   IR_BITS


// Recharge the specificed LEDs 
// Suppress pin change interrupt on them


// This gets called anytime one of the IR LED cathodes has a level change drops. This typically happens because some light 
// hit it and discharged the capacitance, so the pin goes from high to low. We initialize each pin at high, and we charge it
// back to high everything it drops low, so we should in practice only ever see high to low transistions here.

// Note that there is also a backround task that periodically recharges all the LEDs freqnectly enought that 
// that they should only ever go low from a very bright sounce - like an IR led pointing right down thier barrel. 

ISR(IR_ISR)
{	


    DEBUG_1();                
    
               
    // Capture the current time
    
    //uint8_t now = TIMER_NOW();
    
    // Find out which IR LED(s) went low to trigger this interrupt
            
    uint8_t ir_LED_triggered_bits = (~IR_CATHODE_PIN) & IR_BITS;      // A 1 means that LED triggered
    
    PCMSK1 &= ~ir_LED_triggered_bits;                                 // stop Triggering interupts on these pins becuase they are going to change when we charge them
    
    // charge up receiver cathode pins while keeping other pins intact
           
    // This will enable the pull-ups on the LEDs we want to change without impacting other pins
    // The other pins will stay whatever they were.
    
    // NOTE: We are doing something tricky here. Writing a 1 to a PIN bit actually toggles the PORT bit. 
    // This saves about 10 instructions to manually load, or, and write back the bits to the PORT. 
    
    
    /*
        19.2.2. Toggling the Pin
        Writing a '1' to PINxn toggles the value of PORTxn, independent on the value of DDRxn. 
    */
    
    IR_CATHODE_PIN =  ir_LED_triggered_bits;
    
    // Only takes a tiny bit of time to charge up the cathode, even though the pull-up so no extra delay needed here...
    

    PCMSK1 |= ir_LED_triggered_bits;    // Re-enable pin change on the pins we just charged up
                                        // Note that we must do this while we know the pins are still high
                                        // or there might be a *tiny* race condition if the pin changed in the cycle right after
                                        // we finished charging but before we enabled interrupts. This would latch until the next 
                                        // recharge timeout.
                                            
    // Stop charging LED cathode pins (toggle the triggered bits backt o what they were)
    
    IR_CATHODE_PIN = ir_LED_triggered_bits;     
                
    //ir_tx_pulse( LED_BIT(5) );          // Blink D5
        
    //_delay_ms(30);

    DEBUG_0();   
    
    // If any LED pin changed while we were in this ISR, then we will get called again
    // as soon as we return and interrupts are enabled again.      
        
}


// This should be called by a background timer to refresh any LEDs that have not trigged lately.
// This keeps them topped off so that normal bleeding discharge or low level ambient light is not enough to 
// trigger a pin change.

// Since the pin change interrupt always recharges any pins that wen't low, these should all already be high 
// so topping them off should not trigger a pin change. 

// Note that there is  a race condition here where an LED could discharge between when the timer fires and turns off interrupts,
// and when this code charges the LED back up. If this happens, we will see the pin change when interrupts are reenabled after
// the timer is finish and do a redundant recharge f that LED.

// TODO: Maybe this should be a macro to save the overhead of calling a 2 instruction function.
// TODO: Use PIN rather than PORT to make smaller and faster.

void ir_refresh(void) {
        
    //DEBUG_1();                

    IR_CATHODE_PORT |= IR_BITS;         // Enable Pull-ups
    
    // Charging right now...
    
    IR_CATHODE_PORT &= ~IR_BITS;        // Disable pull-ups
    //DEBUG_0();   
        
}    


/*

    Optimal IR refresh ISR...

 16c:	8f 93       	push	r24
 16e:	8d e3       	ldi	r24, 0x3D	; 61
 170:	86 b9       	out	0x06, r24	; 6
 172:	86 b9       	out	0x06, r24	; 6
 174:	8f 91       	pop	r24
 17e:	18 95       	reti

*/

// Bits 0-5 represent IR leds 0-6
// Default state is waiting for pulse, so...
// Anode   : driven 0 - to give a ground to the capactior
// Cathode : as input no pullup  with capacitance charged

// Note that you need to enable interrupts for this to work!

void ir_init(void) {
    
  DEBUG_INIT();     // TODO: Get rid of all debug stuff
      
    
  IR_ANODE_DDR = IR_BITS ;  // Set all ANODES to drive (and leave forever)
                            // THe PORT will be 0, so these will be driven low
                            // until we actively send a pulse
  
  // Initial charge up of cathodes
  
  IR_CATHODE_PORT |= IR_BITS;
    
  // Pin change interrupt setup
  IR_MASK = IR_PCINT;             // Enable pin in Pin Change Mask Register for all 6 cathode pins. Any chyange after this will set the pending interrupt flag.
  SBI( PCICR , IR_PCI );          // Enable the pin group

  // Stop charging  
  IR_CATHODE_PORT &= ~IR_BITS;
  
  

    
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

