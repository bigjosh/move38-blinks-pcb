/*

    Talk to the 6 IR LEDs that are using for communication with adjecent tiles

*/


#include "blinks.h"
#include "hardware.h"


#include <avr/interrupt.h>
#include <util/delay.h>         // Must come after F_CPU definition

#include "ir_comms.h"
#include "utils.h"

// Recharge the specified LEDs 
// Suppress pin change interrupt on them

// This gets called anytime one of the IR LED cathodes has a level change drops. This typically happens because some light 
// hit it and discharged the capacitance, so the pin goes from high to low. We initialize each pin at high, and we charge it
// back to high everything it drops low, so we should in practice only ever see high to low transitions here.

// Note that there is also a background task that periodically recharges all the LEDs frequently enough that 
// that they should only ever go low from a very bright source - like an IR led pointing right down their barrel. 


// Last received byte from corresponding IRLED
// TODO: Buffer? Async notice?

volatile uint8_t irled_value[IRLED_COUNT];   

// TODO: Save a jump and ret by inlining this?  

void ir_isr(void)
{	

    // bitstream  keeps track of the most recently received fixed time slices for each LED 
    // LEDs are checked for pulses at fixed time intervals in this ISR. 
    
    // 0000000 = Waiting for a start pulse. 
    // 0000001 = Start pulse received. Waiting for guard period.
    // 000001G = Guard received, waiting for data window 0
    // 00001GD = waiting for data window 1
    // 0001GDD = waiting for data window 2
    // 001GDDD = waiting for trailing guard
    // 01GDDDG = trailing guard received - done with this bit slot (go back to 00000 on this same pass)
    
    // We recharge the LED at each slice.
    // TODO: Maybe send two blinks to avoid a collision if we happened to TX at the exact moment that the RX is charging? Charging is very fast so no issue.
    
    // We can load these bit quickly by shifting 1 bit left per time slice, and we can test for 00000 to see if we are waiting and
    // test bit 5 to see if we are done. 
    
    // Both guards must be 0 or there is an error. 
    // TODO: If we get a 1 in a guard, go to state 00001 to get ready for a new start?
    
    // If none of the D slots are full then we received a 0 bit.
    
    // If one and only one of the D slots is filled then we received a 1 bit
    // (almost always the middle one, but can be the side ones
    // due to slightly different clock speeds between tiles). 
    
    // If 2 or more D slots are full then there was an error. 
    
    // a byte should always begin with a sync period of at least 6 
        
    static uint8_t ir_bitstream[IRLED_COUNT];  
    
    
    // Build incoming bits into bytes here
    // For now a simple shift
    // TODO: More efficient to keep these all together in a struct so we can use offset references?
    static uint8_t irled_buffer[IRLED_COUNT];     
    static uint8_t irled_bitcount[IRLED_COUNT];     
    
    const uint8_t BITCOUNT_ERROR = UINT8_MAX;               // Indicates that we need an idle before we can start getting good data again
    
                   
    // Sample the LEDs!
    
    uint8_t ir_LED_PIN_sample = IR_CATHODE_PIN;      // Quickly sample the PIN (we will work the bits later)
    
    // We want this time here to be as short as possible since we can miss a pulse that happens exactly while we are recharging.
    // Or possibly see a pulse twice if it exactly straddles the recharge interval.
    // TODO: Make pulses at east 2x as long as recharge cycle? Then we would need to also check for two very fast 
    // consecutive pulses here and ignore the 2nd one.
    
    // TODO: Do in ASM so we make sure each set is only one instruction?
       
    // charge up receiver cathode pins while keeping other pins intact
           
    // This will enable the pull-ups on the LEDs we want to change without impacting other pins
    // The other pins will stay whatever they were.
    
    // NOTE: We are doing something tricky here. Writing a 1 to a PIN bit actually toggles the PORT bit. 
    // This saves about 10 instructions to manually load, or, and write back the bits to the PORT. 
        
    /*
        19.2.2. Toggling the Pin
        Writing a '1' to PINxn toggles the value of PORTxn, independent on the value of DDRxn. 
    */
               
    IR_CATHODE_PIN =  IR_BITS;
    
    // (Only takes a tiny bit of time to charge up the cathode, even through the pull-up so no extra delay needed here...)
    
    // Stop charging LED cathode pins (toggle the triggered bits back o what they were)
    
    IR_CATHODE_PIN = IR_BITS;     


    // TODO: This is where the send code probably should go, right after we check the pin for incoming data...
    // For minimum possible collision chance
    
    // Now let's process the bits we captured above
    // Remember that an incoming blink will discharge the LED, so a 0 here means that we saw a blink in the previous time slice
    
    for( uint8_t led=0; led<IRLED_COUNT; led++) {
                
        if (ir_bitstream[led] == 0b00000000) {            // Did we just receive a pause (8 slices of no LED in a row?)
            
            irled_bitcount[led]=0;                                //  If so, we are clear to start looking for a new byte
            
                        DEBUG_1();
                        _delay_us(6);
                        DEBUG_0();
            
            
        } 
                
        ir_bitstream[led]  <<= 1;                                         // Shift up one to make room for new sample 
        ir_bitstream[led]  |= !( ir_LED_PIN_sample & _BV( led ) );        // Add most recent sample (NOT because a pulse discharges LED)
        
        if ( ir_bitstream[led]  & _BV(5) ) {                              // If bit 5 set, then we potentially have a full RX bit here
                                                                          // Bit 5 right now would be the initial trigger pulse form 5 time slices ago
            
            if ( ir_bitstream[led] & 0b00010001 ) {                       // Is either of the guard slots set?
                
                // If we get here, then there was noise in the bitsteam, so abort current byte
                // and wait for next pause followed by start bit
                
                irled_bitcount[led]= BITCOUNT_ERROR;  
                
                DEBUG_1();
                _delay_us(15);
                DEBUG_0();
                
                
            } else {
                
                // Guard test passed, now we need to make sure there is at most one data pulse present
                
                uint8_t data_slots = ( ir_bitstream[led] >> 1 ) & 0b00000111;
                                
                uint8_t databit=0;          // TODO: This case is ugly! 
                
                switch (data_slots) {


                    // These are the only allowed 1 bit combinations. In real life, these represent a single pulse
                    // aiming for the middle slot but might end up slightly in the left or right slots because
                    // of clock offsets between sender and receiver. 
                                       
                    case 0b00000001:
                    case 0b00000010:
                    case 0b00000100:
                        databit=1;
                        
                        // Fall thorough!
                                
                    case 0b00000000:
                    
                    
                    
                                        
                        // If we got here, then at most one data slot was filled and a valid 0 or 1 bit was received. 
                        
                        if (irled_bitcount[led]!=BITCOUNT_ERROR) {         // Don't process this bit if we are currently in error lockout and waiting for a idle to reset
                            
                            irled_buffer[led] <<= 1;                  // Make room for this next bit
                            irled_buffer[led] |= databit;             // add it at the bottom
                            
                            if (irled_bitcount[led] ==7) {            // did we jst make a full byte?
                                
                                irled_value[led] = irled_buffer[led];
                                
                                // No need to reset the buffer since it will naturally get cleared by next 8 shifts
                                
                                // TODO: Send an async notice that we got a byte?
                                                                
                                irled_bitcount[led]=0;
                                
                            } else {
                                
                                irled_bitcount[led]++;
                                
                                // TODO: make bitcount a mask instead so we can shift it and save an increment and test?
                                
                            }                                                                
                        }                            
                                                    
                        break;
                        
                    default: 
                        irled_bitcount[led]= BITCOUNT_ERROR;  
                        
                        DEBUG_1();
                        _delay_us(7);
                        DEBUG_0();
                        
                                               
                        break;
                }                                      
                    
                
            }                                
                            
        }            
            
    }        
                
                
    //ir_tx_pulse( LED_BIT(5) );          // Blink D5
        
    //_delay_ms(30);

    
    // If any LED pin changed while we were in this ISR, then we will get called again
    // as soon as we return and interrupts are enabled again.      
        
}


void ir_init(void) {
    
  DEBUG_INIT();     // TODO: Get rid of all debug stuff
      
    
  IR_ANODE_DDR = IR_BITS ;  // Set all ANODES to drive (and leave forever)
                            // THe PORT will be 0, so these will be driven low
                            // until we actively send a pulse
                            
  // Leave cathodes DDR in input mode. When we write to PORT, then we will be enabling pull-up which is enough to charge the 
  // LEDs and saves having to switch DDR every charge. 
  
  // Initial charge up of cathodes will happen first time ISR is called. 
      
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
// so it is easy to ease drop on it. 

void blinkIr(void) {
    
    // Lets start with 0
    
    DEBUG_INIT();
    
    // RX on 0
           
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

