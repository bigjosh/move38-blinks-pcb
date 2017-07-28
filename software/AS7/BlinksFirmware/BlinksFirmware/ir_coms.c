/*

    Talk to the 6 IR LEDs that are using for communication with adjecent tiles

*/


#include "blinks.h"
#include "hardware.h"


#include <avr/interrupt.h>
#include <util/delay.h>         // Must come after F_CPU definition

#include "ir_comms.h"
#include "utils.h"


// Send a pulse on all LEDs that have a 1 in bitmask
// bit 0= D1, bit 1= D2...
// This clobbers whatever charge was on the LED, so only call after you have checked it.
// You must charge the LED after this too if you want to receive on it.

// TODO: Queue TX so they only happen after a successful RX or idle time. Unnecessary since TX time so short?


void ir_tx_pulse( uint8_t bitmask ) {

    // ANODE always driven

      IR_CATHODE_DDR |= bitmask ;   // Drive Cathode too (now driving low)
      
      // TODO: Use PIN toggle to save a load/store
      
      IR_ANODE_PORT  |= bitmask;    // Blink!
      
      // TODO: Is this the right TX pulse with? Currently ~6us total width
      // Making too long wastes (a little?) battery and time
      // Making too short might not be enough light to trigger the RX on the other side
      // when TX voltage is low and RX voltage is high?
      // Also replace with a #define and _delay_us() so works when clock changes?

      asm("nop");
      asm("nop");
      asm("nop");
            
      //_delay_us(1);
      IR_ANODE_PORT  ^= bitmask;   // un-Blink! Should be on for ~2us @1Mhz clock 


      IR_CATHODE_DDR ^= bitmask ;   // Cathode back to inpit too (now driving low)
      // TODO: Charge up cap whn all TX work done
     
}

// For testing. 


// from http://www.microchip.com/forums/m587239.aspx

uint8_t oddParity(uint8_t p)
     {
      p = p ^ (p >> 4 | p << 4);
      p = p ^ (p >> 2);
      p = p ^ (p >> 1);
      return p & 1;
     }

static uint8_t tx_count=0;      // count up 0-127 (will use the extra bit for parity)

static uint8_t tx_value=0;

// Bits are 3x spread to account for clock drift. 
// We always start at 0 which is the start pulse
// Then we send MSB bit at slot 2,8, and every 3rd slot until LSB 8th bit , followed by stop bit 
// Stop bit just makes sure we do not see a single pulse as a 0x00

static uint8_t send_slot=0;        // Count up to 255 just to put some idle between bytes   

// Call this once per slot and it will slowly and repeatedly send out the requested bit

void send_test_byte( uint8_t mask) {
    
    //tx_value=65;
    
    if (send_slot==0) {
        
            // Start 
            ir_tx_pulse( mask );        
            
    } else {
        
        uint8_t bitslot = (send_slot-1) / 3;
        
        if (bitslot<8) {                // Only send actual bits during the 1st 8 slots after 
        
            if ( ( (send_slot-1) - (bitslot*3) ) == 2 ) {       // Only send actual bit in the middle slot
            
                if ( tx_value & ( 1 << (7-bitslot) ) ) {            // Only send pulse if bit is 1
                    
                    ir_tx_pulse( mask );
                    
                }
                
            }
            
        }            
        
    }                                                                 
        
    send_slot++;
    
    if (send_slot == ( 3*8 ) + 20 ) {       // The 20 slots give a little breathing room between bytes for resync
        send_slot=0;
        
        tx_count++;                         // Cycle though values
        if (tx_count==128) tx_count=0;      // Only count up to 127 since we use the bottom bit for parity
    }
                
    tx_value = ( tx_count << 1 ) + oddParity( tx_count ) ;     // send 7 bit counter + 1 bit parity. 
            
}     


// Recharge the specified LEDs 
// Suppress pin change interrupt on them

// This gets called anytime one of the IR LED cathodes has a level change drops. This typically happens because some light 
// hit it and discharged the capacitance, so the pin goes from high to low. We initialize each pin at high, and we charge it
// back to high everything it drops low, so we should in practice only ever see high to low transitions here.

// Note that there is also a background task that periodically recharges all the LEDs frequently enough that 
// that they should only ever go low from a very bright source - like an IR led pointing right down their barrel. 


// Last received byte from corresponding IRLED
// TODO: Buffer? Async notice?

volatile uint8_t irled_RX_value[IRLED_COUNT];  
volatile uint8_t irled_TX_value[IRLED_COUNT];

#define IDLE_SLOTS (10*3)       // This many consecutive 0 slots to resync the begingof a new byte

// TODO: Save a jump and ret by inlining this?  

void ir_isr(void)
{	
    
    DEBUGA_1();
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
    
    // TODO: make up to 8 consecutive data bits without start pulse to improve throughput?  
    // Need some kind of check to prevent degenerate case where a single pulse looks like the byte 0x00...
        
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
        
    
    // Build incoming bits into bytes here
    // For now a simple shift
    // TODO: More efficient to keep these all together in a struct so we can use offset references?
    // TODO: Maybe use 7 bits so we can use a high 1 to mark shift steps?
        
    static uint8_t irled_RX_buffer[IRLED_COUNT];       // Hold incoming byte as bits are shifted in
    
    typedef enum { IR_RX_WAITING, IR_RX_ACTIVE } irled_RX_states;
        
    static irled_RX_states irled_RX_state[IRLED_COUNT];
    
    static uint8_t irled_RX_counter[IRLED_COUNT];       // IR_WAITING - counts down how many more idle slices we need to see
                                                        // IR_RX      - current bit being received 7-0
                             

    static uint8_t irled_RX_timeslots[IRLED_COUNT];    // How many data time slots left to receive to make a bit. We need this to oversample. 
    
    
    
    
    static uint8_t irled_TX_buffer[IRLED_COUNT];       // Hold incoming byte as bits are shifted out top first
    
    typedef enum { IR_TX_WAITING, IR_TX_ACTIVE } irled_TX_states;     
        
    static irled_TX_states irled_TX_state[IRLED_COUNT]; 
    
    static uint8_t irled_TX_counter[IRLED_COUNT];       // IR_WAITING - counts down how many more idle slices we need to see
                                                        // IR_TX      - how many more bits we need to send to make a byte
                             

    static uint8_t irled_TX_timeslots[IRLED_COUNT];    // How many data time slots before sending next pulse 
           
    /*        
        if (!( ir_LED_PIN_sample & _BV( 1 ))) {
            DEBUGA_PULSE(1000);                              // Show 1's
        } else {        
            DEBUGA_PULSE(10);                               // Mark sample            
        }        
    */

    /*

    if ( (ir_LED_PIN_sample & IR_BITS  ) != IR_BITS ) {
            DEBUG_1();
            _delay_us(7);
            DEBUG_0();                        
    }            

    return;
    */
    
    // TODO: This is where the send code probably should go, right after we check the pin for incoming data...
    // For minimum possible collision chance
    
    // Now let's process the bits we captured above
    // Remember that an incoming blink will discharge the LED, so a 0 here means that we saw a blink in the previous time slice


   // #define  IRLED_COUNT 1          // TODO:TESTING
    
    uint8_t ledbitmask = 1 << (IRLED_COUNT-1);        // Optimization to not have to recompute every pass
    
    uint8_t led=IRLED_COUNT-1;
    
    do {
        
        // Here we quickly...
        // 1. Sample current state
        // 2. Send next pulse if transmitting and in right slot and such
        // 3. Recharge LED for next time
        // The faster we do it, the less chance for collisions
        
        uint8_t sendpulseflag= 0;       // precompute as much as possible
                                        // to shorten time actually sending
                                        // because that is critical
        
        
        if ( irled_TX_state[led] == IR_TX_ACTIVE ) {
                        
            if (irled_TX_timeslots[led]==0) {   // Done with this bit?
                
                if ( irled_TX_counter[led] ) {      // Any bits left?
                                       
                    irled_TX_timeslots[led] = 2;    
                    
                    irled_TX_counter[led]--;             // Move on to next bit
                    
                } else {                // All done with this byte!
                    
                    irled_TX_state[led] = IR_TX_WAITING;
                    
                    irled_TX_counter[led] = 200;      // TODO: How long do we wait between sends?
                    
                }           
                
            } else {            // inside a bit
                
                if (irled_TX_timeslots[led] == 1) {         // IS this a middle timeslot to actualy send a pulse to indicate a 1 bit?
                    
                    if ( irled_TX_buffer[led] & _BV( irled_TX_counter[led] ) ) {
                        
                        sendpulseflag=1;        // If so, then send it!        
                        
                    }                     
                    
                }
                
                irled_TX_timeslots[led]--;      // We know we are not already at zero becuase we tested above
                
                    
            }                    
            
        } else { // TX_state_waiting
            
            if ( irled_TX_counter[led] ) {               // waiting to start xmit?
                
                irled_TX_counter[led]--;
                
            } else {
                
                irled_TX_state[led] = IR_TX_ACTIVE;
                
                irled_TX_buffer[led]=irled_TX_value[led];
                
                irled_TX_timeslots[led] = 2;
                
                irled_TX_counter[led] = 8;               // TODO: make this a  bitmask
                
                sendpulseflag = 1;                  // Send start pulse NOW
                
                
            }                                            
        }            
       
        // We want this time here to be as short as possible since we can miss a pulse that happens exactly while we are recharging.
        // Or possibly see a pulse twice if it exactly straddles the recharge interval.
        // TODO: Make pulses at east 2x as long as recharge cycle? Then we would need to also check for two very fast 
        // consecutive pulses here and ignore the 2nd one.
    
        // TODO: Do in ASM so we make sure each set is only one instruction?
        
        
        uint8_t ir_LED_PIN_sample = IR_CATHODE_PIN;      // Quickly sample the PIN (we will work the bits later)
        
        if (sendpulseflag) {
        
        
            IR_CATHODE_DDR |= ledbitmask;           // Drive cathode low
            IR_ANODE_PIN    = ledbitmask;           // Drive anode high (anode is alway in drive mode, PIN toggles PORT state)
        
            // LED is now on!

            // TODO: Is this the right TX pulse with? Currently ~6us total width
            // Making too long wastes (a little?) battery and time
            // Making too short might not be enough light to trigger the RX on the other side
            // when TX voltage is low and RX voltage is high?
            // Also replace with a #define and _delay_us() so works when clock changes?

            asm("nop");
            asm("nop");
            asm("nop");
            asm("nop");
            
        
            IR_ANODE_PIN    = ledbitmask;         // back to driving anode low (default state)
        
            IR_CATHODE_DDR &= ~ledbitmask;        // Cathode input again
        
            // DOne with send
                
        }
               
        // charge up receiver cathode pin while keeping other pins intact
           
        // This will enable the pull-ups on the LEDs we want to change without impacting other pins
        // The other pins will stay whatever they were.
    
        // NOTE: We are doing something tricky here. Writing a 1 to a PIN bit actually toggles the PORT bit. 
        // This saves about 10 instructions to manually load, or, and write back the bits to the PORT. 
        
        /*
            19.2.2. Toggling the Pin
            Writing a '1' to PINxn toggles the value of PORTxn, independent on the value of DDRxn. 
        */
    
        // TODO: These need to be asm because it sticks a load here.
               
        IR_CATHODE_PIN =  ledbitmask;
    
        // (Only takes a tiny bit of time to charge up the cathode, even through the pull-up so no extra delay needed here...)
    
        // Stop charging LED cathode pins (toggle the triggered bits back o what they were)
    
        IR_CATHODE_PIN =  ledbitmask;
        
                
        uint8_t currentSample = !( ir_LED_PIN_sample & ledbitmask );           // recent sample (NOT because a pulse discharges LED)
        
        if (currentSample) {
            
            //DEBUGB_PULSE(100);
            
        } else {
            
            //DEBUGB_PULSE(200);
        }                        
        
        if (currentSample) irled_RX_value[led] = 0x01;  //TODO: TEST
        
        if (irled_RX_state[led]== IR_RX_WAITING) {
                        

            if (currentSample == 0 ) {       // idle
                
                if (irled_RX_counter[led]) {
                    
                    irled_RX_counter[led]--;      // Countdown the number of 0's we need to lock
                    
                }
                
            } else {        // Received a pulse in idle mode. 
      
                
                if (irled_RX_counter[led]==0) {       // Did we get enough 0's yet (long enough idle) to start receiving?
                                
                    irled_RX_state[led] = IR_RX_ACTIVE;       // Ok, we had a long enough idle, and we just got a start bit. Start receiving!                                                    
                    irled_RX_timeslots[led] = 3;       // Start walking bits up
                    irled_RX_counter[led] = 7;         // Receive a full byte
                    irled_RX_buffer[led]=0;            // Start fresh since we check bit 0. This goes away if we switch to 7 bit. 
                    
                    
                    // Don't need to clear buffer since it will get shifted out anyway
                    
                } else {

                    
                    irled_RX_counter[led]= IDLE_SLOTS;       // Got a 1 too soon. Start over looking for idle. 

                }                                        
                
            }   
                         
        } else {        /// IR_RX
            
            if ( currentSample ) {
                                                                                                
                if ( irled_RX_buffer[led] & 0b00000001 ) {
                    
                    // If we get here, then we just found a 1 time slot, but there was already a 1 in this bit so 
                    // this is an error.
                    
                    //DEBUGB_PULSE(7);
                    
                    irled_RX_state[led] = IR_RX_WAITING;
                    irled_RX_counter[led] = IDLE_SLOTS;
                    
                } else {
                    
                    irled_RX_buffer[led] |= 0b00000001;

                }                                        
                
            }
            
            
            
            if (irled_RX_timeslots[led] == 0 ) {         // Read enough slots to make a bit?
                
                

                if (irled_RX_buffer[led] &  0x01) {        // 1 bit?
                    
                    DEBUGA_PULSE(150);
                                    
                } 
                
                
                
                
                
                if (irled_RX_counter[led] == 0 ) {         // Was this the last bit in a full byte?
                                        
                    irled_RX_value[led] = irled_RX_buffer[led];
                    irled_RX_state[led] = IR_RX_WAITING;
                    
                    uint8_t value = irled_RX_value[led] >> 1;      // Extract sent count
                    
                    if ( oddParity( value ) == (irled_RX_value[led] & 1) ) {
                        
                        //DEBUGB_PULSE(500);
                        
                    } else {                        
                    
                        //DEBUGB_PULSE(1000);
                        
                    }                        
                                        
                    // count is already 0, so we will start receiving next start bit immediately
                                        
                } else {
                    
                    irled_RX_buffer[led] <<= 1;    // Shift up to get ready for next bit                    
                    irled_RX_timeslots[led] = 3;   // Read next 3 times slots
                    irled_RX_counter[led]--;
                    
                }                                                           
                
            }  else {
                irled_RX_timeslots[led]--;
            }                            
            
        }            
                   
                   /* 
        
        if (irled_state[led]==IR_RX){
            
            if (currentSample) {
                DEBUG_PULSE(1000);
                _delay_us(500);
                                    
            } else {
                                    
                DEBUG_PULSE(500);
                _delay_us(1000);
                                                                        
            } 
                                                    
            
            
        }                                 
        */
                   
        // If any LED pin changed while we were in this ISR, then we will get called again
        // as soon as we return and interrupts are enabled again.      
        

        led--;        
        
        ledbitmask >>=1;
    
            
    } while (ledbitmask);
    
    DEBUGA_0();    

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



// Blink LED D5 at about 100hz for testing.
// We pick that one because it shows up on the MOSI pin (pin #1 on ISP)
// so it is easy to ease drop on it. 

void blinkIr(void) {
    
    // Lets start with 0
        
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

        DEBUGA_1();                
        while (TBI( IR_CATHODE_PIN , 0 ));
        DEBUGA_0();        
                        
        //_delay_ms(10);
    }        
    
}    

