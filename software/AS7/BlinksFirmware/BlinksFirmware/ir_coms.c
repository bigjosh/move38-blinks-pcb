/*

    Talk to the 6 IR LEDs that are using for communication with adjacent tiles


    THEORY OF OPERATION
    ===================
    
    All communication is 7 bits wide. This leaves us with an 8th bit to use as a canary bit to save needing any counters.
    
    Bytes are transmitted least significant bit first.
    
    MORE TO COME HERE NEED PICTURES. 


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
    
    // TODO: CHeck for input before sending and abort if found...

    // ANODE always driven

    IR_CATHODE_DDR = bitmask ;   // Drive Cathode too (now driving low)

    // Anode pins are driven output and low normally, so this will
    // make them be driven high output 
     
    IR_ANODE_PIN  = bitmask;    // Blink!       (Remember, a write to PIN actually toggles PORT)
      
    // TODO: Is this the right TX pulse with? Currently ~6us total width
    // Making too long wastes (a little?) battery and time
    // Making too short might not be enough light to trigger the RX on the other side
    // when TX voltage is low and RX voltage is high?
    // Also replace with a #define and _delay_us() so works when clock changes?

    //DEBUGA_1();    

    //TODO: Optimize this to be exact minimum for the distance in the real physical object
    
    // Currently chosen empirically to work with some tile cases Jon made 7/28/17

    _delay_us(10);
    
    /*
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    */
    
    //DEBUGA_0();
                
    IR_ANODE_PIN  = bitmask;    // Un-Blink! Sets anodes back to low (still output)      (Remember, a write to PIN actually toggles PORT)

    IR_CATHODE_DDR ^= bitmask ;   // Cathode back to input too (now driving low) (XOR saves 4 bytes over AND+NOT)
      
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
               
    IR_CATHODE_PIN =  bitmask;
    
    // (Only takes a tiny bit of time to charge up the cathode, even through the pull-up so no extra delay needed here...)
    
    // Stop charging LED cathode pins (toggle the triggered bits back o what they were)
    
    IR_CATHODE_PIN =  bitmask;
             
     
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


// Top bit always set to indicate byte present and Ready to send
// Cleared when transmission begins

volatile uint8_t irled_TX_value[IRLED_COUNT];


static uint8_t bitBuffer[IRLED_COUNT];      // Bits to be shifted out
                                            // Starts with 1 in top bit
                                            // Bit get shifted down as they are sent out LSB first
                                            // That 1 lets us know when we are done without keeping a counter

// IR clock and data pulses alternate. 
// A clock pulse is always sent if there is an xmit in progress
// A data pulse is only sent for 1 bits


// Send IR clock 

void ir_tx_clk_isr(void) {
    
    uint8_t outgoingPulses=0;                   // Build up which faces are going to see an outgoing pulse on this timeslot
        
    // TODO: invert or use a bitslide to make more efficient
    
    for(uint8_t led=0; led<IRLED_COUNT;led++) {
        
        if (bitBuffer[led]==0) {                // Currently idle on this face
            
            if ( irled_TX_value[led] ) {             // Is there a byte waiting to go out? (Will never be zero because of that top marker bit)
             
                // Start sending a new byte...
             
                bitBuffer[led] = irled_TX_value[led];        // Put it into our buffer so we will start sending it!   
                                                             // We use the top bit as a marker so we know when we are done
                                                             // and that 1 will also send the stop bit
                
             
                irled_TX_value[led]=0x00;              // Consume it so foreground knows 
                                
            }
            
        }
        
        
        if ( bitBuffer[led]  ) {               // Is there a transfer in progress on this face?
            
            outgoingPulses |= _BV(led);            // Send a clock pulse
                
        }                             
                                     
    }        
    
    //DEBUGA_1();
    
    ir_tx_pulse( outgoingPulses );

    //DEBUGA_0();
    
    return;
    
}    

// Send IR data
// Wow, this ended up being clean code. Must be doing it right!

void ir_tx_data_isr(void) {
    
    
    //DEBUGB_1();
    
    uint8_t outgoingPulses=0;                   // Build up which faces are going to see an outgoing pulse on this timeslot
    
    // TODO: invert or use a bitslide to make more efficient
    
    for(uint8_t led=0; led<IRLED_COUNT;led++) {
        
        // TODO: In ASM we could fold this test and shift into one operation using C bit
        
        if ( TBI( bitBuffer[led] , 0 ) )  {                // Is there a 1 in low bit?
            
            outgoingPulses |= _BV(led);            // Send a data pulse
                
        } 
        
        bitBuffer[led] >>=1;            // Shift down for next round
        
    }        
    
    //DEBUGA_1();
    
    ir_tx_pulse( outgoingPulses );

    //DEBUGA_0();
    //DEBUGB_0();
    
    return;
    
}    



// Recharge the specified LEDs 
// Suppress pin change interrupt on them

// This gets called anytime one of the IR LED cathodes has a level change drops. This typically happens because some light 
// hit it and discharged the capacitance, so the pin goes from high to low. We initialize each pin at high, and we charge it
// back to high everything it drops low, so we should in practice only ever see high to low transitions here.

// Note that there is also a background task that periodically recharges all the LEDs frequently enough that 
// that they should only ever go low from a very bright source - like an IR led pointing right down their barrel. 


// Last received byte from corresponding IRLED
// High bit always set, so yuo can set to zero and see if a new one arrived by checking high bit
// TODO: Buffer? Async notice?

volatile uint8_t irled_RX_value[IRLED_COUNT];  

volatile uint8_t irled_rx_error;        // There was an invalid pulse pattern on the indicated face
volatile uint8_t irled_rx_overflow;     // The value[] buffer was not empty when a new byte was recieved


struct tx_state_struct {
    uint8_t bitwindow;          // A sliding window of last 8 samples received. Most recent sample in LSB.
    uint8_t bytebuffer;         // Bits get shifted into here to build bytes. Every byte starts with a 1 in the high bit.
                                // As soon as a byte is decoded, we copy it into values[] to share it with the foreground.
                                // When bytebuffer is 0, then we are waiting for an idle to set it to 0x01 so we we start decoding

};    

/*

    TIMING:
    
    Just straight static uint8_t arrays for bitwindow and bytebuffer takes 312us per pass.
    Array of structs each with the two uints inside takes 220us per pass.
    Array of structs with a pointer that steps though them each iteration take 132us per pass. Wow gcc, you really make me do this?

*/


// TODO: Save a JMP and RET by inlining this?  

// Called every 512us, but must not take more than 256us or it will clobber other background ISRs
// WARNING: IF you modify this code, you MUST make sure it ALWAYS completes in less than ~230us or else things will get unpredicible. 

static struct tx_state_struct tx_state[IRLED_COUNT];


void ir_rx_isr(void)
{	

    //DEBUGA_1();
    
    /*
    static uint8_t bitwindow[IRLED_COUNT];      /
    static uint8_t bytebuffer[IRLED_COUNT];     

    */
    
    
    uint8_t ir_LED_PIN_sample = IR_CATHODE_PIN;      // Quickly sample the PIN (we will work the bits later)
    
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
               
    IR_CATHODE_PIN =  IR_BITS;
    
    // (Only takes a tiny bit of time to charge up the cathode, even through the pull-up so no extra delay needed here...)
    
    // Stop charging LED cathode pins (toggle the triggered bits back o what they were)
    
    IR_CATHODE_PIN =  IR_BITS;
        
                
    uint8_t currentSample = ~ir_LED_PIN_sample;           // recent sample (NOT because a pulse discharges LED)
    
    
    if ( (currentSample & 0x01)) {
        //DEBUGA_PULSE(20);
    } else {
        
    } 
    
        //DEBUGA_PULSE(2);
                   
            
    // work out way though decoding the samples 
    
    // TODO: invert or use a bitslide to make more efficient
    
    // Here we unpack the sample that has 1 bit per face into the moving windows for each face
    

    //DEBUGA_1();        

    static struct tx_state_struct *current_tx_state = tx_state; 
    
        
    for(uint8_t led=0; led<IRLED_COUNT; led++) {
        
        /*
        uint8_t currentBitwindow = tx_state[led].bitwindow;                  // Wow, the compiler really needs us to make a local copy here for else big and slow code.
        uint8_t currentBytebuffer = tx_state[led].bytebuffer;                // Yet this one does not seem to matter
        */

        uint8_t currentBitwindow = current_tx_state->bitwindow;                  // Wow, the compiler really needs us to make a local copy here for else big and slow code.
        uint8_t currentBytebuffer = current_tx_state->bytebuffer;                // Yet this one does not seem to matter

        
        currentBitwindow <<= 1;           // Make room for new sample
        
        currentBitwindow |= (currentSample & 0b00000001);      // Grab received sample for ths face
        
        currentSample >>=1;                                 // Shift down the sample so low bit is new face for next pass
        
        // OK, so the bit window now has the current sample and the past 7 samples in it.
        // Now we will check if these sames make a valid bit pattern
        // This is complicated because there is clock drift between the TX and RX sides so a pulse
        // can land in an adjacent position to the expected one. 
        
        // We only care about the bottom 6 positions of the window for decoding bits. The two higher ones will be the left over from a perviously decoded bit
        // One bit is a leading clock pulse, the presence or absence of a pulse for date, and then the trailing clock pulse.
        // (The final data bit in a byte has an extra trailing clock pulse)
        
        // We stipulate that the leading clock will be window position #5 and then look at all the other lower window positions to
        // to see if they could make a 
        
        // First check if we just saw an idle period so we should start decoding...
        
        if (currentBitwindow == 0b00000000 ) {
            
            currentBytebuffer = 0x01;        // This clears the MSB of the buffer, and resets us to the first bit, so we are ready to receive
            
        } else {   
            
            if (currentBytebuffer !=0 ) {      // Are we currently decoding? (If not, ignore everything except the idle reset above)
                                
                if ( currentBitwindow & 0b00100000 ) {            // Did we get a clock pulse? If so, try to decode a bit!
                
                
                    if (      ( currentBitwindow & 0b00100001 ) == 0b00100001 ) {      // C00D0C - TX clock slower
                    
                        currentBytebuffer <<=1;                                 // make room for new bit in byte buffer
                        currentBytebuffer |= ( currentBitwindow>> 2 & 0x01);     // Extract and save data bit 
                            
                        currentBitwindow = 0b00000001;                          // Save the tailing clock to be leading clock next time                        
                    
                    } else if ( ( currentBitwindow & 0b00110111 ) == 0b00100010 ) {      // C0D0C0 - TX clock matches
                    
                        currentBytebuffer <<=1;                                 // make room for new bit in byte buffer
                        currentBytebuffer |= ( currentBitwindow >> 3 & 0x01);     // Extract and save data bit 
                            
                        currentBitwindow = 0b00000010;                          // Save the tailing clock to be leading clock next time                    
                        

                        /*                        
                        if (currentBytebuffer & 0x01) {
                            DEBUGA_PULSE(50); 
                        } else {
                            DEBUGB_PULSE(50);
                        } 
                        */


                    } else if ( (currentBitwindow & 0b00100110 ) == 0b00100100 ) {      // CD0C0X - TX clock faster (X=dont' care because this is the following data bit)
                    
                        currentBytebuffer <<=1;                                 // make room for new bit in byte buffer
                        currentBytebuffer |= ( currentBitwindow >> 4 & 0x01);     // Extract and save data bit 
                    
                        currentBitwindow = currentBitwindow & 0b00000101;         // Save the tailing clock and data for next time    
                        
                    
                    } else {
                    
                        // Pulses do not match any possible sent patterns, so must be noise error
                                            
                        // If we get here then there was a noisy pulse pattern, so all bets are off
                        // We should start looking for a start bit again (which is preceded by idle)
                            
                        irled_rx_error |= _BV( led );       // Tell foreground
                            
                       currentBytebuffer = 0x00;     // We will not start decoding again until an idle
                        
                    }
                    
                    
                    if (currentBytebuffer & 0x80) {       // Have we decoded a full byte?!?
                        
                        // Note that to get here, we had to have successfully decoded 7 consecutive bits with valid patterns,
                        // which adds some filtering.
                        
                        if (irled_RX_value[led] == 0 )  {   // Is the incoming buffer free?
                            
                            irled_RX_value[led] = currentBytebuffer;  // Send to foreground!
                            
                        } else {
                            
                            irled_rx_overflow |= _BV( led );       // Tell foreground they are too damn slow!

                        }
                        
                        currentBytebuffer = 0x00;         // Start hunting for an idle to receive next byte!                                                                                
                        
                        //DEBUGB_PULSE(30);
                        
                                                                                                  
                    }                                                       
                
                }                                       
                                                        
            }                                
            
        }      
        
        
        /*
        tx_state[led].bitwindow = currentBitwindow;  
        tx_state[led].bytebuffer = currentBytebuffer;    
        */
        
        current_tx_state->bitwindow = currentBitwindow;  
        current_tx_state->bytebuffer = currentBytebuffer;    
                       
    }                      

    //DEBUGA_0();


    return;
    
}                    

void ir_init(void) {
    
  DEBUG_INIT();     // TODO: Get rid of all debug stuff
      
    
  IR_ANODE_DDR = IR_BITS ;  // Set all ANODES to drive (and leave forever)
                            // The PORT will be 0, so these will be driven low
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

        //DEBUGA_1();                
        while (TBI( IR_CATHODE_PIN , 0 ));
        //DEBUGA_0();        
                        
        //_delay_ms(10);
    }        
    
}    

