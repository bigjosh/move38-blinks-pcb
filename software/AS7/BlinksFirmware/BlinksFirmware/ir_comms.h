/*
 * ir_comms.h
 *
 * All the functions for communication and waking on the 6 IR LEDs on the tile edges.
 *
 */ 

#ifndef IR_COMMS_H_
#define IR_COMMS_H_

#include <avr/io.h>


#define IRLED_COUNT FACE_COUNT

// Setup pins, interrupts

void ir_init(void);


// Send a pulse on all LEDs that have a 1 in bitmask
// bit 0= D1, bit 1= D2...

void ir_tx_pulse( uint8_t bitmask );


// Blink LED D5 at about 100hz for testing.
// We pick that one because it shows up on the MOSI pin (pin #1 on ISP)
// so it is easy to ease drop on it. 

void blinkIr(void);


// This ISR should be called at deterministically fixed intervals.
// It...
// 1. Reads LEDs to see if any pulses were received since last call
// 2. Recharges LEDs to be read to receive pulses during upcoming interval
// 3. Decodes bits and bytes and puts them into ir_values[] when a byte is successfully received.

// Faster actually less noise because we are recharging LED more often. 
// Speed limit is CPU overhead and increasing collisions and clock drift causing missed bits

// TODO: Maybe this should be a macro to save the overhead of calling a 2 instruction function.


// Called every 512us, but must not take more than 256us or it will clobber other background ISRs

void ir_rx_isr(void);


// The RX API...

volatile uint8_t irled_RX_value[IRLED_COUNT];   // MSB set indicates data here (data is bottom 7 bits). Set high bit to zero after reading to clear the way for next byte.
volatile uint8_t irled_rx_error;        // There was an invalid pulse pattern on the indicated face
volatile uint8_t irled_rx_overflow;     // The value[] buffer was not empty when a new byte was received


// Send IR data

void ir_tx_clk_isr(void);
void ir_tx_data_isr(void);




// Outgoing data
// TODO: This should only really be volatile in the foreground, not in the ISR
// How does that work in C?
// OR does this really need to be volatile? Foreground only tests and sets if clear. Hmm...
// High bit and low bit always must be set (start and stop bit) 

extern volatile uint8_t ir_tx_data[IRLED_COUNT];



// Last received byte from corresponding IRLED
// TODO: Buffer? Async notice?


// This value is continuously sent on the corresponding face
volatile uint8_t irled_TX_value[IRLED_COUNT];
   
#endif /* IR-COMMS_H_ */