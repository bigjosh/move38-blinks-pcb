/*
 * ir_comms.h
 *
 * All the functions for communication and waking on the 6 IR LEDs on the tile edges.
 *
 */ 

#ifndef IR_COMMS_H_
#define IR_COMMS_H_

#include <avr/io.h>


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

// TODO: Maybe this should be a macro to save the overhead of calling a 2 instruction function.

void ir_isr(void);

// Last received byte from corresponding IRLED
// TODO: Buffer? Async notice?

volatile uint8_t irled_value[IRLED_COUNT];   
    
#endif /* IR-COMMS_H_ */