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
// so it is easy to easedrop on it. 

void blinkIr(void);

#endif /* IR-COMMS_H_ */