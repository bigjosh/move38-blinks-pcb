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

void ir_refresh(void);
    
#endif /* IR-COMMS_H_ */