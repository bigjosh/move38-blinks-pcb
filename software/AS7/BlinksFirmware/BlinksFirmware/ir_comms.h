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

// The RX API...

// Returns last received data (value 0-3) for requested face
// bit 2 is 1 if data found, 0 if not
// if bit 2 set, then bit 1 & 0 are the data
// So possible return values:
// 0x00=0b00000000=No data received since last read
// 0x04=0b00000100=Received 0
// 0x04=0b00000101=Received 1
// 0x04=0b00000110=Received 2
// 0x04=0b00000111=Received 3

uint8_t ir_read( uint8_t led);


// If bit set, then a new byte was received before the previous byte was read.
// Currently the older byte is kept. 

volatile uint8_t irled_rx_overflow;             


// Transmit a value (0-3) on face
// (only 2 bits of data for now)
// If no transmit in prgress, then returns immedeately and starts the transmit within 500us
// IF a transmit is in progress, then blocks until that is complete. 

uint8_t ir_send( uint8_t face , uint8_t data );

#endif /* IR-COMMS_H_ */