/*
 * utils.h
 *
 * Created: 7/14/2017 1:50:04 PM
 *  Author: josh
 */ 


#ifndef UTILS_H_
#define UTILS_H_

// Bit manipulation macros
#define SBI(x,b) (x|= (1<<b))           // Set bit in IO reg
#define CBI(x,b) (x&=~(1<<b))           // Clear bit in IO reg
#define TBI(x,b) (x&(1<<b))             // Test bit in IO reg


// Use pin 19 (PE2)for debug port
#define DEBUG_INIT()            SBI( DDRE  , 2)         // Debug on pin #19 PE2 (otherwise unused)
#define DEBUG_1()               SBI( PORTE , 2)
#define DEBUG_0()               CBI( PORTE , 2)
#define DEBUG_PULSE(width_us)   DEBUG_1();_delay_us(width_us-2);DEBUG_0()   // Generate a pulse. width must be >= 2us.

#endif /* UTILS_H_ */