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


// Use pin 1 for debug port
#define DEBUG_INIT()      SBI( DDRD  , 3 )         // Debug on pin #1 PD5
#define DEBUG_1()         SBI( PORTD , 3)
#define DEBUG_0()         CBI( PORTD , 3)


#endif /* UTILS_H_ */