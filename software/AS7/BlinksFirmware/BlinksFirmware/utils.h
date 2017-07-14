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
#define DEBUG_INIT()      SBI( DDRE  , 2)         // Debug on pin #1 PD5
#define DEBUG_1()         SBI( PORTE , 2)
#define DEBUG_0()         CBI( PORTE , 2)


#endif /* UTILS_H_ */