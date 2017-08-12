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
#define DEBUG_INIT()            SBI( DDRE  , 2); SBI(DDRE,1)         // DebugA on pin #19 PE2
                                                                     // DebugB on pin  #6 PE1 
#define DEBUGA_1()               SBI( PORTE , 2)
#define DEBUGA_0()               CBI( PORTE , 2)
#define DEBUGA_PULSE(width_us)   DEBUGA_1();_delay_us(width_us-2);DEBUGA_0()   // Generate a pulse. width must be >= 2us.

#define DEBUGB_1()               SBI( PORTE , 1)
#define DEBUGB_0()               CBI( PORTE , 1)
#define DEBUGB_PULSE(width_us)   DEBUGB_1();_delay_us(width_us-2);DEBUGB_0()   // Generate a pulse. width must be >= 2us.


#endif /* UTILS_H_ */