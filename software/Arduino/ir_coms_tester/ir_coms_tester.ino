// Easier to do testing with a 2nd arduino on same system.
// Luckily same ports available on arduino, so easy to copy code back and forth
// must use an Arduino programmed up to interneral OSC 8mhz, with CLKDIV8 bringing down to 1mhz


#include <avr/power.h>    // For clock_prescale_set()  

#define F_CPU (1000000)           // Default fuses with /16 prescaller

#include <util/delay.h>         // Must come after F_CPU definition

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>           // PROGMEM to keep data in flash
#include <math.h>
#include <stdlib.h>                 // rand()


// Bit manipulation macros
#define SBI(x,b) (x|= (1<<b))           // Set bit in IO reg
#define CBI(x,b) (x&=~(1<<b))           // Clear bit in IO reg
#define TBI(x,b) (x&(1<<b))             // Test bit in IO reg


// IR transceivers
// There are 6 IR LEDs - one for each face

#define IR_CATHODE_PORT PORTC
#define IR_CATHODE_DDR  DDRC
#define IR_CATHODE_PIN  PINC

#define IR_ANODE_PORT PORTB
#define IR_ANODE_DDR  DDRB
#define IR_ANODE_PIN  PINB


// LED 1-6
#define LED_BIT(b) (_BV(b-1))   // Map to bits 0-5

// Bits 0-5 represent IR leds 0-6
// Default state is waiting for pulse, so...
// Anode   : driven 0 - to give a ground to the capactior
// Cathode : as input no pullup  with capacatance charged

void ir_init(void) {
  IR_ANODE_DDR = LED_BIT(1) | LED_BIT(2) | LED_BIT(3) | LED_BIT(4) | LED_BIT(5) | LED_BIT(6) ;  // Set all ANODES to drive (and leave forever)
  // Everything else ok in default reset state    
}

#define F_CPU (20000000UL/16UL)           // Default fuses with /16 prescaller

#warning cpu F_CPU

void ir_tx_pulse( uint8_t bitmask ) {

    // ANODE always driven

      IR_CATHODE_DDR |= bitmask ;   // Drive Cathode too (now driving low)
      
      IR_ANODE_PORT  |= bitmask;    // Blink!

      asm("nop");
      asm("nop");
      
      //_delay_us(1);
      IR_ANODE_PORT  ^= bitmask;   // un-Blink! Should be on for ~2us @1Mhz clock 


      IR_CATHODE_DDR ^= bitmask ;   // Cathode back to inpit too (now driving low)
      // TODO: Charge up cap whn all TX work done
     

}


void setup() {
  // put your setup code here, to run once:
  ir_init();
  clock_prescale_set(clock_div_16);       // Get us from Arduino 20Mhz XTAL to 1.25Mhz - close enough to the 1Mhz the Blink will run at. 
}

void loop() {
  // put your main code here, to run repeatedly:

  cli();      //Peski interrupts mess everything up!


  while (1) {

    ir_tx_pulse( LED_BIT(5) );

    _delay_ms(30);
  }

}
