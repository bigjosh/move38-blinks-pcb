/*
 * BlinksFirmware.c
 *
 * Created: 3/26/2017 8:39:50 PM
 * Author : josh.com
 */ 

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#define F_CPU 1000000           // Default fuses

#include <util/delay.h>         // Must come after F_CPU definition


// Bit manipulation macros
#define SBI(x,b) (x|= (1<<b))           // Set bit in IO reg
#define CBI(x,b) (x&=~(1<<b))           // Clear bit in IO reg
#define TBI(x,b) (x&(1<<b))             // Test bit in IO reg


// Common Anodes - We drive these 1 to select Pixel. 

#define PIXEL1_PORT PORTB
#define PIXEL1_DDR  DDRB
#define PIXEL1_BIT  6

#define PIXEL2_PORT PORTB
#define PIXEL2_DDR  DDRB
#define PIXEL2_BIT  7

#define PIXEL3_PORT PORTD
#define PIXEL3_DDR  DDRD
#define PIXEL3_BIT  0

#define PIXEL4_PORT PORTD
#define PIXEL4_DDR  DDRD
#define PIXEL4_BIT  4

#define PIXEL5_PORT PORTD
#define PIXEL5_DDR  DDRD
#define PIXEL5_BIT  2

#define PIXEL6_PORT PORTD
#define PIXEL6_DDR  DDRD
#define PIXEL6_BIT  1

#define PIXEL_COUNT 6

// Here are the raw compare register values for each pixel
// These are precomputed from brightness values because we read them often from inside an ISR
// Note that for reg & green, 255 corresponds to OFF and 250 is about maximum prudent brightness. 
volatile uint8_t rawValueR[PIXEL_COUNT];
volatile uint8_t rawValueG[PIXEL_COUNT];
volatile uint8_t rawValueB[PIXEL_COUNT];


// Here are the RGB for each pixel
// 0=0ff, 255=full brightness
// These values are only read and updated once per full display refresh to avoid tearing
volatile uint8_t PixelR[PIXEL_COUNT];
volatile uint8_t PixleG[PIXEL_COUNT];
volatile uint8_t PixelB[PIXEL_COUNT];

// RGB Sinks - We drive these low to light the selected color (note that BLUE has a charge pump on it)
//This will eventually be driven by timers

#define LED_R_PORT PORTD
#define LED_R_DDR  DDRD
#define LED_R_BIT  6

#define LED_G_PORT PORTD
#define LED_G_DDR  DDRD
#define LED_G_BIT  5

#define LED_B_PORT PORTD
#define LED_B_DDR  DDRD
#define LED_B_BIT  3


// This pin is used to sink the blue charge pump
// We drive this HIGH to turn off blue, which otherwise blue led could 
// come on if the battery voltage is high enough to overcome the forward drop on the
// blue LED + Schottky

#define BLUE_SINK_PORT PORTE
#define BLUE_SINK_DDE  DDRE
#define BLUE_SINK_BIT  3



#define BUTTON_PORT    PORTD
#define BUTTON_PIN     PIND
#define BUTTON_BIT     7


#define BUTTON_DOWN() (TBI(BUTTON_PIN,BUTTON_BIT))           // PCINT23

ISR(PCINT2_vect)
{
    //code
}



// Set all the pixel drive pins to output. 

void setupPixelPins(void) {

        // TODO: Compare power usage for driveing LOW with making input. Maybe slight savings becuase we don't have to drain capacitance each time? Probably not noticable...        
        // TODO: This could be slightly smaller code by loading DDRD with a full byte rather than bits
        
        // Setup all the anode driver lines to output. They will be low by default on bootup
        SBI( PIXEL1_DDR , PIXEL1_BIT );
        SBI( PIXEL2_DDR , PIXEL2_BIT );
        SBI( PIXEL3_DDR , PIXEL3_BIT );
        SBI( PIXEL4_DDR , PIXEL4_BIT );
        SBI( PIXEL5_DDR , PIXEL5_BIT );
        SBI( PIXEL6_DDR , PIXEL6_BIT );
        
        // Set the R,G,B cathode sinks to HIGH so no current flows (this will turn on pull-up until next step sets direction bit)..
        SBI( LED_R_PORT , LED_R_BIT );       // RED
        SBI( LED_G_PORT , LED_G_BIT );       // GREEN
        SBI( LED_B_PORT , LED_B_BIT );       // BLUE
                
        // Set the cathode sinks to output (they are HIGH from step above)
        // TODO: These will eventually be driven by timers
        SBI( LED_R_DDR , LED_R_BIT );       // RED
        SBI( LED_G_DDR , LED_G_BIT );       // GREEN
        SBI( LED_B_DDR , LED_B_BIT );       // BLUE
        
        SBI( BLUE_SINK_PORT , BLUE_SINK_BIT);   // Set the sink output high so blue LED will not come on
        SBI( BLUE_SINK_DDE  , BLUE_SINK_BIT);
        
}


// CLOCK CALCULATIONS
// Master clock is running at 1Mhz mostly to avoid FCC 15 issues. 
// Timer0 running with a /8 prescaller, so one PWM cycle takes ~2ms and full refresh takes ~12ms giving 81Hz refrsh


// Timers are hardwired to colors. No pin portable way to do this.
// RED   = OC0A
// GREEN = OC0B
// BLUE  = OC2B     
// 
// Blue is different
// =================
// Blue is not straight PWM since it is connected to a charge pump that charges on the + and activates LED on the 
// TODO: Replace diode with MOSFET, which will require an additional pin for drive

void setupTimers(void) {
    
    // ** First the main Timer0 to drive R & G. We also use the overflow to jump to the next multiplexed pixel 
    // Lets start with a prescaller of 8, which gives us a ~80hz refresh rate on the full 6 leds which should look smooth
    // TODO: How does frequency and duty relate to power efficiency? We can always lower to and trade resolution for faster cycles
    
    // We are running in FAST PWM mode where we continuously count up to TOP and then overflow. Not sure best TOP yet so try 255.
    // The outputs are HIGH at the begining and LOW at the end. HIGH turns OFF the LED and LEDs should be low duty cycle,
    // so this give us time to advance to the next pixel while LED is off to avoid visual glitching. 
    
    // Since we are using both outputs, I think we are stuck with Mode 3 = Fast PWM that does not let use use a different TOP
    // Mode 3 - Fast PWM TOP=0xFF, Update OCRX at BOTTOM, TOV set at MAX
        
    // Ok, here we go with obscure bit twiddling. There really should be a better way to do this...
    
    // First turn everything off so no glitch during setup
    
    // Writing OCR0A=MAX will result in a constantly high or low output (depending on the
    // polarity of the output set by the COM0A[1:0] bits.)
    // So setting OCR to MAX will turn off the LED because the output pin will be constantly HIGH
    
    
    // Timer0 (R,G)        
    OCR0A = 255;                            // Initial value for RED (off)
    OCR0B = 255;                            // Initial value for GREEN (off)
    TCNT0 = 255;                            // This will overflow immediately and set the outputs to 1 so LEDs are off.

    TCCR0A =
        _BV( WGM00 ) | _BV( WGM01 ) |       // Set mode=3 (0b11)
        _BV( COM0A1) |                      // Clear OC0A on Compare Match, set OC0A at BOTTOM, (non-inverting mode) (clearing turns LED on)
        _BV( COM0B1)                        // Clear OC0B on Compare Match, set OC0B at BOTTOM, (non-inverting mode)
    ;
           
    TCCR0B =                                // Turn on clk as soon as possible after setting COM bits to get the outputs into the right state
        _BV( CS01 );                        // clkI/O/8 (From prescaler)- This line also turns on the Timer0
    
    
    TIMSK0 = _BV( TOIE0 );                  // The corresponding interrupt is executed if an overflow in Timer/Counter0 occurs
                                            // We will jump to next pixel here


    // ** Next setup Timer2 for blue. This is different because for the charge pump. We have to drive the pin HIGH to charge
    // the capacitor, then the LED lights on the LOW.
    // So maybe the best way to handle this is to just always be charging except the very short times when we are ofF?
    // Normally this means the LED will be on dimly that while time, but we can compensate by only turn on the BOOST enable
    // pin when there is actually blue in that pixel right now, and maybe bump down the raw compare values to compensate for the
    // the leakage brightness when the battery voltage is high enough to cause some? Should work!

    
    // Timer2 (B)                           // Charge pump is attached to OC2B
    OCR2B = 255;                            // Initial value for BLUE (off)
    TCNT2= 255;                             // This will overflow immediately and set the outputs to 1 so LEDs are off.
    
    TCCR2A = 
        _BV( COM2B1) |                        // Set OC0A on Compare Match, clear OC0A at BOTTOM (inverting mode) (clearing turns off pump and on LED)
        _BV( WGM01) | _BV( WGM00)           // Mode 3 - Fast PWM TOP=0xFF
    ;
    
    TCCR2B =                                // Turn on clk as soon as possible after setting COM bits to get the outputs into the right state
        _BV(CS01);                        // clkI/O/8 (From prescaler)- This line also turns on the Timer0
    
    
    // TODO: Maybe use Timer2 to drive the ISR since it has Count To Top mode available. We could reset Timer0 from there.

 
           
}

// Note that LINE is 0-5 whereas the pixels are labeled p1-p6 on the board. 

void commonActivate( uint8_t line ) {         
    
    // TODO: These could probably be compressed with some bit hacking
    // TODO: Check if we really need dedicated two sides on IR LEDs and if not, use a full PORTC bitwalk to get rid of all this
    
    switch (line) {
        
        case 0:
            SBI( PIXEL1_PORT , PIXEL1_BIT );
            break;
        
        case 1:
            SBI( PIXEL2_PORT , PIXEL2_BIT );
            break;
        
        case 2:
            SBI( PIXEL3_PORT , PIXEL3_BIT );
            break;
            
        case 3:
            SBI( PIXEL4_PORT , PIXEL4_BIT );
            break;
        
        case 4:
            SBI( PIXEL5_PORT , PIXEL5_BIT );
            break;           

        case 5:
            SBI( PIXEL6_PORT  , PIXEL6_BIT );
            break;
        
    }
    
}

void commonDeactivate( uint8_t line ) {           // Also deactivates previous
    
        switch (line) {
            
            case 0:
            CBI( PIXEL1_PORT , PIXEL1_BIT );
            break;
            
            case 1:
            CBI( PIXEL2_PORT , PIXEL2_BIT );
            break;
            
            case 2:
            CBI( PIXEL3_PORT , PIXEL3_BIT );
            break;
            
            case 3:
            CBI( PIXEL4_PORT , PIXEL4_BIT );
            break;
            
            case 4:
            CBI( PIXEL5_PORT , PIXEL5_BIT );
            break;

            case 5:
            CBI( PIXEL6_PORT , PIXEL6_BIT );
            break;
            
        }

}

volatile

volatile uint8_t currentPixel;  // Which pixel is currently lit?
                                // Note that on startup ths is not technically true, so we will unnecessarily but benignly deactivate pixel 0
                                
// Called when Timer0 overflows, which happens at the end of the PWM cycle for each pixel. We advance to the next pixel.

ISR(TIMER0_OVF_vect)
{
    commonDeactivate( currentPixel );

    SBI( BLUE_SINK_PORT , BLUE_SINK_BIT);       // Faster to just blindly disable without even checking if it is currently on
    
    currentPixel++;                             // Scan across all 6 pixels in turn
    
    if (currentPixel==PIXEL_COUNT) {
        currentPixel=0;
    }
    
    if (rawValueB[currentPixel] != 255 ) {
        CBI( BLUE_SINK_PORT , BLUE_SINK_BIT );      // If the blue LED is on at all, then activate the boost. This might cuase the blue to come on slightly 
                                                    // if the battery voltage is high due to leakage, but that is ok because blue will be on anyway         
                                                    // We CBI here because this pin is a SINK so negative is active.                                            
    }
  
  
    /* 
    CBI( BLUE_SINK_PORT , BLUE_SINK_BIT );      // If the blue LED is on at all, then activate the boost. This might cuase the blue to come on slightly
              
    OCR0A = 255; //rawValueR[currentPixel];
    OCR0B = 255; //rawValueG[currentPixel];
    OCR2B = 150; //rawValueB[currentPixel];
    */
    
    OCR0A = rawValueR[currentPixel];
    OCR0B = rawValueG[currentPixel];
    OCR2B = rawValueB[currentPixel];
    
    commonActivate(currentPixel);
    // TODO: Probably a bit-wise more efficient way to do all this without a compare?  Only happens a few thousand times a second, so not *that* creitical... but still
    
}



// Timer1 for internal time keeping (mostly timing IR pulses) because it is 16 bit and its pins happen to fall on ports that are handy for other stuff
// Timer0 A=Red, B=Green. Both happen to be on handy pins
// Timer2B for Blue duty. Works out perfectly because we can use OCR2A as a variable TOP to change the frequency for the charge pump, which is better to change than duty. 

int main(void)
{
      
    setupPixelPins();
    
    for( uint8_t p=0; p<PIXEL_COUNT; p++ ) {
                
        rawValueR[p]=255;
        rawValueG[p]=255;
        rawValueB[p]=255;
        
    }
    
    
    setupTimers();
        
    sei();      // Let interrupts happen. For now, this is the timer overflow that updates to next pixel. 
    
    
    //while(1);
    
    while (1) {
        

        for( int b=200; b<256; b++ ) {
            
            for( uint8_t p=0; p<PIXEL_COUNT; p++ ) {
                
                rawValueR[p]=b;
            }                
            
            _delay_ms(10);
            
        }
        
        _delay_ms(100);
        
        
        for( int b=200; b<256; b++ ) {
            
            for( uint8_t p=0; p<PIXEL_COUNT; p++ ) {
                
                rawValueG[p]=b;
            }
            
            _delay_ms(10);
            
        }            
        
        _delay_ms(100);
        
        
        for( int b= 255-(56*2); b<256; b+=2 ) {
            
            for( uint8_t p=0; p<PIXEL_COUNT; p++ ) {
                
                rawValueB[p]=b;
            }
            
            _delay_ms(10);
            
        }
        
        _delay_ms(100);
        
        
        
    }
    
        
    /*
    set_sleep_mode( SLEEP_MODE_PWR_DOWN );
    cli();
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    // wake up here 
    sleep_disable();
    */
                
  
                            
}

