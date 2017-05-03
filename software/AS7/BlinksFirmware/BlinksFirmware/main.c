/*
 * BlinksFirmware.c
 *
 * Created: 3/26/2017 8:39:50 PM
 * Author : josh.com
 */ 

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>           // PROGMEM to keep data in flash

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

#define PIXEL2_PORT PORTD
#define PIXEL2_DDR  DDRD
#define PIXEL2_BIT  0

#define PIXEL3_PORT PORTB
#define PIXEL3_DDR  DDRB
#define PIXEL3_BIT  7

#define PIXEL4_PORT PORTD
#define PIXEL4_DDR  DDRD
#define PIXEL4_BIT  1

#define PIXEL5_PORT PORTD
#define PIXEL5_DDR  DDRD
#define PIXEL5_BIT  4

#define PIXEL6_PORT PORTD
#define PIXEL6_DDR  DDRD
#define PIXEL6_BIT  2

#define PIXEL_COUNT 6

// Here are the raw compare register values for each pixel
// These are precomputed from brightness values because we read them often from inside an ISR
// Note that for reg & green, 255 corresponds to OFF and 250 is about maximum prudent brightness. 
volatile uint8_t rawValueR[PIXEL_COUNT];
volatile uint8_t rawValueG[PIXEL_COUNT];
volatile uint8_t rawValueB[PIXEL_COUNT];

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
// We drive this HIGH to turn off blue, otherwise blue led could 
// come on if the battery voltage is high enough to overcome the forward drop on the
// blue LED + Schottky

#define BLUE_SINK_PORT PORTE
#define BLUE_SINK_DDE  DDRE
#define BLUE_SINK_BIT  3



#define BUTTON_PORT    PORTD
#define BUTTON_PIN     PIND
#define BUTTON_BIT     7

#define BUTTON_PCI     PCIE2
#define BUTTON_ISR     PCINT2_vect
#define BUTTON_MASK    PCMSK2
#define BUTTON_PCINT   PCINT23

#define BUTTON_DOWN() (TBI(BUTTON_PIN,BUTTON_BIT))           // PCINT23

#define BUTTON_DEBOUNCE_TICKS 10

ISR(BUTTON_ISR)
{
    //code
}


void setupButton(void) {
    
    // GPIO setup
    SBI( BUTTON_PORT , BUTTON_BIT);     // Leave in input mode, enable pull-up
    
    // Pin change interrupt setup
    SBI( BUTTON_MASK , BUTTON_PCINT);   // Enable pin in Pin Change Mask Register 
    SBI( PCICR , BUTTON_PCI );          // Enable the pin group
    
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

volatile uint8_t verticalRetraceFlag=0;     // Turns to 1 when we are about to start a new refresh cycle at pixel zero
                                            // Once this turns to 1, you have about 2ms to load new values into the raw array   
                                            // to have them displayed in the next frame.
                                            // Only matters if you want to have consistent frames and avoid visual tearing
                                            // which might not even matter for this application at 80hz
                                            // TODO: Is this empirically necessary?
                                   

volatile uint8_t previousPixel;     // Which pixel was lit on last pass?
                                    // Note that on startup this is not technically true, so we will unnecessarily but benignly deactivate pixel 0
                                
// Called when Timer0 overflows, which happens at the end of the PWM cycle for each pixel. We advance to the next pixel.

// Non-intuitive sequencing!
// Because the timer only latches the values in the OCR registers when at the moment this ISR fires, by the time we are running
// it is already lateched the *previous* values and they are currently being used. That means that right now we need to...
//
// 1. Activate the common line for the values that were previously latched.
// 2. Load the values into OCRs to be latched when this cycle completes.
//
// You'd think we could just offset the raw values by one, but that doesn't work because the boost enable must match 
// the values currently being displayed. 
//
// Note that we have plenty of time to do stuff once the boost enable is updated for the
// values for the currently displayed pixel (the last loaded OCR values), because we have arranged things so that LEDs
// are always *off* for the 1st half of the cycle. 


ISR(TIMER0_OVF_vect)
{
    
    commonDeactivate( previousPixel );

    SBI( BLUE_SINK_PORT , BLUE_SINK_BIT);       // Faster to just blindly disable without even checking if it is currently on
                                                // Remember, this is a SINK so setting HIGH disables it.

    uint8_t currentPixel = previousPixel+1;
        
    if (currentPixel==PIXEL_COUNT) {
        currentPixel=0;
    }
    
    if (rawValueB[currentPixel] != 255 ) {
        CBI( BLUE_SINK_PORT , BLUE_SINK_BIT );      // If the blue LED is on at all, then activate the boost. This will start charging the boost capactior. 
                                                    // This might cause the blue to come on slightly if the boost capacitor is full
                                                    // if the battery voltage is high due to leakage, but that is ok because blue will be on anyway         
                                                    // We CBI here because this pin is a SINK so negative is active.                                            
    }
    
    commonActivate(currentPixel);
    
    // Ok, current pixel is now ready to display when the OCRs match the timer durring this pass.
    
    // Next we have to set up the OCR values that will get latched when this pass overflows...
    
    uint8_t nextPixel = currentPixel+1;    
  
    if (nextPixel==PIXEL_COUNT) {
        nextPixel=0;
    }
    
    if (nextPixel==PIXEL_COUNT-1) {     // If we are now loading the the last pixel, then we start a new frame on the next pass.
                                        // Note that this ISR can not be interrupted, so no risk of user updating RAW while we are reading them,
                                        // that can only happen after we return.
        
        verticalRetraceFlag = 1;
    }
  
    /* 
    CBI( BLUE_SINK_PORT , BLUE_SINK_BIT );      // If the blue LED is on at all, then activate the boost. This might cuase the blue to come on slightly
              
    OCR0A = 255; //rawValueR[currentPixel];
    OCR0B = 255; //rawValueG[currentPixel];
    OCR2B = 150; //rawValueB[currentPixel];
    */
    
    // Get ready for next pass
    
    // Remember that these values will not actually get loaded into the timer until it overflows
    // after it has finished displaying the current values
    
    OCR0A = rawValueR[nextPixel];
    OCR0B = rawValueG[nextPixel];
    OCR2B = rawValueB[nextPixel];
    
    previousPixel = currentPixel;
    
    // TODO: Probably a bit-wise more efficient way to do all this incrementing without a compare/jmp?  Only a couple of cycles and only few thousand times a second, so why does it bother me so?
    
}

// Gamma table curtsey of adafruit...
//https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix
// TODO: Compress this down, we probably only need like 4 bits of resoltuion. 

const uint8_t PROGMEM gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
    10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
    17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
    25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
    37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
    51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
    69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
    90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
    115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
    144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
    177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
    215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

// Set a single pixel's RGB value
// Normalized and balanced
// 0=off, 255=full brightness
// Note that there will likely be fewer than 256 actual visible values, but the mapping will be linear and smooth

// TODO: Balance, normalize, power optimize, and gamma correct these functions
// Need some exponential compression at the top here
// Maybe look up tables to make all calculations be one step at the cost of memory?

inline static void setPixelRGB( uint8_t p, uint8_t r, uint8_t g, uint8_t b ) { 
    
    // These are just guesstimates that seems to look ok. 
    
    rawValueR[p] = 255- (pgm_read_byte(&gamma8[r])/4);
    rawValueG[p] = 255- (pgm_read_byte(&gamma8[g])/4);
    rawValueB[p] = 255 -(pgm_read_byte(&gamma8[b])/4);
            
}

// Set the color of all pixels to one value

void setRGB( uint8_t r, uint8_t g, uint8_t b ) {

    // TODO: Optimize to avoid recalculating transfer function for every pixel

    for( uint8_t p=0; p<PIXEL_COUNT; p++ ) {
        setPixelRGB( p , r , g , b );
        
    }
    
}

// Use ADC6 (pin 19) for an analog input- mostly for dev work now


uint8_t analogRead(void) {
    ADMUX = 
        _BV(REFS0)   |                  // Refernce AVcc voltage
        _BV( ADLAR ) |                  // Left adjust result so only one 8 bit read of the high register needed
        _BV( MUX2 )  | _BV( MUX1 )      // Select ADC6
     ;
       
     ADCSRA = 
        _BV( ADEN )  |                  // Enable ADC
        _BV( ADSC )                     // Start a conversion
     ;
     
     
      while (TBI(ADCSRA,ADSC)) ;       // Wait for conversion to complete
      
      return( ADCH );
    
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
    
    
    while(1) {
        
        setRGB( 0 , 0 , analogRead() );
        
        _delay_ms(10);
        
    }
    
    while (1) {
        

        for( int b=0; b<255; b+=3) {
                           
            setRGB( b , 0 , 0);
                
            _delay_ms(10);
            
        }
        
        _delay_ms(100);
        
        for( int b=0; b<255; b+=3 ) {
            
            setRGB( 0 , b , 0);
            
            _delay_ms(10);
            
        }
        
        _delay_ms(100);
        
        for( int b=0; b<255; b+=1 ) {
            
            setRGB( 0 , 0 ,  b);
            
            _delay_ms(10);
            
        }
        
        _delay_ms(100);
        
        
        for( uint8_t i=0;i<20;i++) {                       // DO it 10 times
            
            for( uint8_t s=0; s<20; s++) {                 // step animation 10 frames per pixel 
                
                uint8_t b = s * ( 256/20) ;                             // (so we are just generating the brightness for pixel 0)
                        
                for( uint8_t p=0; p<PIXEL_COUNT;p++) {      // Set value each pixel
                
                    setPixelRGB( p , 0 , b ,  0);
                    
                    b += (256/PIXEL_COUNT) ;                          // everything just works naturally via overflow rollover)
                    
                }                    
            
                _delay_ms(10);
                
            }                
            
        }
        
        
        
        
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

