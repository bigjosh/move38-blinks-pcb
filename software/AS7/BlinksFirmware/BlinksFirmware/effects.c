/*
 * effects.c
 *
 * Created: 8/13/2017 8:21:22 PM
 *  Author: passp
 */ 

/*


void rainbowBreathingEffect(void) {

    while (!effectReturnFlag) {
        

        for( int b=0; b<255 && !effectReturnFlag; b+=3) {
            
            setRGB( b , 0 , 0);
            
            _delay_ms(10);
            
        }
        
        for( int b=255; b>0 && !effectReturnFlag ; b-=3) {
            
            setRGB( b , 0 , 0);
            
            _delay_ms(10);
            
        }
        
        
        delayWithReturnFlag(100);
        
        
        
        for( int b=0; b<255 && !effectReturnFlag ; b+=3 ) {
            
            setRGB( 0 , b , 0);
            
            _delay_ms(10);
            
        }
        
        for( int b=255; b>0 && !effectReturnFlag; b-=3) {
            
            setRGB( 0 , b , 0);
            
            _delay_ms(10);
            
        }
        
        
        delayWithReturnFlag(100);
        
        
        for( int b=0; b<255 && !effectReturnFlag; b+=3 ) {
            
            setRGB( 0 , 0 ,  b);
            
            _delay_ms(10);
            
        }
        
        for( int b=255; b>0 && !effectReturnFlag; b-=3) {
            
            setRGB(  0 , 0 , b);
            
            _delay_ms(10);
            
        }
        
        delayWithReturnFlag(100);
        
    }
    
}


void rainbowFadeEffect(void) {
    

    while (!effectReturnFlag) {
        
        
        for( int h=0; h<255 && !effectReturnFlag; h++ ) {     // Fade hue steps
            
            //            uint8_t h = (a + (( 256 * p)/PIXEL_COUNT)) & 255;

            for( uint8_t p=0; p<PIXEL_COUNT ;p++) {      // Set value each pixel

                setPixelHSB( p ,  h , 255 , 200 );
                
            }
            
            delayWithReturnFlag(50);
            
            
        }
        
    }
    
}


void RedSpinnerEffect(void) {
    
    while (!effectReturnFlag) {
        
        for( uint8_t i=0;i<20 && !effectReturnFlag ;i++) {                       // DO it 10 times per color
            
            for( uint8_t s=0; s<20 && !effectReturnFlag ; s++) {                 // step animation 10 frames per pixel
                
                uint8_t b = s * ( 256/20) ;                // (so we are just generating the brightness for pixel 0)
                
                for( uint8_t p=0; p<PIXEL_COUNT;p++) {      // Set value each pixel
                    
                    setPixelRGB( p , b ,  0 , 0 );
                    
                    b += (256/PIXEL_COUNT) ;                          // everything just works naturally via overflow rollover)
                    
                }
                
                _delay_ms(10);
                
            }
        }
    }
    
}




void rainbowSpinnerEffect(void) {
    

    while (!effectReturnFlag) {
        
        
        for( int a=0; a<255 && !effectReturnFlag; a++ ) {     // Angle steps

            for( uint8_t p=0; p<PIXEL_COUNT ;p++) {      // Set value each pixel

                uint8_t h = (a + (( 256 * p)/PIXEL_COUNT)) & 255;

                setPixelHSB( p ,  h , 255 , 180 );
                
            }
            
            _delay_ms(5);
            
        }
        
    }
    
}

#define PI 3.15152

void blueWaveEffect(void) {
    
    // Rotating blue wave effect

    float rotatedAngle=0;             // Current rotated angle

    const int waveSteps = 100;        // Number of steps to take when passing the wave acorss the board for each angle step.

    const int cycle_count = 10;      // Number of waves to cycle to get all the way round

    while (!effectReturnFlag) {

        for( int w=0; w<waveSteps && !effectReturnFlag; w++) {     // Steps in each Wave cycle

            // Imagine the wave is always going to left to right (across X values), wavelength is 2 units. The pixles are all 1 unit from the center.
            
            for( int p=0; p<PIXEL_COUNT; p++ ) {

                // Represent the pixels in polar cooridinates at radius 1
                // Rotate the pixel into position on the board, plus the addisional angle for the wave rotation

                // Compute the x position of the pixel Relative to the incoming wave (it is coming left to right on the x axis)
                // This x will be -1 to 1
                
                float x= sin(
                
                2.0 * PI * (
                
                (( (float) p ) /PIXEL_COUNT)          // The angle of the chip around the board - when CHIP_COUNT is 6, then this ends up being every 30 degrees
                
                )

                +rotatedAngle                        // add in the dynamic rotating angle

                );

                // Next compute the intensity of the wave at this X location (y doesn't matter since this is a planar wavefront)
                
                uint8_t b=  (
                
                -1.0 * cos(
                
                (
                2* PI *     // flip the COS so we start a -1 rather than 1 (we will later adjust up 1 so we start a 0 and peak at 2)

                ( ((float) w )/ waveSteps )       // This is the actual moving wave value at x=0

                ) +x

                )

                +1                                      // Normalize from [-1 to 1] to [0 to 2].
                
                ) * 100.0;   // Bring into our 8-bit color space, brightness down a bit

                setPixelRGB( p , 0 , 0 , b );
                
            }


            delayWithReturnFlag(10);

            rotatedAngle += ( 2.0 * PI ) / ( 1.0 * waveSteps * cycle_count);
            
        }

    }
    
}


void discoMode(void) {
    
    while (!effectReturnFlag) {
        
        uint8_t h = rand() & 0xff;
        
        uint8_t t = (rand() & 0x0f) * 5;
        
        uint8_t p = rand() % 6;
        
        setPixelHSB( p , h , 255 , 200 );
        
        delayWithReturnFlag( t );
        
        setPixelRGB( p , 0 , 0 , 0 );
        
    }
    
}



#define EFFECT_COUNT 6          // Max is 6 since we only have 6 pixels to show the selection on.

uint8_t currentEffect = 0 ;

// Handle button down event, show something nice while waiting for button to go up again

void showEffects() {
    
    while (1) {
        
        effectReturnFlag=0;             // Next button down will set this
        
        // Show current effect
        
        switch (currentEffect) {
            
            case 0:
            rainbowBreathingEffect();
            break;
            
            case 1:
            RedSpinnerEffect();
            break;
            
            case 2:
            blueWaveEffect();
            break;
            
            case 3:
            rainbowSpinnerEffect();
            break;
            
            case 4:
            discoMode();
            break;
            
            case 5:
            rainbowFadeEffect();
            break;
            
            
        }
        
        // If we get here, then the button was pressed and effectReturnFlag was set in the button ISR
        
        // Step the next effect

        currentEffect++;
        
        if (currentEffect==EFFECT_COUNT) {
            currentEffect=0;
        }
        
        // Show a nice flasher while we wait for button up
        
        do {
            
            setPixelRGB( 0 , 0 , 255 , 0 );
            setPixelRGB( 1 , 0 ,   0 , 0 );
            setPixelRGB( 2 , 0 , 255 , 0 );
            setPixelRGB( 3 , 0 ,   0 , 0 );
            setPixelRGB( 4 , 0 , 255 , 0 );
            setPixelRGB( 5 , 0 ,   0 , 0 );
            
            _delay_ms(20);      // Debounce on the way down
            
            setPixelRGB( 0 , 0 ,   0 , 0 );
            setPixelRGB( 1 , 0 , 255 , 0 );
            setPixelRGB( 2 , 0 ,   0 , 0 );
            setPixelRGB( 3 , 0 , 255 , 0 );
            setPixelRGB( 4 , 0 ,   0 , 0 );
            setPixelRGB( 5 , 0 , 255 , 0 );
            
            _delay_ms(20);
            
            
        } while (BUTTON_DOWN());
        
        setRGB( 0, 0, 0 );
        
        _delay_ms(20);      // debounce on the way up
        
    }
}

*/