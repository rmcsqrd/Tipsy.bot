#include "SPI_Driver.h"
#include <stdint.h>

#define _XTAL_FREQ 24000000   //Required in XC8 for delays. 16 Mhz oscillator clock


// SPI bit bang stuff
uint8_t SPI_Transmit_Size = 8;              // global variable for 8bit/single byte R/W
char SPI_OutArray[] = {0,0,0,0,0,0,0,0};    //output array to write via SPI
char SPI_InArray[] = {0,0,0,0,0,0,0,0};


//// Bit-banged SPI////
// why did you do it this way? 
//      1) because hardware is for masochists and I've learned to love the pain
//      2) because the PIC18F2553 has issues with its SSPBUF I think OR my hardware is having an issue (probably the latter)

void SPIGenOutArray(uint8_t data){
    uint8_t cnt = 0;
    uint8_t mask = 0x80;         // b10000000
    while(cnt < SPI_Transmit_Size){
        SPI_OutArray[cnt] = (data << cnt) & mask;    // shift based on count and mask
        SPI_OutArray[cnt] = SPI_OutArray[cnt] >> 7;     // shift result back to LSB (either one or zero)
        cnt++;
    }
}

uint8_t SPIGenInArray(void){
    uint8_t result = 0x00;
    uint8_t cnt = 0;
    while(cnt < SPI_Transmit_Size){
        result = result | (SPI_InArray[cnt] << ((SPI_Transmit_Size - 1)-cnt));
        cnt++;
    }
    return result;
}

void SPITransmit(void){
    uint8_t cnt = 0;
    while(cnt < SPI_Transmit_Size){
        if(SPI_OutArray[cnt] == 1){LATCbits.LATC7 = 1;}  // write SDO low or high based on bit in array
        else{ LATCbits.LATC7 = 0;}
        LATBbits.LATB1 = 0;                              // drive SCK low
        __delay_us(1);
        if(PORTBbits.RB0 == 1){SPI_InArray[cnt] = 0x01;} // write status of SDI into array
        else{SPI_InArray[cnt] = 0x00;}
        LATBbits.LATB1 = 1;                              // drive SCK high
        __delay_us(1);
        cnt++;
    }
}