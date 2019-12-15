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

uint8_t WriteIMU(uint8_t address, uint8_t command, uint8_t reg){   
    INTCONbits.GIEL = 0;            // Disable low-priority interrupts to CPU
    INTCONbits.GIEH = 0;            // Disable all interrupts    
    
    
    if (reg == 0){                  // 0 = accel
        LATBbits.LATB4 = 0;             // write CS low to initiate transfer
    }
    if(reg == 1){                   // 1 = mag
        LATBbits.LATB3 = 0;             // write CS low to initiate transfer
    }
    __delay_us(3);                  // wait a little bit
    SPIGenOutArray(address);        // generate the address transmit array
    SPITransmit();                  // transmit the address
    LATCbits.LATC7 = 1;             // drive SDO high between address and command transmit
    __delay_us(3);                  // wait a little bit
    SPIGenOutArray(command);        // generate the command transmit array
    SPITransmit();                  // transmit the command
    __delay_us(3);                  // wait a little bit
    LATCbits.LATC7 = 0;             // drive the SDO line low

    LATBbits.LATB4 = 1;             // drive CS high to end transfer
    LATBbits.LATB3 = 1;             // drive CS high to end transfer

    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts
    
    uint8_t results = SPIGenInArray();  // process results from SPI
    return results;              // return IMU response after data word transmit (not sure what this should be)
    
}

uint8_t ReadIMU(uint8_t address, uint8_t reg){             // syntax reformat lightly inspired by https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/examples/MPU9250_Debug/MPU9250_Debug.ino
    uint8_t readMask = 0b10000000;      // mask for addresses (MSB=1 for Read per IMU documentation)
    return WriteIMU(address | readMask, 0x00, reg); // mask address with read mask. Give garbage data to write
}    