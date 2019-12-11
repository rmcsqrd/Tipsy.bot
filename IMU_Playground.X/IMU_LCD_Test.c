
/****** ASEN 5519 IMU_LCD_Test ******************************************************
 * Author: Rio McMahon
 * Created 11/26/19
 *
 * Test environment for IMU using LCD as output
 *  

 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Include/pragma
 * Global Variables
 * Function Prototypes
 * 
 * [function]                    [function descriptor]
 ******************************************************************************/

#include <xc.h>
#include <stdint.h>  
#include "LCDroutinesEasyPic.h"



#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF


/******************************************************************************
 * Global variables
 ******************************************************************************/
// IMU stuff
#define REG_BANK_SEL 0x7F;           // IMU REG_BANK_SEL at address 0x7F
#define WHO_AM_I 0x00;               // IMU WHO_AM_I at address 0x00
uint8_t IMU_BSreg0 = 0b00000000;     // IMU bank select register 0 = 0b--00 ----

//LCD Stuff
char LCDRow1[] = {0x80,' ',' ',' ',' ',' ',' ',' ',' ',0x00};     // IMU function title
char LCDRow2[] = {0xC0,' ',' ',' ',' ',' ',' ',' ',' ',0x00};     //IMU function output
        
       



/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
uint8_t IMU_whoami(void);       // read whoami reg from IMU via SPI
void writeIMU_BSR(uint8_t address, uint8_t data);
uint8_t readIMU(uint8_t address);


/******************************************************************************
 * main()
 ******************************************************************************/
void main(){
    Initial();
    uint8_t IMU_BSR_Reg = REG_BANK_SEL
    writeIMU_BSR(IMU_BSR_Reg, IMU_BSreg0);     // write to IMU bank select register, select bank 0
    while(1){
        
        uint8_t whoami = WHO_AM_I;
        uint8_t IMU_output = readIMU(whoami);
        
    }
    
    
    
    
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR0 and sets CCP1 for compare, and enables LoPri interrupts for
 * both.
 ******************************************************************************/
void Initial() {

    // initialize MSSP1
    SSP1CON1bits.SSPEN = 0;     // disable MSSP to config it
    TRISCbits.TRISC5 = 0;       // set SDO1 as output
    TRISCbits.TRISC4 = 1;       // set SDI1 as input
    TRISCbits.TRISC3 = 0;       // set SCK1 as output
    TRISCbits.TRISC2 = 0;       // set CS as output (drive low to transmit, drive high to receive)
    
    SSP1STATbits.SMP = 1;       // set sample bit = 1
    SSP1STATbits.CKE = 1;       // set clock select bit = 1
    SSP1CON1bits.CKP = 0;       // set clock polarity bit = 0 
    SSP1CON1bits.SSPM3 = 0;     // SSPM = 1010
    SSP1CON1bits.SSPM2 = 0;
    SSP1CON1bits.SSPM1 = 0;
    SSP1CON1bits.SSPM0 = 0;
    SSP1CON1bits.SSPEN = 1;     // reenable MSSP after config
    LATCbits.LATC2 = 1;         // write CS high to so transfer is not initiated until we want it to be 
    
    // Configure the LCD pins for output. Defined in LCDRoutines.h
    LCD_RS_TRIS   = 0;             
    LCD_E_TRIS    = 0;
    LCD_DATA_TRIS = 0b11000000;     // Note the LCD is only on the upper nibble
                                    // The lower nibble is all inputs
    LCD_DATA_LAT = 0;           // Initialize LCD data LAT to zero
    
    // Initialize the LCD and print to it
    InitLCD();                      
    DisplayV(LCDRow1);              
    DisplayV(LCDRow2);
    
}

uint8_t readIMU(uint8_t address){
    uint8_t readMask = 0b01111111;
    address = readMask & address;   // mask readMask with address so MSB = 0 for read

    LATCbits.LATC2 = 0;             // write CS low to initiate transfer
    uint8_t IMU_output = SSP1BUF;      // read garbage from SSP buffer
    
    SSP1BUF = address;              // write address to read from
    while(SSP1STATbits.BF == 0){}   // sit tight until receive complete
    IMU_output = SSP1BUF;           // read output from SSP buffer
    SSP1BUF = 0b00000000;           // write garbage to buffer
    while(SSP1STATbits.BF == 0){}   // sit tight until receive complete
    IMU_output = SSP1BUF;              // read garbage from SSP buffer
    LATCbits.LATC2 = 1;             // write CS high to stop transfer
    return IMU_output;
}

void writeIMU_BSR(uint8_t address, uint8_t data){
    uint8_t writeMask = 0b10000000;      // MSB = 1 for address write
    address = writeMask | address;       // mask write command with address
    
    LATCbits.LATC2 = 0;             // write CS low to initiate transfer
    uint8_t garbage = SSP1BUF;      // read garbage from SSP buffer
    SSP1BUF = address;              // write address into SSPbuffer
    while(SSP1STATbits.BF == 0){}   // sit tight until receive complete
    garbage = SSP1BUF;
    SSP1BUF = data;                 // write data into SSPbuffer
    while(SSP1STATbits.BF == 0){}   // sit tight until receive complete
    garbage = SSP1BUF;              // read garbage from SSP buffer
    LATCbits.LATC2 = 1;             // write CS high to end transfer
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
  
}	

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR0IF and CCP1IF are clear.
 ******************************************************************************/

void __interrupt(low_priority) LoPriISR(void) {
    
}

