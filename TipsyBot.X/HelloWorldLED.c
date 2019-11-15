
/****** ASEN 5519 Lab 6 ******************************************************
 * Author: Rio McMahon
 * Created 11/14/19
 *
 * Pragma and high level definitions modified from ASEN5519 lab 6 source code
 * 
 * Description
 * [include description]
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

#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS
#pragma PWRTEN=ON, BOREN=ON, BORV=1, WDTEN=OFF, CCP2MX=PORTC, XINST=OFF


/******************************************************************************
 * Global variables
 ******************************************************************************/
unsigned long temp;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts

/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
    Initial();
    
//    Blink onboard LED's forever so I know the MPU is doing something
      while(1) {                   // do this loop forever
          LATCbits.LATC0 = 0;
          LATCbits.LATC1 = 1;
//          LATAbits.LATA3 = 1;  // use this to validate the logic analyzer
          temp = 0;
          while(temp < 99999){
              temp++;
          }
          LATCbits.LATC0 = 1;
          LATCbits.LATC1 = 0;
//          LATAbits.LATA3 = 0;
          temp = 0;
          while(temp < 99999){
              temp++;
          }
//          __delays_ms(500);
//          LATCbits.LATC0 = 0;
//          LATCbits.LATC1 = 1;
//          __delays_ms(500);
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
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
//    TRISAbits.TRISA3 = 0;
    
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


/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter
 ******************************************************************************/
