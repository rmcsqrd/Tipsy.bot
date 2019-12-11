
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
#include <pic18f2553.h>
#include "MotorDriver.h"

#define _XTAL_FREQ 24000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS
#pragma PWRTEN=ON, BOREN=ON, BORV=1, CCP2MX=PORTC, XINST=OFF
#pragma config WDT=OFF        // clear this to turn off watchdog timer
#pragma config LVP=OFF        // clear this, allows debug to occur

/******************************************************************************
 * Global variables
 ******************************************************************************/
char  xtest      = 250;
char  temptest[] = "Accel1";
int   ytest      = -2;
int  *ztest      = &ytest;
char *atest;


char Motor_PWM_status = 0;          // status of the motor PWM 
unsigned short Motor_PWM_cnt = 0;   // counter for motor PWM signal on TMR0
unsigned short PWM_max = 0;         // what motor_PWM_cnt counts up to, changes based on Motor_PWM_status
unsigned short motor_speed = 0;     // speed (0-65535) of motor
unsigned char motor_orientation = 0;// orientation (0=CW, 1=CCW)
unsigned long temp = 0;                  // temp counter for blink alive
unsigned char cnt = 0;



/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void TMR0handler(void);     // function for lowpri interrupts
void BlinkAlive(void);      // blink LED's forever so I know the board is doing something

/******************************************************************************
 * main()
 ******************************************************************************/
void main(){
    Initial();
      while(1) {                   // do this loop forever
          motor_speed = 200;
          motor_orientation = 0;
          MotorDriver(motor_speed, motor_orientation);  // takes two inputs (speed (0-255) and orientation (0=CW, 1=CCW) )

          BlinkAlive();
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
//    initialize motor driver bits as outputs
    _MD_STBY_TRIS = 0;    
    _MD_AI1_TRIS = 0;     
    _MD_AI2_TRIS = 0;     
    _MD_PWMA_TRIS = 0; 
        
//    initialize blinkalive LEDs
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    
    TRISBbits.TRISB6 = 0;  // drive these pins low because rat tail programming line leads were causing grounding issues
    TRISBbits.TRISB7 = 0;
    
    
//    set up interrupts to handle PWM generation
    
//    initialize TMR0
    T0CONbits.TMR0ON = 0;   //keep timer off during init
    T0CONbits.T08BIT = 1;   //8-bit
    T0CONbits.T0CS = 0;     //internal clock
    T0CONbits.T0SE = 0;     // does something, don't care
    T0CONbits.PSA = 1;      // no prescaler, other prescale bits don't matter
    TMR0L = 0;              // Clearing TMR0 registers
    TMR0H = 0;
    
//    initialize interrupts for TMR0
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt
    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts
    
    
    T0CONbits.TMR0ON = 1;           // Turning on TMR0
    
}


void BlinkAlive(){
    // lazily blink onboard LED's forever so I know the MPU is doing something
    // count up to some arbitrary value and flip outputs to LEDs
    if(LATCbits.LATC0 == 0 && temp == 19999){
            LATCbits.LATC0 = 1;
            LATCbits.LATC1 = 0;
            temp = 0;
        }
    if(LATCbits.LATC0 != 0  && temp == 19999){
            LATCbits.LATC0 = 0;
            LATCbits.LATC1 = 1;
            temp = 0;
        }
    
    // increment up counter
    temp++;
    
}
    
void MotorDriver(unsigned char speed, unsigned char orientation){

    // update value of the PWMA pin on motor driver to match the PWM status on TMR0
    _MD_PWMA = Motor_PWM_status;
    
    // check if speed = 0, if yes set standby bit low so motors don't operate
    if(speed == 0){
        _MD_STBY = 0;
    }
    
        else{
            _MD_STBY = 1;
    }
    
    // check if orientation = 1 or 0, adjust input pins accordingly
    if(orientation == 0){
        _MD_AI1 = 0;
        _MD_AI2 = 1;
    }
    
    if(orientation == 1){
        _MD_AI1 = 1;
        _MD_AI2 = 0;
    }

    
    
    
    
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
    while(1) {
        
            // check if TMR0 interrupt was thrown
            if( INTCONbits.TMR0IF ) {
                TMR0handler();
                continue;
            }
            break;
        }
}


/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter
 ******************************************************************************/
void TMR0handler() {
    
    // PWM wave generator for DC motor
    if(Motor_PWM_cnt < PWM_max){Motor_PWM_cnt++;}
        else if(Motor_PWM_status == 0){
            PWM_max = motor_speed;
            Motor_PWM_status = 1;
            Motor_PWM_cnt = 0;
        }
        else if(Motor_PWM_status == 1){
            PWM_max = 255 - motor_speed;
            Motor_PWM_status = 0;
            Motor_PWM_cnt = 0;
        }
    // clear the timer
    INTCONbits.TMR0IF = 0;
    }
    
