
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
#include <stdint.h>
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

// Motor Driver Variables
char Motor_PWM_status = 0;          // status of the motor PWM 
unsigned short Motor_PWM_cnt = 0;   // counter for motor PWM signal on TMR0
unsigned short PWM_max = 0;         // what motor_PWM_cnt counts up to, changes based on Motor_PWM_status
unsigned short motor_speed = 0;     // speed (0-65535) of motor
unsigned char motor_orientation = 0;// orientation (0=CW, 1=CCW)


// Blink Alive Variables
unsigned long temp = 0;                  // temp counter for blink alive

// IMU variables
uint8_t readMask = 0b10000000;      // mask for addresses (MSB=1 for Read per IMU documentation)
uint8_t writeMask = 0b00000000;     // mask for addresses (MSB=0 for Read per IMU documentation)
uint8_t IMU_identity = 0;           // IMU identity from calling IMU_whoami. Should always be 0xEA if IMU SPI is working
uint8_t IMU_read_garbage = 0;       // write only dump variable for IMU SPI communication, should never be read from because contents is variable
uint8_t IMU_output = 0;             // output variable for IMU responses

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);                 // Function to initialize hardware and interrupts

// Motor Driver Functions
void TMR0handler(void);             // function for lowpri interrupts

// Blink Alive Functions
void BlinkAlive(void);              // blink LED's forever so I know the board is doing something

// IMU Functions
uint8_t WriteIMU(uint8_t address, uint8_t command); // master function to reading/writing IMU
uint8_t ReadIMU(uint8_t address);   // master function for reading IMU by address. Returns output from IMU
void IMU_whoami(void);              // function to verify IMU is working properly
void IMU_BSR0_Select(void);         // function to force select user bank 0 on IMU

/******************************************************************************
 * main()
 ******************************************************************************/
void main(){
    Initial();
//    IMU_BSR0_Select();             // select IMU User Bank Select 0 via SPI comm
      while(1) {                   // do this loop forever
          motor_speed = 200;
          motor_orientation = 0;
          MotorDriver(motor_speed, motor_orientation);  // takes two inputs (speed (0-255) and orientation (0=CW, 1=CCW) )

          BlinkAlive();
          IMU_whoami();             // check on identity of IMU
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
        
    
// initialize IMU SPI registers   
    
    SSPCON1bits.SSPEN = 0;     // disable MSSP to config it
    TRISCbits.TRISC7 = 0;       // set SDO as output
    TRISBbits.TRISB0 = 1;       // set SDI as input
    TRISBbits.TRISB1 = 0;       // set SCK as output

    
    SSPSTATbits.SMP = 1;    // input sampled at end of output time (tuning knob)
    SSPSTATbits.CKE = 1;    // transmit occurs on transfer from active to idle clock state
    SSPCON1bits.CKP = 1;    // clock idle is high state (tuning knob)
    SSPCON1bits.SSPM3 = 0;  // SSPM = 0000
    SSPCON1bits.SSPM2 = 0;
    SSPCON1bits.SSPM1 = 0;
    SSPCON1bits.SSPM0 = 1;
    TRISBbits.TRISB2 = 0;    // use RB2 as CS for SPI, set as output
    LATBbits.LATB2 = 1;     // write RB2 high until SPI communication initiated
    SSPCON1bits.SSPEN = 1;  // enable MMSP
    


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

void IMU_whoami(){
//    test function to verify SPI communication is working properly. Should return 0xEA
    uint8_t address = 0x00;
    IMU_identity = ReadIMU(address);
    
}

void IMU_BSR0_Select(){
//    function to select User Bank 0 as working IMU bank in initialization routine
    uint8_t address = 0x7F;
    uint8_t command = 0x00;
    IMU_read_garbage = WriteIMU(address, command);
}

uint8_t WriteIMU(uint8_t address, uint8_t command){
    address = readMask & address;   // mask address and command with Read and Write bits respectively
    command = writeMask & command;
    
    LATBbits.LATB4 = 0;             // write CS low to initiate transfer
    IMU_read_garbage = SSPBUF;     // read garbage from SSP buffer to clear it
    SSPBUF = address;              // write address to send command to
    while(SSPSTATbits.BF == 0){}   // sit tight until receive complete
    IMU_read_garbage = SSPBUF;     // read garbage from SSP buffer
    
    SSPBUF = command;              // write command to send
    while(SSPSTATbits.BF == 0){}   // sit tight until receive complete
    IMU_output = SSPBUF;     // read garbage from SSP buffer
    LATBbits.LATB2 = 1;             // drive CS high to end transfer
    return IMU_output;              // return IMU response after data word transmit (not sure what this should be)
}

uint8_t ReadIMU(uint8_t address){
    address = readMask | address;   // mask readMask with address so MSB = 1 for read
    
    LATBbits.LATB2 = 1;             // write CS low to initiate transfer
//    IMU_read_garbage = SSPBUF;      // read garbage from SSP buffer
    SSPBUF = address;               // write address to read from
    while(SSPSTATbits.BF == 0){}    // sit tight until receive complete
    IMU_output = SSPBUF;            // read output from SSP buffer
    SSPBUF = 0x00;               // write address to read from
    while(SSPSTATbits.BF == 0){}    // sit tight until receive complete
    IMU_output = SSPBUF;            // read output from SSP buffer
    LATBbits.LATB2 = 0;             // write CS high to stop transfer
    return IMU_output;
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
    
