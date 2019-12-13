
/****** ASEN 5519 Final Project ******************************************************
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
#pragma config PBADEN=OFF     // clear 

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
uint8_t IMU_identity = 0;           // IMU identity from calling IMU_whoami. Should always be 0xEA if IMU SPI is working
uint8_t IMU_read_garbage = 0;       // write only dump variable for IMU SPI communication, should never be read from because contents is variable
uint8_t IMU_output = 0;             // output variable for IMU responses
uint8_t AccelXLow = 0;              // variable for accelerometer output X, low
uint8_t AccelXHigh = 0;             // variable for accelerometer output X, high
uint16_t AccelX = 0;                // variable for accelerometer output X, combined
uint16_t Gyro = 0;

// SPI bit bang stuff
uint8_t SPI_Transmit_Size = 8;              // global variable for 8bit/single byte R/W
char SPI_OutArray[] = {0,0,0,0,0,0,0,0};    //output array to write via SPI
char SPI_InArray[] = {0,0,0,0,0,0,0,0};


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);                 // Function to initialize hardware and interrupts

// Motor Driver Functions
void TMR0handler(void);             // function for lowpri interrupts

// Blink Alive Functions
void BlinkAlive(void);              // blink LED's forever so I know the board is doing something

// IMU Functions
uint8_t WriteIMU(uint8_t address, uint8_t command, uint8_t reg); // master function to reading/writing IMU
uint8_t ReadIMU(uint8_t address, uint8_t reg);   // master function for reading IMU by address. Returns output from IMU
void IMU_whoami(void);              // function to verify IMU is working properly
void IMU_BSR0_Select(void);         // function to force select user bank 0 on IMU
void IMU_AccelX(void);               // function to read accelerometer
void IMU_gyro(void);                // funciton to read gyro
void SPIGenOutArray(uint8_t data); // function to convert address into array of ones and zeros
uint8_t SPIGenInArray(void);        // function to convert data recieved from SPI into hex value
void SPITransmit(void);     // function to transmit SPI data

/******************************************************************************
 * main()
 ******************************************************************************/
void main(){
    Initial();
    IMU_whoami();

      while(1) {                   // do this loop forever
//          IMU_AccelX();
//          ReadIMU(0x22, 1);
          IMU_whoami();
          motor_speed = 200;
          motor_orientation = 0;
          MotorDriver(motor_speed, motor_orientation);  // takes two inputs (speed (0-255) and orientation (0=CW, 1=CCW) )
//         __delay_ms(500);
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
//    Set up ADC stuff to be all digital because it is interacting negatively with MSSP
    ADCON0 = 0x00;
    ADCON1 = 0x0F;
    ADCON2 = 0x00;
    
// initialize IMU SPI registers   
    TRISCbits.TRISC7 = 0;       // set SDO as output
    TRISBbits.TRISB0 = 1;       // set SDI as input  DO NOT CHANGE THIS!!!!
    TRISBbits.TRISB1 = 0;       // set SCK as output
    TRISBbits.TRISB4 = 0;    // use RB2 as CS_AG for SPI, set as output
    TRISBbits.TRISB3 = 0;   // use RB3 as CS_M, set as output
    
    LATBbits.LATB4 = 1;     // write RB2 high until SPI communication initiated
    LATBbits.LATB3 = 1;     // write RB3 high until SPI communication initiated
    LATCbits.LATC7 = 0;     // MOSI idle state is low
    LATBbits.LATB1 = 1;     // SCK idle state is high
    
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
    
//    initialize IMU
    
    
    T0CONbits.TMR0ON = 1;           // Turning on TMR0
   
    
}

void IMU_whoami(){
//    test function to verify SPI communication is working properly. Should return 0xEA
    uint8_t IdentityAddress = 0x0F;
    IMU_identity = ReadIMU(IdentityAddress, 0);
}

void IMU_AccelZ(){
    uint8_t AccelAddressH = 0x2C;
    uint8_t AccelAddressL = 0x2D;
    AccelXLow = ReadIMU(AccelAddressL, 0);
    AccelXHigh = ReadIMU(AccelAddressH, 0);
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
    IMU_output = results;
    return results;              // return IMU response after data word transmit (not sure what this should be)
    
}

uint8_t ReadIMU(uint8_t address, uint8_t reg){             // syntax reformat lightly inspired by https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/examples/MPU9250_Debug/MPU9250_Debug.ino
    return WriteIMU(address | readMask, 0x00, reg); // mask address with read mask. Give garbage data to write
}    

void IMU_gyro(){
    uint8_t address = 0x2B;
    
}


void BlinkAlive(){
    // lazily blink onboard LED's forever so I know the MPU is doing something
    // count up to some arbitrary value and flip outputs to LEDs
    if(LATCbits.LATC0 == 0 && temp == 10000){
            LATCbits.LATC0 = 1;     // yellow light is alive LED
            
            temp = 0;
        }
    if(LATCbits.LATC0 != 0  && temp == 10000){
            LATCbits.LATC0 = 0;     // yellow light is alive LED
            
            
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

//// Bit-banged SPI////
// why did you do it this way? 
//      1) because hardware is for masochists and I've learned to love the pain
//      2) because the PIC18F2553 has issues with its SSPBUF I think

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
//// Interrupt Stuff ////
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
            BlinkAlive();

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
    


