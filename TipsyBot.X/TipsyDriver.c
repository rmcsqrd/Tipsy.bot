
/****** ASEN 5519 Final Project ******************************************************
 * Author: Rio McMahon
 * Created 11/14/19
 *
 * Pragma and high level definitions modified from ASEN5519 lab 6 source code. Reference
 * in line citations.
 * 
 **************************************************************************************/

// import standard libraries
#include <xc.h>
#include <stdint.h>
#include <pic18f2553.h>
#include <math.h>

// link in project header files
#include "MotorDriver.h"
#include "SPI_Driver.h"


#define _XTAL_FREQ 24000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS
#pragma PWRTEN=ON, BOREN=ON, BORV=1, CCP2MX=PORTC, XINST=OFF
#pragma config WDT=OFF        // clear this to turn off watchdog timer
#pragma config LVP=OFF        // clear this, allows debug to occur
#pragma config PBADEN=OFF     // clear 

/******************************************************************************
 * Global variables
 ******************************************************************************/

// Motor Driver Variables
char Motor_PWM_status;              // status of the motor PWM 
unsigned short Motor_PWM_cnt;       // counter for motor PWM signal on TMR0
unsigned short PWM_max;             // what motor_PWM_cnt counts up to, changes based on Motor_PWM_status
uint8_t motor_speed = 0;            // speed (0-255) of motor
uint8_t motor_orientation = 0;      // orientation (0=CW, 1=CCW)
uint8_t fall_status = 0;            // bit to detect if bot has fallen and toggles RC1 LED, 1 == fall condition

// Blink Alive Variables
unsigned long temp = 0;                  // temp counter for blink alive

// IMU variables
uint8_t IMU_read_garbage = 0;       // write only dump variable for IMU SPI communication, should never be read from because contents is variable
int16_t AccelZ = 0;                 // variable for accelerometer output Z, combined
int16_t AccelX = 0;                 // variable for accelerometer output X, combined
int16_t GyroY = 0;                  // variable for gyro output Y

// controller variables;
uint8_t dt = 10;                    // discretized time step in ms
double theta = 0;                   // Tipsy angle measured from the vertical position (standing condition theta = 0)
double errork;                      // error  at time step k (error = theta - target angle, target angle = 0)
double errork1;                     // error  at time step k-1
double PID_out;                     // PID output
double pfactor;
double dfactor;
double ifactor;
double imax = 200;
#define PI 3.1415       
double Kp = 110;                    // PID tuning variables
double Kd = 130;
double Ki = 15;
double alpha = 0.99;                 // complementary filter scaling factor
#define target_angle 0

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);                 // Function to initialize hardware and interrupts

// Motor Driver Functions
void TMR0handler(void);             // function for lowpri interrupts

// Blink Alive Functions
void BlinkAlive(void);               // blink LED's forever so I know the board is doing something

// IMU Functions
void IMU_whoami(void);               // function to verify IMU is working properly
void IMU_AccelZ(void);               // function to read accelerometer
void IMU_AccelX(void);               // function to read accelerometer
void IMU_GyroY(void);                // function to read gyro

// control functions
void TipsyController(void);         // control algorithm for tipsy
void SensorFusion(void);            // fuse gyro/accel data via complementary filter
void FallCondition(void);           // function to detect if bot has fallen and toggles RC1 LED
void PID_Controller(void);


/******************************************************************************
 * main()
 ******************************************************************************/
void main(){
    Initial();                      // initialize lots of stuff
    IMU_whoami();                   // have the IMU make an introduction (not required just useful for debug)

      while(1) {                    // do this loop forever
            IMU_AccelZ();             // read accel data from IMU
            IMU_AccelX();
            IMU_GyroY();              // read Gyro data from IMU
            FallCondition();          // check to see if fallen over
            TipsyController();        // control algorithm that updates global variables motor_speed, motor_orientation
            MotorDriver(motor_speed, motor_orientation);  // takes two inputs (speed (0-255) and orientation (0=CW, 1=CCW) )
            __delay_ms(dt);           // delay for discretized control loop between state k-1 and k
      }
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
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
    
//    initialize IMU inspired by https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library/blob/master/src/SparkFunLSM9DS1.cpp
    WriteIMU(0x20, 0xC0, 0);        // set accel ODR to 952Hz
    WriteIMU(0x1F, 0x38, 0);        // turn on X,Y,Z accel

    WriteIMU(0x10, 0xC0, 0);        // set gyro ODR to 952Hz, sensitivity is 245 dps (degrees per second)
    WriteIMU(0x11, 0x00, 0);        // clear INT and OUT selection
    WriteIMU(0x12, 0x00, 0);        // low power mode off, disable high pass filter
    WriteIMU(0x1E, 0x38, 0);        // turn on X,Y,Z gyros
    
//    turn on timer to start interrupts
    T0CONbits.TMR0ON = 1;           // Turning on TMR0
   
    
}

/******************************************************************************
 * IMU Functions
 *
 * These functions are how Tipsy gets information from her IMU. Functions all update
 * globally defined variables. 
 ******************************************************************************/
void IMU_whoami(){
//    test function to verify SPI communication is working properly. Should return 0xEA
    uint8_t IdentityAddress = 0x0F;
    uint8_t IMU_identity = ReadIMU(IdentityAddress, 0);
}

void IMU_AccelZ(){
    uint16_t temp = 0x0000;                         // temp variable to do manipulation within
    uint8_t AccelAddressH = 0x2D;                   // Set address of Z axis high byte
    uint8_t AccelAddressL = 0x2C;                   // set address of Z axis low byte
    uint8_t AccelZLow = ReadIMU(AccelAddressL, 0);  // capture z axis low byte
    uint8_t AccelZHigh = ReadIMU(AccelAddressH, 0); // capture z axis high byte (note that this is 2s complement so MSB is +/-)
    temp = temp | AccelZLow;                        // mask low byte w/ temp
    temp = temp | (AccelZHigh << 8);                // mask bit shifted high byte with temp
    AccelZ = temp;
}

void IMU_AccelX(){
    uint16_t temp = 0x0000;                         // temp variable to do manipulation within
    uint8_t AccelAddressH = 0x29;                   // Set address of Z axis high byte
    uint8_t AccelAddressL = 0x28;                   // set address of Z axis low byte
    uint8_t AccelXLow = ReadIMU(AccelAddressL, 0);  // capture z axis low byte
    uint8_t AccelXHigh = ReadIMU(AccelAddressH, 0); // capture z axis high byte (note that this is 2s complement so MSB is +/-)
    temp = temp | AccelXLow;                        // mask low byte w/ temp
    temp = temp | (AccelXHigh << 8);                // mask bit shifted high byte with temp
    AccelX = temp;
}

void IMU_GyroY(){
    uint16_t temp = 0x0000;                         // temp variable to do manipulation within
    uint8_t GyroAddressH = 0x1B;                   // Set address of Z axis high byte
    uint8_t GyroAddressL = 0x1A;                   // set address of Z axis low byte
    uint8_t GyroYLow = ReadIMU(GyroAddressL, 0);  // capture z axis low byte
    uint8_t GyroYHigh = ReadIMU(GyroAddressH, 0); // capture z axis high byte (note that this is 2s complement so MSB is +/-)
    temp = temp | GyroYLow;                        // mask low byte w/ temp
    temp = temp | (GyroYHigh << 8);                // mask bit shifted high byte with temp
    GyroY = temp;
}

/******************************************************************************
 * IMU Functions
 *
 * These functions are how Tipsy interfaces with her IMU peripheral. Functions all update
 * globally defined variables. Note that the profile view of Tipsy is looking at
 * left side (wires poking out right). Negative PID_out implies she is rotating
 * CCW and requires CCW wheel input. Positive PID_out implies she is rotating
 * CW and required CW wheel input. Kd, Kp, Ki are empirically tuned either
 * through software or (eventually) the knobs on the front. 
 * 
 * algorithm loosely inspired by several sources found in list below. 
 * See any inline comments for more specific references.
 * Resources:
 *      http://ohmwardbond.blogspot.com/2014/12/self-balancing-robot.html?m=1
 *      https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library/blob/master/src/SparkFunLSM9DS1.cpp
 ******************************************************************************/

void TipsyController(){
    INTCONbits.GIEL = 0;            // Disable low-priority interrupts to CPU
    INTCONbits.GIEH = 0;            // Disable all interrupts  
    
    SensorFusion();
    PID_Controller();
    if(PID_out < 0){
        motor_orientation = 1;                         // CCW motor input
        motor_speed = (int) -PID_out;  // empirical scale and cast to int 
    }
    else{
        motor_orientation = 0;                          // CW motor input
        motor_speed = (int) PID_out;    //  empirical scale and cast to int 
    }
    
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts  
}

void SensorFusion(){
    double theta_accel = atan2((double) AccelZ, (double) AccelX);   // interpret theta from accelerometer (atan2 returns radians)
    theta_accel = -theta_accel * 180/PI;                            // convert from radians to degrees
    double theta_gyro = (double) GyroY * 0.00875 * dt/1000;         // interpret theta from gyro, sensitivity is 245 dps. Factor comes from sparkfun code
    theta = alpha*(theta + theta_gyro) + (1-alpha)*theta_accel;     // estimate theta from vertical via complementary filter
}

void FallCondition(){
    if(AccelZ < 6000 && AccelZ > -6000){
        fall_status = 0;                                            // update fall status and toggle indicator LED
        LATCbits.LATC1 = 0;
    }
    else{
        fall_status = 1;
        LATCbits.LATC1 = 1;
    }
    return;
}
   
void PID_Controller(){
//    shamelessly implemented from https://github.com/johnnyonthespot/self-balancing-robot-psoc4/blob/master/main.c
    errork1 = errork;                       // store error at time step k-1
    errork = theta - target_angle;          // calculate current error (deviation from vertical)
    pfactor = Kp*errork;                    // calculate proportional term
    dfactor = Kd*(errork-errork1);          // calculate derivative term
    ifactor = ifactor + Ki*errork;          // calculate integral term
    if(ifactor >= imax)
    {
        ifactor = imax;
    }
    else if(ifactor <= -imax)
    {
        ifactor = -imax;
    }
    
    PID_out = pfactor + dfactor + ifactor;  // update PID output
}

/******************************************************************************
 * Interrupts/Status Functions
 *
 * These functions are how Tipsy handles the ISRs used for motor driving and her 
 * alive LED
 ******************************************************************************/
void BlinkAlive(){
    // lazily blink onboard LED's forever so I know the MPU is doing something
    // count up to some arbitrary value and flip outputs to LEDs
    if(LATCbits.LATC0 == 0 && temp == 3000){
            LATCbits.LATC0 = 1;     // yellow light is alive LED
            temp = 0;
        }
    if(LATCbits.LATC0 != 0  && temp == 3000){
            LATCbits.LATC0 = 0;     // yellow light is alive LED
            temp = 0;
        }
    
    // increment up counter
    temp++;
    
}
   
void __interrupt() HiPriISR(void) {
  
}

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
    


