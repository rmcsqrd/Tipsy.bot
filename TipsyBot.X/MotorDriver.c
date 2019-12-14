#include "MotorDriver.h"
#include <stdint.h>

#define _XTAL_FREQ 24000000   //Required in XC8 for delays. 16 Mhz oscillator clock



// Motor Driver Variables
char Motor_PWM_status = 0;          // status of the motor PWM 
unsigned short Motor_PWM_cnt = 0;   // counter for motor PWM signal on TMR0
unsigned short PWM_max = 0;         // what motor_PWM_cnt counts up to, changes based on Motor_PWM_status
uint8_t fall_status;            // bit to detect if bot has fallen and toggles RC1 LED


void MotorDriver(unsigned char speed, unsigned char orientation){

    // update value of the PWMA pin on motor driver to match the PWM status on TMR0
    _MD_PWMA = Motor_PWM_status;
    
    // check if speed = 0 or if fall condition bit it high; if yes set standby bit low so motors don't operate
    if(speed == 0 || fall_status == 1){
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
