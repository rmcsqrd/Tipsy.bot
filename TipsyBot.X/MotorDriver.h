/******************************************************************************
 *
 * Author(s): Rio McMahon
 * Created 11/14/19
 *
 *******************************************************************************
 *
 * FileName:        MotorDriver.h
 * Dependencies:    xc.h 
 * Processor:       PIC18F
 * Compiler:        xc8
 *
 *******************************************************************************
 * File Description: This library contains a set of functions for the Motor Driver
 ******************************************************************************/

#include <xc.h>
#include <stdint.h>


#ifndef _MotorDriver_
#define _MotorDriver_


/*------------------------------------------------------------------------------
 * Definitions for this Motor Driver
 -----------------------------------------------------------------------------*/
//Define motor driver pin assignments - should match info at
//https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide#board-overview

#define _MD_STBY_TRIS    TRISAbits.TRISA0
#define _MD_AI1_TRIS     TRISAbits.TRISA1
#define _MD_AI2_TRIS     TRISAbits.TRISA2
#define _MD_PWMA_TRIS    TRISAbits.TRISA3

#define _MD_STBY    LATAbits.LATA0
#define _MD_AI1     LATAbits.LATA1
#define _MD_AI2     LATAbits.LATA2
#define _MD_PWMA    LATAbits.LATA3


/*------------------------------------------------------------------------------
 * Public Library Functions
 -----------------------------------------------------------------------------*/

/******************************************************************************
 *     Function Name:	MotorDriver
 *     Parameters:      Speed [0 to 255], Orientation (=0 for CW, =1 for CCW)
 *     Description:		This function accepts a speed and orientation command 
 *                      which generates an appropriate PWM wave and toggles the 
 *                      appropriate pins on the motor driver to rotate wheels
 *                      at the correct speed/orientation. Reference
 *                      https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide#board-overview
 *                      for simplified pin descriptions
 *
 ******************************************************************************/
void MotorDriver( unsigned char, unsigned char );



#endif


