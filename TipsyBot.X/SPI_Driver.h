/******************************************************************************
 *
 * Author(s): Rio McMahon
 * Created 11/14/19
 *
 *******************************************************************************
 *
 * FileName:        SPI_Driver.h
 * Dependencies:    xc.h 
 * Processor:       PIC18F
 * Compiler:        xc8
 *
 *******************************************************************************
 * File Description: This library contains a set of functions for the bit banged SPI Driver
 ******************************************************************************/

#include <xc.h>
#include <stdint.h>


#ifndef _SPI_Driver_
#define _SPI_Driver_
/*------------------------------------------------------------------------------
 * Public Library Functions
 -----------------------------------------------------------------------------*/

/******************************************************************************
 *     Function Name:	SPIGenOutArray
 *     Parameters:      Data to send to SPI one byte
 *     Description:		This function unpacks a single byte into an array to be 
 *                      sent over SPI
 *
 ******************************************************************************/
void SPIGenOutArray(uint8_t);

/******************************************************************************
 *     Function Name:	SPIGenInArray
 *     Parameters:      None
 *     Description:		This function does the opposite of SPIGenOutArray; it 
 *                      takes the output array from SPI and packs it into a single 
 *                      byte 
 *
 ******************************************************************************/
uint8_t SPIGenInArray(void);

/******************************************************************************
 *     Function Name:	SPITransmit
 *     Parameters:      None
 *     Description:		This function transmits the array from SPIGenOutArray
 *
 ******************************************************************************/
void SPITransmit(void);

/******************************************************************************
 *     Function Name:	WriteIMU
 *     Parameters:      address, command, reg
 *     Description:		This function writes to the IMU. It passes the IMU 
 *                      address and command via SPI communication to the
 *                      appropriate IMU register. Reg is 0 for Accel/Gyro
 *                      register bank and 1 for magnetometer bank.
 *
 ******************************************************************************/
uint8_t WriteIMU(uint8_t, uint8_t, uint8_t);

/******************************************************************************
 *     Function Name:	ReadIMU
 *     Parameters:      address, reg
 *     Description:		This function is identitcal to WriteIMU except that it
 *                      masks the "readMask" (makes MSB of address a 1) and 
 *                      sends dummy data.
 *
 ******************************************************************************/
uint8_t ReadIMU(uint8_t, uint8_t);  


#endif


