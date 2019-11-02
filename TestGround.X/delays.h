/* 
 * File:   delays.h
 * Author: riomcmahon
 *
 * Created on October 17, 2019, 8:24 PM
 * Shamelessly ripped from https://gist.github.com/jerivas/6814108
 */


#ifndef __DELAYS_H
#define __DELAYS_H

#if defined (__18CXX) || defined(_PLIB)

/* C18 cycle-count delay routines. */

/* Delay of exactly 1 Tcy */
#define Delay1TCY() _delay(1)

/* Delay of exactly 10 Tcy */
#define Delay10TCY() _delay(10)

/* Delay10TCYx
 * Delay multiples of 10 Tcy
 * Passing 0 (zero) results in a delay of 2560 cycles.
 */
void Delay10TCYx(unsigned char);

/* Delay100TCYx
 * Delay multiples of 100 Tcy
 * Passing 0 (zero) results in a delay of 25,600 cycles.
 */
void Delay100TCYx(unsigned char);

/* Delay1KTCYx
 * Delay multiples of 1000 Tcy
 * Passing 0 (zero) results in a delay of 256,000 cycles.
 */
void Delay1KTCYx(unsigned char);

/* Delay10KTCYx
 * Delay multiples of 10,000 Tcy
 * Passing 0 (zero) results in a delay of 2,560,000 cycles.
 */
void Delay10KTCYx(unsigned char);

#endif
#endif

