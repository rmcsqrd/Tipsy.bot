/*********************************************************************
 * Hello World - just blink an LED on UBW
 ********************************************************************/
#include <p18cxxx.h>
//#include <delays.h>

#define LED_1              LATCbits.LATC0
#define LED_2              LATCbits.LATC1

void main(void)
{
	// Default all pins to digital
    ADCON1 |= 0x0F;

	// Start out both LEDs off
	LED_1 = 0;
	LED_2 = 0;

	// Make both LED I/O pins be outputs
	TRISCbits.TRISC0 = 0;
	TRISCbits.TRISC1 = 0;

	while(1)
    {
		// Alternate LEDs
		LED_1 = 1;
		LED_2 = 1;
//		Delay10KTCYx(250);

//		LED_1 = 0;
//		LED_2 = 0;
//		Delay10KTCYx(100);
    }
}
