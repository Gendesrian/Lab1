// ******************************************************************************************* //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

#include "p24fj64ga002.h"
#include <stdio.h>

// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
          BKBUG_ON & COE_ON & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

int main(void)
{
	// ****************************************************************************** //

	// TODO: Configure AD1PCFG register for configuring input pins between analog input
	// and digital IO.
    /*/I AM UNSURE ABOUT THIS
    AD1PCFG.PCFGAN0 = 1;
    AD1PCFG.PCFGAN1 = 1;*/

	// TODO: Configure TRIS register bits for Right and Left LED outputs.
    //USE A0 AND A1 FOR IO1 AND IO2. SET TO 0 FOR OUTPUT
    TRISAbits.TRISA0 = 0; //red light -> IO1
    TRISAbits.TRISA1 = 0; //green light -> IO2

	// TODO: Configure LAT register bits to initialize Right LED to on.
    //SET IO1 TO ON FOR RED LIGHT
    LATAbits.LATA0 = 0; //??-- TODO: I THINK IT IS 0 FOR LAT TO BE ON. -- DOUBLE CHECK THEN EDIT COMMENT


	// TODO: Configure ODC register bits to use open drain configuration for Right
	// and Left LED output.
    ODCAbits.ODA0 = 1;
    ODCAbits.ODA1 = 1;


	// TODO: Configure TRIS register bits for swtich input.
    //RB2 FOR IO5. SET 1 FOR INPUT;
    TRISBbits.TRISB2 = 1;

	// TODO: Configure CNPU register bits to enable internal pullup resistor for switch
	// input.
    CNPU1bits.CN2PUE = 1;
    CNPU1bits.CN3PUE = 1;
    

	// TODO: Setup Timer 1 to use internal clock (Fosc/2).

	// TODO: Setup Timer 1's prescaler to 1:256.

 	// TODO: Set Timer 1 to be initially off.

	// TODO: Clear Timer 1 value and reset interrupt flag

	// TODO: Set Timer 1's period value register to value for 5 ms.

	while(1)
	{
		// TODO: For each distinct button press, alternate which
		// LED is illuminated (on).

		// TODO: Use DebounceDelay() function to debounce button press
		// and button release in software.
	}
	return 0;
}

// *******************************************************************************************