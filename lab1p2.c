// ******************************************************************************************* //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

#include "p24fj64ga002.h"
#include <stdio.h>
#include "lcd.h"


// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
          BKBUG_ON & COE_ON & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //
// Defines to simply UART's baud rate generator (BRG) regiser
// given the osicllator freqeuncy and PLLMODE.

#define XTFREQ          7372800         	  // On-board Crystal frequency
#define PLLMODE         4               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)

#define BAUDRATE         115200
#define BRGVAL          ((FCY/BAUDRATE)/16)-1

// ******************************************************************************************* //

// Variable utilized to store the count that is incremented within the Timer 1 interrupt
// service routine every second.
// Notes:
//        1. This variable is declared are volatile becuase it is updated in the interrupt
//           service routine but will be read in the main execution loop.
//        2. Declared as unsigned char as the varaible will only store the values between
//           0 and 10.
volatile unsigned char cnt;
unsigned char command;
// ******************************************************************************************* //

typedef enum stateTypeEnum {Stop, Debounce1, Start, Debounce2} stateType;

int main(void)
{
	// printf by default is mapped to serial communication using UART1.
	// NOTES:
	//        1. You must specify a heap size for printf. This is required
	//           becuase printf needs to allocate its own memory, which is
	//           allocated on the heap. This can be set in MPLAB by:
	//           a.) Selecting Build Options...->Project from the Project menu.
	//           b.) Selecting the MPLABLINK30 Tab.
	//           c.) Entering the size of heap, e.g. 512, under Heap Size
	//        2. printf function is advanced and using printf may require
	//           significant code size (6KB-10KB).
	printf("Lab 1: Debugging Statements\n\r");

	// The following code will not work until you have implemented the
	// the required LCD functions defined within lcd.c
	LCDInitialize();

/*          FROM LAB 1 PART 1   */
    stateType state;
    state = Stop;
	// ****************************************************************************** //

	// TODO: Configure AD1PCFG register for configuring input pins between analog input
	// and digital IO.
    AD1PCFGbits.PCFG0 = 0; // 0 analog mode ---- 1 digital mode
    AD1PCFGbits.PCFG1 = 0;
    AD1PCFGbits.PCFG4 = 1; // digital input for external switch @ IO5

	// TODO: Configure TRIS register bits for Right and Left LED outputs.
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;

	// TODO: Configure LAT register bits to initialize Right LED to on.
    LATAbits.LATA0 = 1;
    LATAbits.LATA1 = 1;
  //  PORTAbits.RA0 = 0; // changed for constitency
    //PORTAbits.RA1 = 1;

	// TODO: Configure ODC register bits to use open drain configuration for Right
	// and Left LED output.
    ODCAbits.ODA0 = 1;
    ODCAbits.ODA1 = 1;

	// TODO: Configure TRIS register bits for switch input.
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB5 = 1;

	// TODO: Configure CNPU register bits to enable internal pullup resistor for switch
	// input.
    CNPU1bits.CN6PUE = 1; // FOR  IO5 SWITCH
    CNPU2bits.CN27PUE = 1; //on board switch


	// TODO: Setup Timer 1 to use internal clock (Fosc/2).
	// TODO: Setup Timer 1's prescaler to 1:256.
    T1CON = 0x8030;

 	// TODO: Set Timer 1 to be initially off.
    _TON = 0;

	// TODO: Clear Timer 1 value and reset interrupt flag
    TMR1 = 0;

	// TODO: Set Timer 1's period value register to value for 5 ms.
        // (((Fosc/2) / 256) - 1) * 5 * 10^-3
        //
    PR1 = 288;
    
 while(1)
	{

            switch(state) {
                case Stop:
                    LATAbits.LATA0 = 0; // RED LED ON
                    LATAbits.LATA1 = 1; // GREEN LED OFF

                    if (PORTBbits.RB2 == 0 && TMR1 == 0) {
                        state = Debounce1;

                    }
                    break;

                case Debounce1:
                    if(PORTBbits.RB2 == 1 && TMR1 == 0) {
                        state = Start;
                        LCDPrintString("hi");
                    }
                    break;

                case Start:
                    //PORTAbits.RA0 = 1; // RED LED OFF
                    //PORTAbits.RA1 = 0; // GREEN LED ON
                    LATAbits.LATA0 = 1; // RED TURNS OFF
                    LATAbits.LATA1 = 0; // GREEN TURNS ON

                    if (PORTBbits.RB2 == 0 && TMR1 == 0) {
                        state = Debounce2;
                    }
                    break;

                case Debounce2:
                    if (PORTBbits.RB2  == 1 && TMR1 == 0) {
                        state = Stop;
                    }
                    break;

            }
		// TODO: For each distinct button press, alternate which
		// LED is illuminated (on).

		// TODO: Use DebounceDelay() function to debounce button press
		// and button release in software.
	}

}

// ******************************************************************************************* //
// Defines an interrupt service routine that will execute whenever Timer 1's
// count reaches the specfied period value defined within the PR1 register.
//
//     _ISR and _ISRFAST are macros for specifying interrupts that
//     automatically inserts the proper interrupt into the interrupt vector
//     table
//
//     _T1Interrupt is a macro for specifying the interrupt for Timer 1
//
// The functionality defined in an interrupt should be a minimal as possible
// to ensure additional interrupts can be processed.

void __attribute__((interrupt,auto_psv)) _T1Interrupt(void)
//void _ISR _T1Interrupt(void)
{
	// Clear Timer 1 interrupt flag to allow another Timer 1 interrupt to occur.
	IFS0bits.T1IF = 0;
}

// ******************************************************************************************* //
