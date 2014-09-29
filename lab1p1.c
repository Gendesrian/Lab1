// ******************************************************************************************* //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.


// ******************************************************************************************* //
// WIRE COLOR CODING SCHEME

// RED- POWER WIRES (Voltage Sources)
// BLACK- GROUND
// YELLOW- CONTROL WIRES, (SWITCHES, POTS)
// BLUE- MCU TO DEVICES (IO)
// GREEN- DEVICE TO DEVICE (LED-LED, LED-SWITCHES)
// ******************************************************************************************* //

#include "p24fj64ga002.h"
#include <stdio.h>

typedef enum stateTypeEnum {Stop, Debounce1, Start, Debounce2} stateType;       // Define state for State Machine (Switch)

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
    stateType state;                                                            // Define StateType
    state = Stop;                                                               // Initialize state to 'STOP' state

// ****************************************************************************** //

// TODO: Configure AD1PCFG register for configuring input pins between analog input
// and digital IO.

    AD1PCFGbits.PCFG0 = 0;                                                      // 0 analog mode ---- 1 digital mode
    AD1PCFGbits.PCFG1 = 0;                                                      //
    AD1PCFGbits.PCFG4 = 1;                                                      // digital input for external switch @ IO5

// TODO: Configure TRIS register bits for Right and Left LED outputs.
    TRISAbits.TRISA0 = 0;                                                       // A0 as output for Red LED
    TRISAbits.TRISA1 = 0;                                                       // A1 as output for Green LED

// TODO: Configure LAT register bits to initialize Right LED to on.
    LATAbits.LATA0 = 1;                                                         // Turn off A0, Red LED
    LATAbits.LATA1 = 1;                                                         // Turn off A1, Green LED
  //  PORTAbits.RA0 = 0; // changed for constitency
    //PORTAbits.RA1 = 1;

// TODO: Configure ODC register bits to use open drain configuration for Right
// and Left LED output.
    ODCAbits.ODA0 = 1;                                                          // Initialize A0, Red LED, to Open Drain Config
    ODCAbits.ODA1 = 1;                                                          // Initialize A1, Green LED, to Open Drain Config

// TODO: Configure TRIS register bits for switch input.
    TRISBbits.TRISB2 = 1;                                                       // Initialize I05, external switch, as Input (For State Change)
    TRISBbits.TRISB5 = 1;                                                       // Initialize RB5, SW1 switch, as Input (For Reset)

// TODO: Configure CNPU register bits to enable internal pullup resistor for switch
// input.
    CNPU1bits.CN6PUE = 1;                                                       // Initialize IO5, external switch, for internal pullup resistor config
    CNPU2bits.CN27PUE = 1;                                                      // Initialize RB5, SW1 switch, for internal pullup resistor config


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

    // TODO: For each distinct button press, alternate which
// LED is illuminated (on).

// TODO: Use DebounceDelay() function to debounce button press
// and button release in software.
    
while(1) {

            switch(state) {
                case Stop:
                    LATAbits.LATA0 = 0;                                         // RED LED ON
                    LATAbits.LATA1 = 1;                                         // GREEN LED OFF

                    if (PORTBbits.RB2 == 0 && TMR1 == 0) {                      // External Switch Pressed
                        state = Debounce1;

                    }
                    break;

                case Debounce1:
                    if(PORTBbits.RB2 == 1 && TMR1 == 0) {                       // External Switch Released
                        state = Start;
                    }
                    break;

                case Start:
                    //PORTAbits.RA0 = 1; // RED LED OFF
                    //PORTAbits.RA1 = 0; // GREEN LED ON
                    LATAbits.LATA0 = 1;                                         // RED TURNS OFF
                    LATAbits.LATA1 = 0;                                         // GREEN TURNS ON

                    if (PORTBbits.RB2 == 0 && TMR1 == 0) {                      // External Switch Pressed
                        state = Debounce2;
                    }
                    break;

                case Debounce2:
                    if (PORTBbits.RB2  == 1 && TMR1 == 0) {                     // External Switch Released
                        state = Stop;
                    }
                    break;

            }

    }
    return 0;
}

// *******************************************************************************************