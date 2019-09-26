/*******************************************************************************
  Input / Output Port COnfiguration Routine source File

  File Name:
    port_config.c

  Summary:
    This file includes subroutine for initializing GPIO pins as analog/digital,
    input or output etc. Also to PPS functionality to Remap-able input or output 
    pins

  Description:
    Definitions in the file are for dsPIC33CK256MP508 MC PIM plugged onto
    Motor Control Development board from Microchip
 
*******************************************************************************/
/*******************************************************************************
* Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
*
* SOFTWARE LICENSE AGREEMENT:
* 
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
* THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
* SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include "port_config.h"
#include "pim_select.h"
#include "userparms.h"

// *****************************************************************************
/* Function:
    SetupGPIOPorts()

  Summary:
    Routine to set-up GPIO ports

  Description:
    Function initializes GPIO pins for input or output ports,analog/digital pins,
    remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void SetupGPIOPorts(void)
{
    // Reset all PORTx register (all inputs)
    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    #ifdef TRISE
        TRISE = 0xFFFF;
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;
    #endif

    MapGPIOHWFunction();

    return;
}
// *****************************************************************************
/* Function:
    Map_GPIO_HW_Function()

  Summary:
    Routine to setup GPIO pin used as input/output analog/digital etc

  Description:
    Function initializes GPIO pins as input or output port pins,analog/digital 
    pins,remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void MapGPIOHWFunction(void)
{
    
    /* ANALOG SIGNALS */

    // Configure Port pins for Motor Current Sensing
    
#ifdef INTERNAL_OPAMP_PIM
    // Ib Out
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;   // Pin 23: OA3OUT/AN4/CMP3B/IBIAS3/RA4
    
    //Ib- PIM:66
    ANSELCbits.ANSELC1 = 1;
    TRISCbits.TRISC1 = 1;   //Pin 28: OA3IN-/AN13/CMP1B/ISRC0/RP49/PMA7/RC1
    
    //Ib+ PIM:73
    ANSELCbits.ANSELC2 = 1;
    TRISCbits.TRISC2 = 1;   //Pin 29: OA3IN+/AN14/CMP2B/ISRC1/RP50/PMD13/PMA13/RC2
    
    //Ia Out
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 1;   //Pin 41: OA2OUT/AN1/AN7/ANA0/CMP1D/CMP2D/CMP3D/RP34/SCL3/INT0/RB2
    
    //Ia- PIM:66
    ANSELBbits.ANSELB3 = 1;
    TRISBbits.TRISB3 = 1;   //Pin 43: PGD2/OA2IN-/AN8/RP35/RB3
    
    //Ia+ PIM:74
    ANSELBbits.ANSELB4 = 1;
    TRISBbits.TRISB4 = 1;   //Pin 45: PGC2/OA2IN+/RP36/RB4
    
    #ifdef SINGLE_SHUNT
     //Ibus Out
    ANSELAbits.ANSELA0 = 1;
    TRISAbits.TRISA0 = 1;   //Pin 16: OA1OUT/AN0/CMP1A/IBIAS0/RA0
    
    //Ibus- PIM:67
    ANSELAbits.ANSELA1 = 1;
    TRISAbits.TRISA1 = 1;   //Pin 18: OA1IN-/ANA1/RA1
    
    //Ibus+ PIM:66
    ANSELAbits.ANSELA2 = 1;
    TRISAbits.TRISA2 = 1;   //Pin 20: OA1IN+/AN9/PMA6/RA2
    
    //Op-Amp Configuration
    AMPCON1Hbits.NCHDIS1 = 0;    //Wide input range for Op Amp #2
    AMPCON1Lbits.AMPEN1 = 1;     //Enables Op Amp #2
    #endif
    
    //Op-Amp Configuration
    AMPCON1Hbits.NCHDIS2 = 0;    //Wide input range for Op Amp #2
    AMPCON1Lbits.AMPEN2 = 1;     //Enables Op Amp #2
    
    
    AMPCON1Hbits.NCHDIS3 = 0;    //Wide input range for Op Amp #3
    AMPCON1Lbits.AMPEN3 = 1;     //Enables Op Amp #3
    
    AMPCON1Lbits.AMPON = 1;      //Enables op amp modules if their respective AMPENx bits are also asserted
    
#else
    //Amplified Ib PIM:21
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;   // Pin 23: OA3OUT/AN4/CMP3B/IBIAS3/RA4
    
    //Amplified Ia PIM:22
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 1;   //Pin 41: OA2OUT/AN1/AN7/ANA0/CMP1D/CMP2D/CMP3D/RP34/SCL3/INT0/RB2
    
#endif
    
    // Potentiometer #1 input - used as Speed Reference
    // POT1 : PIM #32
    ANSELDbits.ANSELD11 = 1;
    TRISDbits.TRISD11 = 1;   // PIN36: AN19/CMP2C/RP75/PMPA0/PMPALL/PSPA0/RD11

    
    /* Digital SIGNALS */   
    // DIGITAL INPUT/OUTPUT PINS

    // Inverter Control - PWM Outputs
    // PWM1L : PIM #93  RP47/PWM1L/PMD6/RB15
    // PWM1H : PIM #94  RP46/PWM1H/PMD5/RB14
    // PWM2L : PIM #98  RP45/PWM2L/PMD4/RB13
    // PWM2H : PIM #99  TDI/RP44/PWM2H/PMD3/RB12
    // PWM3L : PIM #100 TCK/RP43/PWM3L/PMD2/RB11
    // PWM3H : PIM #03  TMS/RP42/PWM3H/PMD1/RB10
    TRISBbits.TRISB14 = 0 ;          
    TRISBbits.TRISB15 = 0 ;         
    TRISBbits.TRISB12 = 0 ;          
    TRISBbits.TRISB13 = 0 ;           
    TRISBbits.TRISB10 = 0 ;          
    TRISBbits.TRISB11 = 0 ;         
    
    // FAULT Pins
    // FAULT : PIM #18
    TRISDbits.TRISD8 = 1;           // RP72/SDO2/PCI19/RD8

    // Debug LEDs
    // LED2 : PIM #01
    TRISEbits.TRISE9 = 0;           // PIN:44 - RE9
    // LED1 : PIM #60
    TRISEbits.TRISE8 = 0;           // PIN:42 - RE8

    // Push button Switches
    
#ifdef MCLV2    
    // S2 : PIM #83
    TRISDbits.TRISD5 = 1;           // PIN:54 - RP69/PMA15/PMCS2/RD5
    // S3 : PIM #84
    TRISEbits.TRISE7 = 1;           // PIN:39 - RE7
#endif

#ifdef MCHV2_MCHV3    
    // Push Button : PIM #68
    TRISEbits.TRISE5 = 1;   // PIN24: RE5 
#endif    
	
	/** Diagnostic Interface for MCLV-2,MCHV-2/3,LVMCDB etc.
        Re-map UART Channels to the device pins connected to the following 
        PIM pins on the Motor Control Development Boards .
        UART_RX : PIM #49 (Input)
        UART_TX : PIM #50 (Output)   */
    _U1RXR = 71;
    _RP70R = 0b000001;
}
