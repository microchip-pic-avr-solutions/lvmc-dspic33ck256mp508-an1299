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
#include <stdint.h>
#include <stdbool.h>
#include "port_config.h"
#include "board_service.h"
#include "adc.h"
#include "pwm.h"

BUTTON_T buttonStartStop;
#ifdef MCLV2
    BUTTON_T buttonSpeedHalfDouble;
#endif
uint16_t boardServiceISRCounter = 0;

void DisablePWMOutputsInverterA(void);
void EnablePWMOutputsInverterA(void);
void BoardServiceInit(void);
void BoardServiceStepIsr(void);
void BoardService(void);
bool IsPressed_Button1(void);
#ifdef MCLV2
bool IsPressed_Button2(void);
#endif

static void ButtonGroupInitialize(void);
static void ButtonScan(BUTTON_T * ,bool);

bool IsPressed_Button1(void)
{
    if(buttonStartStop.status)
    {
        buttonStartStop.status = false;
        return true;
    }
    else
    {
        return false;
    }
}
#ifdef MCLV2
bool IsPressed_Button2(void)
{
    if(buttonSpeedHalfDouble.status)
    {
        buttonSpeedHalfDouble.status = false;
        return true;
    }
    else
    {
        return false;
    }
}
#endif
void BoardServiceStepIsr(void)
{
    if (boardServiceISRCounter <  BOARD_SERVICE_TICK_COUNT)
    {
        boardServiceISRCounter += 1;
    }
}
void BoardService(void)
{
    if (boardServiceISRCounter ==  BOARD_SERVICE_TICK_COUNT)
    {
        /* Button scanning loop for Button 1 to start Motor A */
        ButtonScan(&buttonStartStop,BUTTON_START_STOP);
#ifdef MCLV2
        /* Button scanning loop for SW2 to enter into filed
            weakening mode */
        ButtonScan(&buttonSpeedHalfDouble,BUTTON_SPEED_HALF_DOUBLE);
#endif
        boardServiceISRCounter = 0;
    }
}
void BoardServiceInit(void)
{
    ButtonGroupInitialize();
    boardServiceISRCounter = BOARD_SERVICE_TICK_COUNT;
}

void ButtonScan(BUTTON_T *pButton,bool button) 
{
    if (button == true) 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->debounceCount--;
            pButton->state = BUTTON_DEBOUNCE;
        }
    } 
    else 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->state = BUTTON_NOT_PRESSED;
        } 
        else 
        {
            pButton->state = BUTTON_PRESSED;
            pButton->status = true;
        }
        pButton->debounceCount = 0;
    }
}
void ButtonGroupInitialize(void)
{
    buttonStartStop.state = BUTTON_NOT_PRESSED;
    buttonStartStop.debounceCount = 0;
    buttonStartStop.state = false;
#ifdef MCLV2
    buttonSpeedHalfDouble.state = BUTTON_NOT_PRESSED;
    buttonSpeedHalfDouble.debounceCount = 0;
    buttonSpeedHalfDouble.state = false;
#endif
}
// *****************************************************************************
/* Function:
    Init_Peripherals()

  Summary:
    Routine initializes controller peripherals

  Description:
    Routine to initialize Peripherals used for Inverter Control

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitPeripherals(void)
{                
    InitializeADCs();
    
    InitPWMGenerators();
    
    /* Make sure ADC does not generate interrupt while initializing parameters*/
    DisableADCInterrupt();
}
/**
 * Disable the PWM channels assigned for Inverter #A by overriding them to low state.
 * @example
 * <code>
 * DisablePWMOutputsInverterA();
 * </code>
 */
void DisablePWMOutputsInverterA(void)
{
    /** Set Override Data on all PWM outputs */
    // 0b00 = State for PWM3H,L, if Override is Enabled
    PG3IOCONLbits.OVRDAT = 0;
    // 0b00 = State for PWM2H,L, if Override is Enabled
    PG2IOCONLbits.OVRDAT = 0; 
    // 0b00 = State for PWM1H,L, if Override is Enabled
    PG1IOCONLbits.OVRDAT = 0;  
    
    // 1 = OVRDAT<1> provides data for output on PWM3H
    PG3IOCONLbits.OVRENH = 1; 
    // 1 = OVRDAT<0> provides data for output on PWM3L
    PG3IOCONLbits.OVRENL = 1; 
    
    // 1 = OVRDAT<1> provides data for output on PWM2H
    PG2IOCONLbits.OVRENH = 1;
    // 1 = OVRDAT<0> provides data for output on PWM2L
    PG2IOCONLbits.OVRENL = 1; 
    
    // 1 = OVRDAT<1> provides data for output on PWM1H
    PG1IOCONLbits.OVRENH = 1;  
    // 1 = OVRDAT<0> provides data for output on PWM1L
    PG1IOCONLbits.OVRENL = 1;     
}

/**
 * Enable the PWM channels assigned for Inverter #A by removing Override.
 * @example
 * <code>
 * EnablePWMOutputsInverterA();
 * </code>
 */
void EnablePWMOutputsInverterA(void)
{    
    // 0 = PWM Generator provides data for the PWM3H pin
    PG3IOCONLbits.OVRENH = 0; 
    // 0 = PWM Generator provides data for the PWM3L pin
    PG3IOCONLbits.OVRENL = 0; 
    
    // 0 = PWM Generator provides data for the PWM2H pin
    PG2IOCONLbits.OVRENH = 0;
    // 0 = PWM Generator provides data for the PWM2L pin
    PG2IOCONLbits.OVRENL = 0; 
    
    // 0 = PWM Generator provides data for the PWM1H pin
    PG1IOCONLbits.OVRENH = 0;  
    // 0 = PWM Generator provides data for the PWM1L pin
    PG1IOCONLbits.OVRENL = 0;     
}