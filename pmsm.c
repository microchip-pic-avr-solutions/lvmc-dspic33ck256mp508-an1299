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
// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Boot Segment Flash page address  limit)

// FSIGN

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)
//#pragma config PLLKEN = LOCK            // PLL Lock source select (Source for PLL Lock signal is lock detect)
#pragma config XTCFG = G3               // XT Config (24-32 MHz crystals)
#pragma config XTBST = DISABLE           // XT Boost (Boost the kick-start)

// FWDT
#pragma config RWDTPS = PS1048576       // Run Mode Watchdog Timer Post Scaler select bits (1:1048576)
#pragma config RCLKSEL = LPRC           // Watchdog Timer Clock Select bits (Always use LPRC)
#pragma config WINDIS = ON              // Watchdog Timer Window Enable bit (Watchdog Timer operates in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config SWDTPS = PS1048576       // Sleep Mode Watchdog Timer Post Scaler select bits (1:1048576)
#pragma config FWDTEN = ON_SW           // Watchdog Timer Enable bit (WDT controlled via SW, use WDTCON.ON bit)

// FPOR
#pragma config BISTDIS = DISABLED       // Memory BIST Feature Disable (mBIST on reset feature disabled)
//#pragma config BSSO = NORMAL            // Boot Space Start Option (Normal startup operation after reset, execute instruction at 0x000000)

// FICD
#pragma config ICS = PGD3               // ICD Communication Channel Select bits (Communicate on PGEC3 and PGED3)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config NOBTSWP = DISABLED       // BOOTSWP instruction disable bit (BOOTSWP instruction is disabled)

// FDMTIVTL
#pragma config DMTIVTL = 0x0         // Dead Man Timer Interval low word (Lower 16 bits of 32 bitDMT window interval (0-0xFFFF))

// FDMTIVTH
#pragma config DMTIVTH = 0x0        // Dead Man Timer Interval high word (Uper 16 bits of 32 bitDMT window interval (0-0xFFFF))

// FDMTCNTL
#pragma config DMTCNTL = 0x0         // Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF))

// FDMTCNTH
#pragma config DMTCNTH = 0x0         // Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF))

// FDMT
#pragma config DMTDIS = OFF             // Dead Man Timer Disable bit (Dead Man Timer is Disabled and can be enabled by software)

// FDEVOPT
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config ALTI2C3 = OFF            // Alternate I2C3 Pin bit (I2C3 mapped to SDA3/SCL3 pins)
#pragma config SMBEN = SMBUS            // SM Bus Enable (SMBus input threshold is enabled)
#pragma config SPI2PIN = PPS            // SPI2 Pin Select bit (SPI2 uses I/O remap (PPS) pins)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT3 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config CTXT4 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)

// FBTSEQ
#pragma config BSEQ = 0xFFF             // Relative value defining which partition will be active after device Reset; the partition containing a lower boot number will be active (Boot Sequence Number bits)
#pragma config IBSEQ = 0xFFF            // The one's complement of BSEQ; must be calculated by the user and written during device programming. (Inverse Boot Sequence Number bits)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include <libq.h>      
#include "motor_control_noinline.h"

#include "general.h"   
#include "userparms.h"

#include "control.h"   
#include "estim.h"
#include "fdweak.h"

#include "readadc.h"   
#include "meascurr.h" 
#include "clock.h"
#include "pwm.h"
#include "adc.h"
#include "port_config.h"
#include "delay.h"
#include "board_service.h"
#include "diagnostics.h"
#ifdef SINGLE_SHUNT
    #include "singleshunt.h"
#endif

volatile UGF_T uGF;

CTRL_PARM_T ctrlParm;
MOTOR_STARTUP_DATA_T motorStartUpData;

MEAS_CURR_PARM_T measCurrParm;   
READ_ADC_PARM_T readADCParm;  

volatile int16_t thetaElectrical = 0,thetaElectricalOpenLoop = 0;
uint16_t pwmPeriod;
uint16_t adcDataBuffer12=0,adcDataBuffer15=0;

MC_ALPHABETA_T valphabeta,ialphabeta;
MC_SINCOS_T sincosTheta;
MC_DQ_T vdq,idq;
MC_DUTYCYCLEOUT_T pwmDutycycle;
MC_ABC_T   vabc,iabc;

MC_PIPARMIN_T piInputIq;
MC_PIPARMOUT_T piOutputIq;
MC_PIPARMIN_T piInputId;
MC_PIPARMOUT_T piOutputId;
MC_PIPARMIN_T piInputOmega;
MC_PIPARMOUT_T piOutputOmega;

volatile uint16_t adcDataBuffer;
volatile uint16_t measCurrOffsetFlag = 0;

/** Definitions */
/* Open loop angle scaling Constant - This corresponds to 1024(2^10)
   Scaling down motorStartUpData.startupRamp to thetaElectricalOpenLoop   */
#define STARTUPRAMP_THETA_OPENLOOP_SCALER       10 
/* Fraction of dc link voltage(expressed as a squared amplitude) to set the limit for current controllers PI Output */
#define MAX_VOLTAGE_VECTOR                      0.98

void InitControlParameters(void);
void DoControl( void );
void CalculateParkAngle(void);
void ResetParmeters(void);
void MeasCurrOffset(int16_t *,int16_t *);
void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *,MC_DUTYCYCLEOUT_T *);
void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *);

// *****************************************************************************
/* Function:
   main()

  Summary:
    main() function

  Description:
    program entry point, calls the system initialization function group 
    containing the buttons polling loop

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

int main ( void )
{
    InitOscillator();
    SetupGPIOPorts();

    /* Turn on LED2 to indicate the device is programmed */
    LED2 = 1;
    /* Initialize Peripherals */
    InitPeripherals();
    #ifdef SINGLE_SHUNT
    /* Initializing Current offsets in structure variable */
        measBusCurrParm.offsetBus = (int16_t)(ADCBUF_INV_A_IBUS);
    #else
    MeasCurrOffset(&measCurrParm.Offseta,&measCurrParm.Offsetb);
    #endif

    DiagnosticsInit();
    
    BoardServiceInit();
    CORCONbits.SATA = 0;
    while(1)
    {        
        /* Initialize PI control parameters */
        InitControlParameters();        
        /* Initialize estimator parameters */
        InitEstimParm();
        /* Initialize flux weakening parameters */
        InitFWParams();
        /* Reset parameters used for running motor through Inverter A*/
        ResetParmeters();
        
        while(1)
        {
            DiagnosticsStepMain();
            BoardService();
            
            if(IsPressed_Button1())
            {
                if(uGF.bits.RunMotor == 1)
                {
                    ResetParmeters();
                }
                else
                {
                    EnablePWMOutputsInverterA();
                    uGF.bits.RunMotor = 1;
                }

            }
            // Monitoring for Button 2 press in LVMC
            if(IsPressed_Button2())
            {
                if((uGF.bits.RunMotor == 1) && (uGF.bits.OpenLoop == 0))
                {
                    uGF.bits.ChangeSpeed = !uGF.bits.ChangeSpeed;
                }
            }

            /* LED1 is used as motor run Status */
            LED1 = uGF.bits.RunMotor;
        }

    } // End of Main loop
    // should never get here
    while(1){}
}
// *****************************************************************************
/* Function:
    ResetParmsA()

  Summary:
    This routine resets all the parameters required for Motor through Inv-A

  Description:
    Reinitializes the duty cycle,resets all the counters when restarting motor

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void ResetParmeters(void)
{
    /* Make sure ADC does not generate interrupt while initializing parameters*/
	DisableADCInterrupt();
    
    /* Re initialize the duty cycle to minimum value */
    INVERTERA_PWM_PDC3 = MIN_DUTY;
    INVERTERA_PWM_PDC2 = MIN_DUTY;
    INVERTERA_PWM_PDC1 = MIN_DUTY;
    
    DisablePWMOutputsInverterA();
    
    /* Stop the motor   */
    uGF.bits.RunMotor = 0;        
    /* Set the reference speed value to 0 */
    ctrlParm.qVelRef = 0;
    /* Restart in open loop */
    uGF.bits.OpenLoop = 1;
    /* Change speed */
    uGF.bits.ChangeSpeed = 0;
    /* Change mode */
    uGF.bits.ChangeMode = 1;
    
    /* Initialize PI control parameters */
    InitControlParameters();        
    /* Initialize estimator parameters */
    InitEstimParm();
    /* Initialize flux weakening parameters */
    InitFWParams();
    
    #ifdef SINGLE_SHUNT
        /* Initialize Single Shunt Related parameters */
        SingleShunt_InitializeParameters(&singleShuntParam);
        INVERTERA_PWM_TRIGA = ADC_SAMPLING_POINT;
        PG4PHASE = MIN_DUTY;
        PG2PHASE = MIN_DUTY;
        PG1PHASE = MIN_DUTY;
    #else
        INVERTERA_PWM_TRIGA = ADC_SAMPLING_POINT;
    #endif

    /* Enable ADC interrupt and begin main loop timing */
    ClearADCIF();
    adcDataBuffer = ClearADCIF_ReadADCBUF();
    EnableADCInterrupt();
}
// *****************************************************************************
/* Function:
    DoControl()

  Summary:
    Executes one PI iteration for each of the three loops Id,Iq,Speed

  Description:
    This routine executes one PI iteration for each of the three loops
    Id,Iq,Speed

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void DoControl( void )
{
    /* Temporary variables for sqrt calculation of q reference */
    volatile int16_t temp_qref_pow_q15;
    
    if(uGF.bits.OpenLoop)
    {
        /* OPENLOOP:  force rotating angle,Vd and Vq */
        if(uGF.bits.ChangeMode)
        {
            /* Just changed to open loop */
            uGF.bits.ChangeMode = 0;

            /* Synchronize angles */
            /* VqRef & VdRef not used */
            ctrlParm.qVqRef = 0;
            ctrlParm.qVdRef = 0;

            /* Reinitialize variables for initial speed ramp */
            motorStartUpData.startupLock = 0;
            motorStartUpData.startupRamp = 0;
            #ifdef TUNING
                motorStartUpData.tuningAddRampup = 0;
                motorStartUpData.tuningDelayRampup = 0;
            #endif
        }
        /* Speed reference */
        ctrlParm.qVelRef = Q_CURRENT_REF_OPENLOOP;
        /* q current reference is equal to the velocity reference 
         while d current reference is equal to 0
        for maximum startup torque, set the q current to maximum acceptable 
        value represents the maximum peak value */
        ctrlParm.qVqRef = ctrlParm.qVelRef;

        /* PI control for Q */
        piInputIq.inMeasure = idq.q;
        piInputIq.inReference = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;

        /* PI control for D */
        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d = piOutputId.out;

    }
    else
    /* Closed Loop Vector Control */
    {
        /* if change speed indication, double the speed */
        if(uGF.bits.ChangeSpeed)
        {
            /* read not signed ADC */
            ReadADC0(ADCBUF_SPEED_REF_A,&readADCParm);
            
            /* Potentiometer value is scaled between NOMINALSPEED_ELECTR and 
             * MAXIMUMSPEED_ELECTR to set the speed reference*/
            readADCParm.qAnRef = (__builtin_muluu(readADCParm.qADValue,
                    MAXIMUMSPEED_ELECTR-NOMINALSPEED_ELECTR)>>15)+
                    NOMINALSPEED_ELECTR;  

        }
        else
        {
            /* unsigned values */
            ReadADC0(ADCBUF_SPEED_REF_A,&readADCParm);

            /* Potentiometer value is scaled between ENDSPEED_ELECTR 
             * and NOMINALSPEED_ELECTR to set the speed reference*/
            
            readADCParm.qAnRef = (__builtin_muluu(readADCParm.qADValue,
                    NOMINALSPEED_ELECTR-ENDSPEED_ELECTR)>>15) +
                    ENDSPEED_ELECTR;  
            
        }
        if(ctrlParm.speedRampCount < SPEEDREFRAMP_COUNT)
        {
           ctrlParm.speedRampCount++; 
        }
        else
        {
        /* Ramp generator to limit the change of the speed reference
          the rate of change is defined by CtrlParm.qRefRamp */
        ctrlParm.qDiff = ctrlParm.qVelRef - readADCParm.qAnRef;
        /* Speed Ref Ramp */
        if (ctrlParm.qDiff < 0)
        {
            /* Set this cycle reference as the sum of
            previously calculated one plus the reference ramp value */
            ctrlParm.qVelRef = ctrlParm.qVelRef+ctrlParm.qRefRamp;
        }
        else
        {
            /* Same as above for speed decrease */
            ctrlParm.qVelRef = ctrlParm.qVelRef-ctrlParm.qRefRamp;
        }
        /* If difference less than half of ref ramp, set reference
        directly from the pot */
        if (_Q15abs(ctrlParm.qDiff) < (ctrlParm.qRefRamp << 1))
        {
            ctrlParm.qVelRef = readADCParm.qAnRef;
        }
            ctrlParm.speedRampCount = 0;
        }
        /* Tuning is generating a software ramp
        with sufficiently slow ramp defined by 
        TUNING_DELAY_RAMPUP constant */
        #ifdef TUNING
            /* if delay is not completed */
            if(motorStartUpData.tuningDelayRampup > TUNING_DELAY_RAMPUP)
            {
                motorStartUpData.tuningDelayRampup = 0;
            }
            /* While speed less than maximum and delay is complete */
            if((motorStartUpData.tuningAddRampup < (MAXIMUMSPEED_ELECTR - ENDSPEED_ELECTR)) &&
                                                  (motorStartUpData.tuningDelayRampup == 0) )
            {
                /* Increment ramp add */
                motorStartUpData.tuningAddRampup++;
            }
            motorStartUpData.tuningDelayRampup++;
            /* The reference is continued from the open loop speed up ramp */
            ctrlParm.qVelRef = ENDSPEED_ELECTR +  motorStartUpData.tuningAddRampup;
        #endif

        if( uGF.bits.ChangeMode )
        {
            /* Just changed from open loop */
            uGF.bits.ChangeMode = 0;
            piInputOmega.piState.integrator = (int32_t)ctrlParm.qVqRef << 13;
        }

        /* If TORQUE MODE skip the speed controller */
        #ifndef	TORQUE_MODE
            /* Execute the velocity control loop */
            piInputOmega.inMeasure = estimator.qVelEstim;
            piInputOmega.inReference = ctrlParm.qVelRef;
            MC_ControllerPIUpdate_Assembly(piInputOmega.inReference,
                                           piInputOmega.inMeasure,
                                           &piInputOmega.piState,
                                           &piOutputOmega.out);
            ctrlParm.qVqRef = piOutputOmega.out;
        #else
            ctrlParm.qVqRef = ctrlParm.qVelRef;
        #endif
        
        /* Flux weakening control - the actual speed is replaced 
        with the reference speed for stability 
        reference for d current component 
        adapt the estimator parameters in concordance with the speed */
        ctrlParm.qVdRef=FieldWeakening(_Q15abs(ctrlParm.qVelRef));

        /* PI control for D */
        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d    = piOutputId.out;

        /* Dynamic d-q adjustment
         with d component priority 
         vq=sqrt (vs^2 - vd^2) 
        limit vq maximum to the one resulting from the calculation above */
        temp_qref_pow_q15 = (int16_t)(__builtin_mulss(piOutputId.out ,
                                                      piOutputId.out) >> 15);
        temp_qref_pow_q15 = Q15(MAX_VOLTAGE_VECTOR) - temp_qref_pow_q15;
        piInputIq.piState.outMax = Q15SQRT (temp_qref_pow_q15);

        /* PI control for Q */
        piInputIq.inMeasure  = idq.q;
        piInputIq.inReference  = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;
    }
      
}
// *****************************************************************************
/* Function:
   _ADCInterrupt()

  Summary:
   _ADCInterrupt() ISR routine

  Description:
    Does speed calculation and executes the vector update loop
    The ADC sample and conversion is triggered by the PWM period.
    The speed calculation assumes a fixed time interval between calculations.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void __attribute__((__interrupt__,no_auto_psv)) _ADCInterrupt()
{
    /* Read ADC Buffet to Clear Flag */
	adcDataBuffer = ClearADCIF_ReadADCBUF();
    adcDataBuffer12 = ADCBUF12;
    adcDataBuffer15 = ADCBUF15;

    #ifdef SINGLE_SHUNT
    /* If SINGLE_SHUNT is Enabled then single shunt three phase reconstruction
       algorithm is executed in this ISR */   
    if(uGF.bits.RunMotor)
    {
        /* If single shunt algorithm is enabled, three ADC interrupts will be
		 serviced every PWM period in order to sample current twice and
		 be able to reconstruct the three phases and to read the value of POT*/
        
        switch(singleShuntParam.adcSamplePoint)
        {
            /* This algorithm only reads one analog input when running in 
                single shunt mode. During PWM Timer is counting down, sample
                and hold zero is configured to read bus current. Once the 
                algorithm is done measuring shunt current at two different 
                times, then the input of this sample and hold is configured 
                to read the POT when PWM Timer is counting up*/
            case SS_SAMPLE_POT:
                /*Set Trigger to measure BusCurrent first sample during PWM 
                  Timer is counting down*/
                 
                singleShuntParam.adcSamplePoint = 1;                             
            break;
            case SS_SAMPLE_BUS1:
                /*Set Trigger to measure BusCurrent Second sample during PWM 
                  Timer is counting down*/
                singleShuntParam.adcSamplePoint = 2;  
                /* Ibus is measured and offset removed from measurement*/
                singleShuntParam.Ibus1 = (int16_t)(ADCBUF_INV_A_IBUS) - measBusCurrParm.offsetBus;
                
            break;
            case SS_SAMPLE_BUS2:
                /*Set Trigger to measure POT Value when PWM 
                  Timer value is zero*/
                INVERTERA_PWM_TRIGA = ADC_SAMPLING_POINT;
                singleShuntParam.adcSamplePoint = 0;
                /* this interrupt corresponds to the second trigger and 
                    save second current measured*/
                /* Ibus is measured and offset removed from measurement*/
                singleShuntParam.Ibus2 = (int16_t)(ADCBUF_INV_A_IBUS) - measBusCurrParm.offsetBus;
                /* Reconstruct Phase currents from Bus Current*/                
                SingleShunt_PhaseCurrentReconstruction(&singleShuntParam);
                /* Calculate qIa,qIb */
                MeasCompCurr(singleShuntParam.Ia, singleShuntParam.Ib,&measCurrParm);
                iabc.a = measCurrParm.qIa;
                iabc.b = measCurrParm.qIb;
                /* Calculate qId,qIq from qSin,qCos,qIa,qIb */
                MC_TransformClarke_Assembly(&iabc,&ialphabeta);
                MC_TransformPark_Assembly(&ialphabeta,&sincosTheta,&idq);

                /* Speed and field angle estimation */
                Estim();
                /* Calculate control values */
                DoControl();
                /* Calculate qAngle */
                CalculateParkAngle();
                /* if open loop */
                if(uGF.bits.OpenLoop == 1)
                {
                    /* the angle is given by park parameter */
                    thetaElectrical = thetaElectricalOpenLoop;
                }
                else
                {
                    /* if closed loop, angle generated by estimator */
                    thetaElectrical = estimator.qRho;
                }
                MC_CalculateSineCosine_Assembly_Ram(thetaElectrical,&sincosTheta);
                MC_TransformParkInverse_Assembly(&vdq,&sincosTheta,&valphabeta);

                MC_TransformClarkeInverseSwappedInput_Assembly(&valphabeta,&vabc);
                SingleShunt_CalculateSpaceVectorPhaseShifted(&vabc,pwmPeriod,&singleShuntParam);
                PWMDutyCycleSetDualEdge(&singleShuntParam.pwmDutycycle1,&singleShuntParam.pwmDutycycle2);
                DiagnosticsStepIsr();
                BoardServiceStepIsr();
            break;
            
            default:
            break;  
        }
        
    }
    else
    {
        DiagnosticsStepIsr();
        BoardServiceStepIsr();
    }    
    
    #else  
    /* When single shunt is not enabled, that is when is running dual 
        shunt resistor algorithm, ADC interrupt is serviced only once*/
    if( uGF.bits.RunMotor )
    {
        /* Calculate qIa,qIb */
        MeasCompCurr(ADCBUF_INV_A_IPHASE1, ADCBUF_INV_A_IPHASE2,&measCurrParm);
        iabc.a = measCurrParm.qIa;
        iabc.b = measCurrParm.qIb;
        /* Calculate qId,qIq from qSin,qCos,qIa,qIb */
        MC_TransformClarke_Assembly(&iabc,&ialphabeta);
        MC_TransformPark_Assembly(&ialphabeta,&sincosTheta,&idq);

        /* Speed and field angle estimation */
        Estim();
        /* Calculate control values */
        DoControl();
        /* Calculate qAngle */
        CalculateParkAngle();
        /* if open loop */
        if(uGF.bits.OpenLoop == 1)
        {
            /* the angle is given by park parameter */
            thetaElectrical = thetaElectricalOpenLoop;
        }
        else
        {
            /* if closed loop, angle generated by estimator */
            thetaElectrical = estimator.qRho;
        }
        MC_CalculateSineCosine_Assembly_Ram(thetaElectrical,&sincosTheta);
        MC_TransformParkInverse_Assembly(&vdq,&sincosTheta,&valphabeta);

        MC_TransformClarkeInverseSwappedInput_Assembly(&valphabeta,&vabc);
        MC_CalculateSpaceVectorPhaseShifted_Assembly(&vabc,pwmPeriod,
                                                            &pwmDutycycle);
        PWMDutyCycleSet(&pwmDutycycle);
        
    }
    else
    {
//        pwmDutycycle.dutycycle3 = MIN_DUTY;
//        pwmDutycycle.dutycycle2 = MIN_DUTY;
//        pwmDutycycle.dutycycle1 = MIN_DUTY;
//        PWMDutyCycleSet(&pwmDutycycle);
        INVERTERA_PWM_PDC3 = MIN_DUTY;
        INVERTERA_PWM_PDC2 = MIN_DUTY;
        INVERTERA_PWM_PDC1 = MIN_DUTY;
        measCurrOffsetFlag = 1;
    }
    DiagnosticsStepIsr();
    BoardServiceStepIsr();
    #endif
    
    ClearADCIF();   
}
// *****************************************************************************
/* Function:
    CalculateParkAngle ()

  Summary:
    Function calculates the angle for open loop control

  Description:
    Generate the start sine waves feeding the motor terminals
    Open loop control, forcing the motor to align and to start speeding up .
 
  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void CalculateParkAngle(void)
{
    /* if open loop */
    if(uGF.bits.OpenLoop)
    {
        /* begin with the lock sequence, for field alignment */
        if (motorStartUpData.startupLock < LOCK_TIME)
        {
            motorStartUpData.startupLock += 1;
        }
        /* Then ramp up till the end speed */
        else if (motorStartUpData.startupRamp < END_SPEED)
        {
            motorStartUpData.startupRamp += OPENLOOP_RAMPSPEED_INCREASERATE;
        }
        /* Switch to closed loop */
        else 
        {
            #ifndef OPEN_LOOP_FUNCTIONING
                uGF.bits.ChangeMode = 1;
                uGF.bits.OpenLoop = 0;
            #endif
        }
        /* The angle set depends on startup ramp */
        thetaElectricalOpenLoop += (int16_t)(motorStartUpData.startupRamp >> 
                                            STARTUPRAMP_THETA_OPENLOOP_SCALER);

    }
    /* Switched to closed loop */
    else 
    {
        /* In closed loop slowly decrease the offset add to the estimated angle */
        if(estimator.qRhoOffset > 0)
        {
            estimator.qRhoOffset--;
        }
    }
}
// *****************************************************************************
/* Function:
    InitControlParameters()

  Summary:
    Function initializes control parameters

  Description:
    Initialize control parameters: PI coefficients, scaling constants etc.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitControlParameters(void)
{
    /* ADC - Measure Current & Pot */
    /* Scaling constants: Determined by calibration or hardware design.*/
    readADCParm.qK = KPOT;
    measCurrParm.qKa = KCURRA;
    measCurrParm.qKb = KCURRB;
    
    ctrlParm.qRefRamp = SPEEDREFRAMP;
    ctrlParm.speedRampCount = SPEEDREFRAMP_COUNT;
    /* Set PWM period to Loop Time */
    pwmPeriod = LOOPTIME_TCY;
 
    /* PI - Id Current Control */
    piInputId.piState.kp = D_CURRCNTR_PTERM;
    piInputId.piState.ki = D_CURRCNTR_ITERM;
    piInputId.piState.kc = D_CURRCNTR_CTERM;
    piInputId.piState.outMax = D_CURRCNTR_OUTMAX;
    piInputId.piState.outMin = -piInputId.piState.outMax;
    piInputId.piState.integrator = 0;
    piOutputId.out = 0;

    /* PI - Iq Current Control */
    piInputIq.piState.kp = Q_CURRCNTR_PTERM;
    piInputIq.piState.ki = Q_CURRCNTR_ITERM;
    piInputIq.piState.kc = Q_CURRCNTR_CTERM;
    piInputIq.piState.outMax = Q_CURRCNTR_OUTMAX;
    piInputIq.piState.outMin = -piInputIq.piState.outMax;
    piInputIq.piState.integrator = 0;
    piOutputIq.out = 0;

    /* PI - Speed Control */
    piInputOmega.piState.kp = SPEEDCNTR_PTERM;
    piInputOmega.piState.ki = SPEEDCNTR_ITERM;
    piInputOmega.piState.kc = SPEEDCNTR_CTERM;
    piInputOmega.piState.outMax = SPEEDCNTR_OUTMAX;
    piInputOmega.piState.outMin = -piInputOmega.piState.outMax;
    piInputOmega.piState.integrator = 0;
    piOutputOmega.out = 0;
}
// *****************************************************************************
/* Function:
    measCurrOffset()

  Summary:
    Routine initializes Offset values of current
  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void MeasCurrOffset(int16_t *pOffseta,int16_t *pOffsetb)
{    
    int32_t adcOffsetIa = 0, adcOffsetIb = 0;
    uint16_t i = 0;
                
    /* Enable ADC interrupt and begin main loop timing */
    ClearADCIF();
    adcDataBuffer = ClearADCIF_ReadADCBUF();
    EnableADCInterrupt();
    
    /* Taking multiple sample to measure voltage offset in all the channels */
    for (i = 0; i < (1<<CURRENT_OFFSET_SAMPLE_SCALER); i++)
    {
        measCurrOffsetFlag = 0;
        /* Wait for the conversion to complete */
        while (measCurrOffsetFlag == 1);
        /* Sum up the converted results */
        adcOffsetIa += ADCBUF_INV_A_IPHASE1;
        adcOffsetIb += ADCBUF_INV_A_IPHASE2;
    }
    /* Averaging to find current Ia offset */
    *pOffseta = (int16_t)(adcOffsetIa >> CURRENT_OFFSET_SAMPLE_SCALER);
    /* Averaging to find current Ib offset*/
    *pOffsetb = (int16_t)(adcOffsetIb >> CURRENT_OFFSET_SAMPLE_SCALER);
    measCurrOffsetFlag = 0;
    
    /* Make sure ADC does not generate interrupt while initializing parameters*/
    DisableADCInterrupt();
}

void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *pPwmDutycycle)
{
    if(pPwmDutycycle->dutycycle1 < MIN_DUTY)
    {
        pPwmDutycycle->dutycycle1 = MIN_DUTY;
    }
    if(pPwmDutycycle->dutycycle2<MIN_DUTY)
    {
        pPwmDutycycle->dutycycle2 = MIN_DUTY;
    }
    if(pPwmDutycycle->dutycycle3<MIN_DUTY)
    {
        pPwmDutycycle->dutycycle3 = MIN_DUTY;
    }
        
    INVERTERA_PWM_PDC3 = pPwmDutycycle->dutycycle3;
    INVERTERA_PWM_PDC2 = pPwmDutycycle->dutycycle2;
    INVERTERA_PWM_PDC1 = pPwmDutycycle->dutycycle1;
}
void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *pPwmDutycycle1,MC_DUTYCYCLEOUT_T *pPwmDutycycle2)
{
    if(pPwmDutycycle1->dutycycle1 < MIN_DUTY)
    {
        pPwmDutycycle1->dutycycle1 = MIN_DUTY;
    }
    if(pPwmDutycycle1->dutycycle2<MIN_DUTY)
    {
        pPwmDutycycle1->dutycycle2 = MIN_DUTY;
    }
    if(pPwmDutycycle1->dutycycle3<MIN_DUTY)
    {
        pPwmDutycycle1->dutycycle3 = MIN_DUTY;
    }
    
    INVERTERA_PWM_PHASE3 = pPwmDutycycle1->dutycycle3 + (DDEADTIME>>1);
    INVERTERA_PWM_PHASE2 = pPwmDutycycle1->dutycycle2 + (DDEADTIME>>1);
    INVERTERA_PWM_PHASE1 = pPwmDutycycle1->dutycycle1 + (DDEADTIME>>1);
    
    
    if(pPwmDutycycle2->dutycycle1 < MIN_DUTY)
    {
        pPwmDutycycle2->dutycycle1 = MIN_DUTY;
    }
    if(pPwmDutycycle2->dutycycle2<MIN_DUTY)
    {
        pPwmDutycycle2->dutycycle2 = MIN_DUTY;
    }
    if(pPwmDutycycle2->dutycycle3<MIN_DUTY)
    {
        pPwmDutycycle2->dutycycle3 = MIN_DUTY;
    }
    
    INVERTERA_PWM_PDC3 = pPwmDutycycle2->dutycycle3 - (DDEADTIME>>1);
    INVERTERA_PWM_PDC2 = pPwmDutycycle2->dutycycle2 - (DDEADTIME>>1);
    INVERTERA_PWM_PDC1 = pPwmDutycycle2->dutycycle1 - (DDEADTIME>>1);
}
