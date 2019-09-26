/**
 * rtdm.h
 * 
 * Real-Time Diagnostics & Monitoring
 * 
 * Component: diagnostics
 */
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
//	File:		RTDM.h
//
// This program along with MPLAB DMCI ( MPLAB 8.10 or higher) create an alternative link 
//between Host PC and target device for debugging applications in real-time. 
//It is required to include the RTDM.C file and RTDM.h into the application project 
//in order to send/receive data through the UART to/from the host PC running under 
//MPLAB (release 8.10 or higher) DMCI environment. 
// NOTE:DMCI included in MPLAB 8.10 or higher is ready and enabled to support data exchange 
//between the host PC and target device. Previous versions of DMCI do not support this feature. 
// NOTE: RTDM is currently supported by PIC24H, dsPIC30F, dsPIC33F and dsPIC33E processors
//
//
//	Written By:		M.Ellis, D. Torres,
//				Microchip Technology Inc
//						
// 
// The following files should be included in the MPLAB project:
//
//		RTDM.c			-- RTDM source code file
//		RTDM.h			-- RTDM header file
//		RTDMUSER.h		-- RTDM user definitions file
//		libpXXXX-coff.a		-- Your dsPIC/24H Peripheral Library		
//		pXXFJXXX.gld		-- your dsPIC/24H Linker script file
//				
//
/*****************************************************************************/
//
// Revision History
//
// 4/7/08  -- First Version Release
/****************************************************************************/

#include <string.h>
#include "hal/uart1.h"
#include "rtdmuser.h"

#ifndef RTDM_H
#define RTDM_H

#define RTDM_UART_V2
			
		
 #if defined RTDM_FCY
	#if defined RTDM_BAUDRATE
	 #define RTDM_BRG	(RTDM_FCY/(16*RTDM_BAUDRATE))-1
	#else
	  #error Cannot calculate BRG value. Please define RTDM_BAUDRATE in RTDMUSER.h file
	#endif
 #else
	 #error Cannot calculate RTDM_BRG value. Please define RTDM_FCY in RTDMUSER.h file
 #endif

 #define RTDM_BAUDRATE_ACTUAL	(RTDM_FCY/(16*(RTDM_BRG+1)))
 #define RTDM_BAUD_ERROR		((RTDM_BAUDRATE_ACTUAL > RTDM_BAUDRATE) ? RTDM_BAUDRATE_ACTUAL - RTDM_BAUDRATE : RTDM_BAUDRATE - RTDM_BAUDRATE_ACTUAL)
 #define RTDM_BAUD_ERROR_PERCENT	(((RTDM_BAUD_ERROR*100)+(RTDM_BAUDRATE/2))/RTDM_BAUDRATE)

 #if	(RTDM_BAUD_ERROR_PERCENT > 1)
	 #error The value loaded to the BRG register produces a baud rate error higher than 2%
 #endif

#define RTDM_BYTE_START_OF_FRAME            '$'
#define RTDM_BYTE_END_OF_FRAME              '#'
#define RTDM_COMMAND_READ_MEMORY            'm'
#define RTDM_COMMAND_WRITE_MEMORY           'M'
#define RTDM_COMMAND_SANITY_CHECK           's'
#define RTDM_COMMAND_GET_RXBUF_LENGTH       'L'

#define RTDM_COMMAND_READ_MEMORY_EOF_INDEX    8
#define RTDM_COMMAND_WRITE_MEMORY_EOF_INDEX   7
#define RTDM_COMMAND_SANITY_CHECK_EOF_INDEX   2
#define RTDM_COMMAND_GET_LENGTH_EOF_INDEX     2

/** RTDM Data Frame Receive State Machine State 
  Description:
    This Enumerated Variable defines the various RTDM Data Frame Receive state 
    machine states .
 */
typedef enum tagRTDM_RECEIVE_STATE
{
    RTDM_IDLE_START = 0,
    /** State to verify Start of Frame 0x24('$') */
    RTDM_VERIFY_SOF = 1,
    /** State to verify received commands are 0x6D('m') or 0x4D('M') 
    *   or 0x73('s') or 0x4C('L') */
    RTDM_VERIFY_COMMAND = 2,
    /** State to read Number of bytes expected from Host if command is 0x4D */
    RTDM_READ_NO_OF_BYTES = 3,
    /** State to to verify End of Frame 0x23('#') */
    RTDM_VERIFY_EOF = 4,
    /** The state to wait till CRC byte CRCH and CRCL are received ,marking
     *  all the data  characters in the frame are received */
    RTDM_READ_CRC = 5,
}RTDM_RECEIVE_STATE;

/**********************  RTDM FUNCTIONS **************************/
int RTDM_ProcessMsgs();
int RTDM_Close();
int RTDM_Start();
unsigned int RTDM_CumulativeCrc16 (unsigned char *buf, unsigned int u16Length, unsigned int u16CRC);

#endif
