/**
 * rtdm.c
 * 
 * RTDM testing and diagnostics
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
//	File:		RTDM.c
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
//		libpXXXX-coff.a		-- Your dsPIC/PIC24 Peripheral Library		
//		pXXXXXXX.gld		-- your dsPIC/PIC24 Linker script file
//				
//
/*****************************************************************************/
//
// Revision History
//
// 4/7/08  -- First Version Release
/****************************************************************************/
//
// Revision History
//
// 4/12/10  -- Added support for dsPIC33E and PIC24E (supports UART3 and UART4)
/****************************************************************************/

#include "rtdm.h"
#include "rtdm_hal_adapter.h"

#if defined(RTDM) && (RTDM == 1)

/*+++++++++++++++++++++++++++++++ RTDM Variables ++++++++++++++++++++++++++++++++++++++++*/
/* Received data is stored in array RTDMRxBuffer  */
unsigned char RTDMRxBuffer[RTDM_RXBUFFERSIZE];
unsigned char * RTDMRxBufferLoLimit = RTDMRxBuffer;
unsigned char * RTDMRxBufferHiLimit = RTDMRxBuffer + RTDM_RXBUFFERSIZE - 1;
unsigned char * RTDMRxBufferIndex = RTDMRxBuffer;
unsigned char * RTDMRxBufferStartMsgPointer;
unsigned char * RTDMRxBufferEndMsgPointer;

/* Data to be transmitted using UART communication module */
const unsigned char RTDMTxdata[] = {'R','T','D','M','\0'};
const unsigned char RTDMSanityCheckOK[] = {'+','$','R','T','D','M','#',0x1B,0x86,'\0'}; 
const unsigned char RTDMWriteMemoryOK[] = {'+','$','O','K','#',0x4C,0x08,'\0'};
const unsigned char RTDMErrorIllegalFunction[] = {'-','$','E',0x01,'#',0xD3,0x6A,'\0'};
unsigned char RTDMErrorFrame[] = {'-','$','E',0,'#',0,0,'\0'};

/* Temp variables used to calculate the CRC16*/
unsigned int RTDMcrcTemp,RTDMcrcTempH,RTDMcrcTempL;

/*Structure enclosing the RTDM flags*/
struct {
			unsigned MessageReceived  :		1;
			unsigned TransmitNow	  :		1;
			unsigned unused :		14;     
		}	RTDMFlags; 
        

/** Variable holding state of RTDM Data Frame Receive States */
RTDM_RECEIVE_STATE rtdmRxState = RTDM_IDLE_START; 

/* UART Configuration data */
/* Holds the value of uart config reg */
unsigned int RTDM_UART_MODE_VALUE;
/* Holds the information regarding uart	TX & RX interrupt modes */    	
unsigned int RTDM_UART_STA_VALUE; 

#if (RTDM_UART == 1)
/******************************************************************************
* Function:     RTDM_Start()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code initilizes the UART to be used to
*			exchange data wiht the host PC
*
* Note:		Some processors may have more UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
#if defined (RTDM_UART_V2)
int RTDM_Start()
{
    UART1_InterruptReceiveDisable();
    UART1_InterruptReceiveFlagClear();
    UART1_InterruptTransmitDisable();
    UART1_InterruptTransmitFlagClear();
    UART1_Initialize();
    UART1_InterruptReceiveEnable();
    UART1_BaudRateDividerSet(RTDM_BRG);
    UART1_SpeedModeStandard();
    UART1_ModuleEnable();     
	
	/************* RTDM Flags Configuration & Initial Values *****************/
	RTDMFlags.MessageReceived = 0;
	RTDMFlags.MessageReceived = 0;
	RTDMRxBufferIndex = RTDMRxBufferLoLimit;
	RTDMRxBufferStartMsgPointer = 	RTDMRxBufferLoLimit;		
	RTDMRxBufferEndMsgPointer = RTDMRxBufferLoLimit;

    return 0;
}

	/********************** UART CONFIGURATIION ***************************/
	/* Turn off UART1 module */	
	/* Configure UART2 receive and transmit interrupt */
	/* Configure UART1 module to transmit 8 bit data with one stopbit.  */ 
	
	
	
	
	/************* RTDM Flags Configuration & Initial Values *****************/

#endif

/******************************************************************************
* Function:     	CloseRTDM()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code closes the UART used to
*			exchange data wiht the host PC
*
* Note:		Some processors may have more UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
int CloseRTDM()
{
	int nRet = 0;
	CloseUART1();
	return nRet;
		
}

/******************************************************************************
* Function:     	RTDM_ProcessMsgs()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code process the message received and then 
*			executes the required task. These tasks are reading an specified memory
*			location, writing an specified memory location, receive a communication
*			link sanity check command, or being asked for the size of the bufffers.
*
* Note:		Some processors may have more UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
int RTDM_ProcessMsgs()
{
	
	//Local pointer management variables
	unsigned long int * RTDMpu32AddressTemp;
	unsigned char     * RTDMpucWrData;
    unsigned char     * RTDMpucRdData;
    unsigned char     * RTDMpucWrAddr;	
    unsigned short      RTDMNumBytes;
    unsigned char       RTDMPacketBuf[16];
    
	unsigned int        RTDMProcessMsgsTemp1, RTDMProcessMsgsTemp2;
	unsigned int        N;

    if (!RTDMFlags.MessageReceived)
	{
		return -1;
	}


	RTDMcrcTemp = 
		RTDM_CumulativeCrc16
			(RTDMRxBufferStartMsgPointer, 
			(unsigned int)(RTDMRxBufferEndMsgPointer-RTDMRxBufferStartMsgPointer)+1, 
			0xFFFF);
			
	RTDMcrcTempH = (RTDMcrcTemp & 0xFF00)>>8;
	RTDMcrcTempL = RTDMcrcTemp & 0x00FF;
	RTDMProcessMsgsTemp1 = (unsigned int)*((RTDMRxBufferEndMsgPointer)+2);
	RTDMProcessMsgsTemp2 = (unsigned int)*((RTDMRxBufferEndMsgPointer)+1);



	RTDMRxBufferStartMsgPointer +=2;
	if((RTDMProcessMsgsTemp1 == (unsigned)RTDMcrcTempH) && (RTDMProcessMsgsTemp2 == RTDMcrcTempL))
	  {

		switch(*((RTDMRxBufferLoLimit)+1))
		  {
		  case 'm':
			/*************** Extract Address **************/
			//Capture address as 32 bit quantity to match protocol definition. 
			RTDMpu32AddressTemp = ((unsigned long *) RTDMRxBufferStartMsgPointer);
			
			//Increment receive buffer pointer to length field.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned long);
			 
			//Init a byte oriented data pointer  
			RTDMpucRdData = (unsigned char *) ((unsigned int) *RTDMpu32AddressTemp);

			/********* Extract Number of Bytes ************/			
			//Capture address as 16 bit quantity to match protocol definition. 
			RTDMNumBytes = *((unsigned short *) RTDMRxBufferStartMsgPointer);
			
			//Increment receive buffer pointer to start of data payload.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned short);
			
            //Init the CRC seed for the cumulative checksum calculation. 					
			RTDMcrcTemp = 0xffff;
			
			//Add packet header prefix
			RTDMPacketBuf[0] = '+';
			RTDMPacketBuf[1] = '$';
			//Add null terminator for putsUARTx function...
			RTDMPacketBuf[2] = 0; 
			
			//Calc header prefix checksum piece
			RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMPacketBuf, 2, RTDMcrcTemp);	
			//Calc data payload checksum
			RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMpucRdData, RTDMNumBytes, RTDMcrcTemp);			
			
			//Send packet header. Use string function to save code space... 
			putsUART1(RTDMPacketBuf);
			while(BusyUART1()); 
			
			//Send data portion of message... 
			while(RTDMNumBytes--)
			 {
				WriteUART1(*RTDMpucRdData++);
				while(BusyUART1()); 
			 }
			
			//Add packet trailer   
		    RTDMPacketBuf[0] = '#';
		    RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMPacketBuf, 1, RTDMcrcTemp);			
		    
		    //Add checksum bytes to packet
			RTDMPacketBuf[1] = RTDMcrcTemp & 0x00FF;
			RTDMPacketBuf[2] = (RTDMcrcTemp & 0xFF00) >> 8;
			
			//Send packet trailer and checksum. 			
			for (N=0; N < 3; N++)
			{
				WriteUART1(RTDMPacketBuf[N]); 
				while(BusyUART1()); 
			}	
		  break;
		  
		  case 'M':
			{
			/*************** Extract Address **************/						
		    //Capture address as 32 bit quantity to match protocol definition. 
		    RTDMpu32AddressTemp = (unsigned long *) RTDMRxBufferStartMsgPointer;
		    
		    //Increment receive buffer pointer to length field.
		    RTDMRxBufferStartMsgPointer += sizeof(unsigned long);
		    		    
			// Init a byte oriented address pointer for use in incrementing 
			//through the address range properly as we write each byte of data
			//in the range (length) of this write request.   
			RTDMpucWrAddr = (unsigned char *) ((unsigned int) *RTDMpu32AddressTemp);

			/********* Extract Number of Bytes ************/
			//MEllis Capture length as 16 bit quantity to match protocol definition.
			RTDMNumBytes = *((unsigned short *) RTDMRxBufferStartMsgPointer);
			
			//MEllis Increment receive buffer pointer to start of data payload.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned short);
			
			/********** Extract Data ************/
			
			//Init a byte oriented data pointer so that we can increment a byte at at 
			//time for as many bytes as are in the range for this write. 
			RTDMpucWrData = RTDMRxBufferStartMsgPointer;
			
			//*** Write Data in specified RAM location *****			
			//Important to increment through address range using byte oriented address and data
			//pointers. Otherwise, single byte or odd byte ranges do not get written correctly. 
			while(RTDMNumBytes--)
			  {
    			*RTDMpucWrAddr++ = *RTDMpucWrData++;
    		  }
			  			
			//Transmit OK message
			putsUART1(RTDMWriteMemoryOK);
			/* Wait for  transmission to complete */
			while(BusyUART1()); 
			break;
			}
		  case 's':
			{
			/* Load transmit buffer and transmit the same till null character is encountered */
			//Transmit OK message
			putsUART1(RTDMSanityCheckOK);
			/* Wait for  transmission to complete */
			while(BusyUART1()); 
		    break;
			}
		  
		  case 'L':
		    RTDMcrcTemp = 0xffff; //Init the CRC seed.
			
			RTDMPacketBuf[0] = '+';
			RTDMPacketBuf[1] = '$';
			//Size of the RTDM Receive buffer.
			RTDMPacketBuf[2] = (sizeof(RTDMRxBuffer) & 0x00FF);
			RTDMPacketBuf[3] = (sizeof(RTDMRxBuffer) & 0xFF00) >> 8;
			//Note: We dod not utilize a transmit buffer since any data memory source is 
			//essentially already buffered. So the transmit limit is now just a way to 
			//limit the total message length that a client make with any single read request.
			RTDMPacketBuf[4] = (RTDM_MAX_XMIT_LEN & 0x00FF);
			RTDMPacketBuf[5] = (RTDM_MAX_XMIT_LEN & 0xFF00) >> 8;
			RTDMPacketBuf[6] = '#';		
			RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMPacketBuf, 7, RTDMcrcTemp);			
			RTDMPacketBuf[7] = (RTDMcrcTemp & 0x00FF);
			RTDMPacketBuf[8] = (RTDMcrcTemp & 0xFF00) >> 8;

			//Send completed message which is 9 bytes in length.
			for (N=0; N < 9; N++)
			{
				WriteUART1(RTDMPacketBuf[N]);
				while(BusyUART1()); 
			}	
			break;
		  
		  default:
			// ---> COMMAND SUPPORTED?? IF NOT ERROR HANDLER
			//Transmit ERROR message 1
			putsUART1 (RTDMErrorIllegalFunction);
			/* Wait for  transmission to complete */
			while(BusyUART1()); 
			break;
		  }
				   
	  }

	  memset(&RTDMRxBuffer, 0, sizeof(RTDMRxBuffer));
	  
	  RTDMFlags.MessageReceived = 0;
	  RTDMRxBufferIndex             = RTDMRxBufferLoLimit;
	  RTDMRxBufferStartMsgPointer   = RTDMRxBufferLoLimit;		
	  RTDMRxBufferEndMsgPointer     = RTDMRxBufferLoLimit;
	  
	  return 0;
}	

/******************************************************************************
* Function:     	_U1RXInterrupt(void)
*
* Output:		void
*
* Overview:	Here is where the RTDM receives the messages using the UART receiver 
*			interrupt, If polling method is selected in the RTDMUSER.h file then
*			the user application should call the RTDM_ProcessMsgs routine in order 
*			to precess up comming messages. If polling method is disabled then the 
*			RTDM_ProcessMsgs routine is called in the UART received interrupt
*			routine.
*
* Note:		Some processors may have more UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
#if (RTDM_POLLING == YES)
/* This is UART1 receive ISR Polling RTDM Messages*/ 
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) 
{
   /* Read the receive buffer until at least one or more character can be read */
    while (DataRdyUART1()) 
    {
        *(RTDMRxBufferIndex) = ReadUART1();

        if ((RTDMRxBufferIndex - RTDMRxBufferStartMsgPointer) == 0) 
        {
            rtdmRxState = RTDM_VERIFY_SOF;
        }
        switch (rtdmRxState) 
        {
            case RTDM_IDLE_START:

                break;
            case RTDM_VERIFY_SOF:
                if (*(RTDMRxBufferIndex) == RTDM_BYTE_START_OF_FRAME) 
                {
                    rtdmRxState = RTDM_VERIFY_COMMAND;
                } 
                else 
                {
                    rtdmRxState = RTDM_IDLE_START;
                }
                break;
            case RTDM_VERIFY_COMMAND:
                if (*(RTDMRxBufferIndex) == RTDM_COMMAND_READ_MEMORY) 
                {
                    RTDMRxBufferEndMsgPointer = RTDMRxBufferStartMsgPointer + 
                            RTDM_COMMAND_READ_MEMORY_EOF_INDEX;
                    rtdmRxState = RTDM_VERIFY_EOF;
                } 
                else if (*(RTDMRxBufferIndex) == RTDM_COMMAND_WRITE_MEMORY) 
                {
                    RTDMRxBufferEndMsgPointer = RTDMRxBufferStartMsgPointer + 
                            RTDM_COMMAND_WRITE_MEMORY_EOF_INDEX;
                    rtdmRxState = RTDM_READ_NO_OF_BYTES;
                } 
                else if (*(RTDMRxBufferIndex) == RTDM_COMMAND_SANITY_CHECK) 
                {
                    RTDMRxBufferEndMsgPointer = RTDMRxBufferStartMsgPointer + 
                            RTDM_COMMAND_SANITY_CHECK_EOF_INDEX;
                    rtdmRxState = RTDM_VERIFY_EOF;
                } 
                else if (*(RTDMRxBufferIndex) == RTDM_COMMAND_GET_RXBUF_LENGTH) 
                {
                    RTDMRxBufferEndMsgPointer = RTDMRxBufferStartMsgPointer + 
                            RTDM_COMMAND_GET_LENGTH_EOF_INDEX;
                    rtdmRxState = RTDM_VERIFY_EOF;
                } 
                else 
                {
                    rtdmRxState = RTDM_IDLE_START;    
                }
                break;
            case RTDM_READ_NO_OF_BYTES:
                if (RTDMRxBufferEndMsgPointer == RTDMRxBufferIndex) 
                {
                    RTDMRxBufferEndMsgPointer += 
                         (*(unsigned int *)(RTDMRxBufferEndMsgPointer - 1)) + 1;
                    rtdmRxState = RTDM_VERIFY_EOF;
                }
                break;
            case  RTDM_VERIFY_EOF:
                if (RTDMRxBufferEndMsgPointer == RTDMRxBufferIndex) 
                {
                    if (*(RTDMRxBufferIndex) == RTDM_BYTE_END_OF_FRAME) 
                    {
                        rtdmRxState = RTDM_READ_CRC;
                    } 
                    else 
                    {
                        rtdmRxState = RTDM_IDLE_START;
                    }
                }
                break;
            case RTDM_READ_CRC:
                if ((RTDMRxBufferEndMsgPointer + 2) == RTDMRxBufferIndex) 
                {
                    RTDMFlags.MessageReceived = 1;
                }
                break;
            default:
                rtdmRxState = RTDM_IDLE_START;
                break;
        }
        if(rtdmRxState == RTDM_IDLE_START)
        {
            RTDMRxBufferIndex = RTDMRxBufferLoLimit;
            RTDMRxBufferEndMsgPointer = RTDMRxBufferLoLimit;
        }
        else
        {
            if (RTDMRxBufferIndex < RTDMRxBufferHiLimit) 
            {
                RTDMRxBufferIndex++;
            } 
            else
            {
                RTDMRxBufferIndex = RTDMRxBufferLoLimit;
                RTDMRxBufferEndMsgPointer = RTDMRxBufferLoLimit;
            }
        }       
    }
    _U1RXIF = 0;
}  

/******************************************************************************
* Function:     	_U1RXInterrupt(void)
*
* Output:		void
*
* Overview:	Here is where the RTDM receives the messages using the UART receiver 
*			interrupt, If polling method is selected in the RTDMUSER.h file then
*			the user application should call the RTDM_ProcessMsgs routine in order 
*			to precess up comming messages. If polling method is disabled then the 
*			RTDM_ProcessMsgs routine is called in the UART received interrupt
*			routine.
*
* Note:		Some processors may have more UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
#else
/* This is UART1 receive ISR without polling RTDM Messages */ 
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) 
{
    
   /* Read the receive buffer until at least one or more character can be read */
    while (DataRdyUART1()) 
    {
        *(RTDMRxBufferIndex) = ReadUART1();

        if ((RTDMRxBufferIndex - RTDMRxBufferStartMsgPointer) == 0) 
        {
            rtdmRxState = RTDM_VERIFY_SOF;
        }
        switch (rtdmRxState) 
        {
            case RTDM_IDLE_START:

                break;
            case RTDM_VERIFY_SOF:
                if (*(RTDMRxBufferIndex) == RTDM_BYTE_START_OF_FRAME) 
                {
                    rtdmRxState = RTDM_VERIFY_COMMAND;
                } 
                else 
                {
                    rtdmRxState = RTDM_IDLE_START;
                }
                break;
            case RTDM_VERIFY_COMMAND:
                if (*(RTDMRxBufferIndex) == RTDM_COMMAND_READ_MEMORY) 
                {
                    RTDMRxBufferEndMsgPointer = RTDMRxBufferStartMsgPointer + 
                            RTDM_COMMAND_READ_MEMORY_EOF_INDEX;
                    rtdmRxState = RTDM_VERIFY_EOF;
                } 
                else if (*(RTDMRxBufferIndex) == RTDM_COMMAND_WRITE_MEMORY) 
                {
                    RTDMRxBufferEndMsgPointer = RTDMRxBufferStartMsgPointer + 
                            RTDM_COMMAND_WRITE_MEMORY_EOF_INDEX;
                    rtdmRxState = RTDM_READ_NO_OF_BYTES;
                } 
                else if (*(RTDMRxBufferIndex) == RTDM_COMMAND_SANITY_CHECK) 
                {
                    RTDMRxBufferEndMsgPointer = RTDMRxBufferStartMsgPointer + 
                            RTDM_COMMAND_SANITY_CHECK_EOF_INDEX;
                    rtdmRxState = RTDM_VERIFY_EOF;
                } 
                else if (*(RTDMRxBufferIndex) == RTDM_COMMAND_GET_RXBUF_LENGTH) 
                {
                    RTDMRxBufferEndMsgPointer = RTDMRxBufferStartMsgPointer + 
                            RTDM_COMMAND_GET_LENGTH_EOF_INDEX;
                    rtdmRxState = RTDM_VERIFY_EOF;
                } 
                else 
                {
                    rtdmRxState = RTDM_IDLE_START;    
                }
                break;
            case RTDM_READ_NO_OF_BYTES:
                if (RTDMRxBufferEndMsgPointer == RTDMRxBufferIndex) 
                {
                    RTDMRxBufferEndMsgPointer += 
                         (*(unsigned int *)(RTDMRxBufferEndMsgPointer - 1)) + 1;
                    rtdmRxState = RTDM_VERIFY_EOF;
                }
                break;
            case  RTDM_VERIFY_EOF:
                if (RTDMRxBufferEndMsgPointer == RTDMRxBufferIndex) 
                {
                    if (*(RTDMRxBufferIndex) == RTDM_BYTE_END_OF_FRAME) 
                    {
                        rtdmRxState = RTDM_READ_CRC;
                    } 
                    else 
                    {
                        rtdmRxState = RTDM_IDLE_START;
                    }
                }
                break;
            case RTDM_READ_CRC:
                if ((RTDMRxBufferEndMsgPointer + 2) == RTDMRxBufferIndex) 
                {
                    RTDMFlags.MessageReceived = 1;
                }
                break;
            default:
                rtdmRxState = RTDM_IDLE_START;
                break;
        }
        if(rtdmRxState == RTDM_IDLE_START)
        {
            RTDMRxBufferIndex = RTDMRxBufferLoLimit;
            RTDMRxBufferEndMsgPointer = RTDMRxBufferLoLimit;
        }
        else
        {
            if (RTDMRxBufferIndex < RTDMRxBufferHiLimit) 
            {
                RTDMRxBufferIndex++;
            } 
            else
            {
                RTDMRxBufferIndex = RTDMRxBufferLoLimit;
                RTDMRxBufferEndMsgPointer = RTDMRxBufferLoLimit;
            }
        }       
    }
    if(RTDMFlags.MessageReceived == 1)
    {
        RTDM_ProcessMsgs();
    }
    _U1RXIF = 0;
}  
#endif

#else
	#error Please define the UART to be used by RTDM in RTDMuserdef.h file
#endif

//Conditionally compile for speed performance or minimum code size.
#if (RTDM_MIN_CODE_SIZE == NO)

//When RTDM_MIN_CODE_SIZE is not defined we employ a table driven crc 
//calculation with pre-calculated polyniomial values to speed-up 
//checksum calculation time. 

const unsigned int crc_16_tab[] = {
  0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
  0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
  0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
  0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
  0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
  0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
  0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
  0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
  0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
  0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
  0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
  0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
  0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
  0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
  0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
  0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
  0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
  0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
  0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
  0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
  0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
  0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
  0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
  0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
  0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
  0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
  0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
  0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
  0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
  0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
  0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
  0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};

/******************************************************************************
* Function:     	RTDM_CumulativeCrc16 
				(unsigned char *buf, 
				unsigned int u16Length, 
				unsigned int u16CRC)
*
* Output:		return CRC16
*
* Overview:	This routine calculates the polynomial for the checksum byte using
*			a coefficients table. 
*			This approach has a faster performance but consumes a higher amount of 
*			program memory
*
* Note:		Some processors may have more UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
unsigned int RTDM_CumulativeCrc16(unsigned char *buf, unsigned int bsize, unsigned int crcSeed)
{
   unsigned char * pData = buf;

   while (bsize--)
    {
   		crcSeed = (unsigned int)(crcSeed >> 8) ^ crc_16_tab[(crcSeed ^ *pData++) & 0xff];
    }		
   
  return crcSeed;

}

#else //This is when the _RTDM_CODE_FOOTPRINT = RTDM_MIN_SIZE



/******************************************************************************
* Function:     	RTDM_CumulativeCrc16 
				(unsigned char *buf, 
				unsigned int u16Length, 
				unsigned int u16CRC)
*
* Output:		return CRC16
*
* Overview:	This routine calculates the polynomial for the checksum byte on the fly
*			every time. Saves code space because no const table is required. 
*			This approach saves code space but yields slower throughput 
*			performance.
*
* Note:		Some processors may have more UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/

int wcopy;
unsigned int RTDM_CumulativeCrc16 (unsigned char *buf, unsigned int u16Length, unsigned int u16CRC)
{
	unsigned int u16Poly16   = 0xA001; // Bits 15, 13 and 0 
	unsigned int DATA_BITS   = 8;      // Number of data bits 
	unsigned int u16BitIdx;
	unsigned int u16MsgIdx;
	unsigned int u16MsgByte;

    for ( u16MsgIdx = 0; u16MsgIdx < u16Length; u16MsgIdx++)
	{
		asm("mov.w w14,_wcopy");
		u16MsgByte = 0x00FF & *buf++;
		for( u16BitIdx = 0; u16BitIdx < DATA_BITS; u16BitIdx++)
		{
			if ( (u16CRC ^ u16MsgByte) & 0x0001 )
			 {
				 u16CRC = ( u16CRC >> 1 ) ^ u16Poly16;
			 }
			else
			 {
				u16CRC = u16CRC >> 1;
			 }
			
			u16MsgByte >>= 1; // Right shift one to get to next bit 
		}
	}

	return u16CRC;
}

#endif // RTDM defined and set to 1

#endif /* End of #ifndef compilation condition. */


