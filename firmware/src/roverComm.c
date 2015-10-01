/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    rovercomm.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "rovercomm.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

ROVERCOMM_DATA rovercommData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

// Public functions

char sendToRover(int command, int durration)
{
    char origin = 0x00;
    char sequence = rovercommData.sequenceNumberTx;
    
    // Disassemble the command data bytes.
    char commandByte1 = command & 0xff;
    char commandByte2 = (command & (0xff00)) >> 8;
    char commandByte3 = (command & (0xff0000)) >> 16;
    char commandByte4 = (command & (0xff000000)) >> 24;
    
    // Disassemble the duration data bytes.
    char durationByte1 = command & 0xff;
    char durationByte2 = (command & (0xff00)) >> 8;
    char durationByte3 = (command & (0xff0000)) >> 16;
    char durationByte4 = (command & (0xff000000)) >> 24;
    
    // Send the metadata.
    xQueueSend(rovercommData.txQueue, (void*)&(origin), 0);
    xQueueSend(rovercommData.txQueue, (void*)&(sequence), 0);
    
    // Send the command byte by byte.
    xQueueSend(rovercommData.txQueue, (void*)&(commandByte1), 0);
    xQueueSend(rovercommData.txQueue, (void*)&(commandByte2), 0);
    xQueueSend(rovercommData.txQueue, (void*)&(commandByte3), 0);
    xQueueSend(rovercommData.txQueue, (void*)&(commandByte4), 0);
    
    // Send the duration byte by byte.
    xQueueSend(rovercommData.txQueue, (void*)&(durationByte1), 0);
    xQueueSend(rovercommData.txQueue, (void*)&(durationByte2), 0);
    xQueueSend(rovercommData.txQueue, (void*)&(durationByte3), 0);
    xQueueSend(rovercommData.txQueue, (void*)&(durationByte4), 0);
        
    return 0x01; // success
    
}

/*
char sendToRover_ISR(int command, int duration)
{
    char origin = 0x00;
    char sequence = rovercommData.sequenceNumberTx;
    
    // Disassemble the command data bytes.
    char commandByte1 = command & 0xff;
    char commandByte2 = (command & (0xff00)) >> 8;
    char commandByte3 = (command & (0xff0000)) >> 16;
    char commandByte4 = (command & (0xff000000)) >> 24;
    
    // Disassemble the duration data bytes.
    char durationByte1 = command & 0xff;
    char durationByte2 = (command & (0xff00)) >> 8;
    char durationByte3 = (command & (0xff0000)) >> 16;
    char durationByte4 = (command & (0xff000000)) >> 24;
    
    // Send the metadata.
    xQueueSendToBackFromISR(rovercommData.txQueue, (void*)&(origin), 0);
    xQueueSendToBackFromISR(rovercommData.txQueue, (void*)&(sequence), 0);
    
    // Send the command byte by byte.
    xQueueSendToBackFromISR(rovercommData.txQueue, (void*)&(commandByte1), 0);
    xQueueSendToBackFromISR(rovercommData.txQueue, (void*)&(commandByte2), 0);
    xQueueSendToBackFromISR(rovercommData.txQueue, (void*)&(commandByte3), 0);
    xQueueSendToBackFromISR(rovercommData.txQueue, (void*)&(commandByte4), 0);
    
    // Send the duration byte by byte.
    xQueueSendToBackFromISR(rovercommData.txQueue, (void*)&(durationByte1), 0);
    xQueueSendToBackFromISR(rovercommData.txQueue, (void*)&(durationByte2), 0);
    xQueueSendToBackFromISR(rovercommData.txQueue, (void*)&(durationByte3), 0);
    xQueueSendToBackFromISR(rovercommData.txQueue, (void*)&(durationByte4), 0);
    
    return 0x01; // success
  
}*/

void sendToRxQueue_UART( char data )
{
    xQueueSendFromISR(rovercommData.rxQueue, (void *)&(data), 0);
}

/*
void receiveFromTxQueue_UART ()
{
    char byteToSend;
    xQueueReceive(rovercommData.txQueue, (void *)&(byteToSend), portMAX_DELAY);
    DRV_USART1_WriteByte(byteToSend);  
}
*/

// Local functions.

void clearWiflyBuffer()
{
    int i = 0;
    for( i; i < 10; i++ )
    {
        rovercommData.wiflyBuffer[i] = '\0';
    }
}

void clearLastRoverMsg()
{
    rovercommData.roverMsg.msgOrigin = 0x00;
    rovercommData.roverMsg.sequenceNumber = '\0';
    rovercommData.roverMsg.command = 0;
    rovercommData.roverMsg.durration = 0; 
}

void clearLastFeedbackMsg()
{
    rovercommData.roverFeedback.msgOrigin = 0x00;
    rovercommData.roverFeedback.sequenceNumber = '\0';
    rovercommData.roverFeedback.leftFeedback = 0;
    rovercommData.roverFeedback.rightFeedback = 0; 
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ROVERCOMM_Initialize ( void )

  Remarks:
    See prototype in rovercomm.h.
 */

void ROVERCOMM_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    rovercommData.state = ROVERCOMM_STATE_RXCHAR;
    
    // Initialize App data. 
    
    rovercommData.rxQueue = xQueueCreate(10, sizeof(char)); // UART queues data by byte (char).
    
    
    if ( rovercommData.rxQueue == 0 ) // The queue was not initialized.
    {
        crash("roverComm task message queue not initialized.");
    }
    
    rovercommData.txQueue = xQueueCreate(10, sizeof(char)); // UART queues data by byte (char).
    if ( rovercommData.txQueue == 0 ) // The queue was not initialized.
    {
        crash("roverComm send queue not initialized.");
    }
        
    rovercommData.rxChar = '\0';  
    clearWiflyBuffer();
    clearLastRoverMsg(); 
    
    rovercommData.sequenceNumberTx = 0x00;
    rovercommData.sequenceNumberRx = 0x00;

    DRV_USART1_Initialize();
}


/******************************************************************************
  Function:
    void ROVERCOMM_Tasks ( void )

  Remarks:
    See prototype in rovercomm.h.
 */

void ROVERCOMM_Tasks ( void )
{
    while ( 1 )
    {
        if( (rovercommData.rxQueue != 0) && (rovercommData.txQueue !=0) ) // The queues were initialized.
        {
            /* Check the application's current state. */
            switch ( rovercommData.state )
            {
                /* Application's initial state. */
                case ROVERCOMM_STATE_TXCHAR:
                {
                    
                    if ( uxQueueMessagesWaiting(rovercommData.rxQueue) > 0 )
                    {
                        rovercommData.state = ROVERCOMM_STATE_RXCHAR;
                    }
                    
                    if ( xQueueReceive(rovercommData.txQueue, &(rovercommData.txChar), 0 ) != pdTRUE ) // portMAX_DELAY
                    {
                        break;
                    }

                    rovercommData.sequenceNumberTx++;

                    DRV_USART1_WriteByte(rovercommData.txChar);
                                        
                    break;
                }
                
                case ROVERCOMM_STATE_RXCHAR:
                {
                    if ( uxQueueMessagesWaiting(rovercommData.txQueue) != 0 )
                    {
                       rovercommData.state = ROVERCOMM_STATE_TXCHAR;
                    }
                    
                    if ( xQueueReceive(rovercommData.rxQueue, &(rovercommData.rxChar), 0 ) != pdTRUE )
                    {
                        //rovercommData.state = ROVERCOMM_STATE_TXCHAR;
                        //rovercommData.state = ROVERCOMM_STATE_RXCHAR;
                        break;
                    }
                    
                    // check the sequence number

                    if( (rovercommData.sequenceNumberRx == 0x09) && (rovercommData.rxChar == 0x80) )
                    {
                        rovercommData.state = ROVERCOMM_STATE_FULLBUF;
                        break;
                    }
                    
                    rovercommData.wiflyBuffer[rovercommData.sequenceNumberRx] = rovercommData.rxChar;
                    rovercommData.sequenceNumberRx++;
                    
                    // Simulate receiving a command from somewhere else
                    xQueueSend(rovercommData.txQueue, (void*)&(rovercommData.rxChar), 0);
                    
                    break;
                }

                case ROVERCOMM_STATE_FULLBUF:
                {
                    // Assemble message.
                    rovercommData.roverFeedback.msgOrigin = rovercommData.wiflyBuffer[0];
                    rovercommData.roverFeedback.sequenceNumber = rovercommData.wiflyBuffer[1];
                    rovercommData.roverFeedback.leftFeedback = (rovercommData.wiflyBuffer[2] << 24) | 
                            (rovercommData.wiflyBuffer[3] << 16) | (rovercommData.wiflyBuffer[4] << 8) | rovercommData.wiflyBuffer[5];
                    rovercommData.roverFeedback.rightFeedback = (rovercommData.wiflyBuffer[6] << 24) | 
                            (rovercommData.wiflyBuffer[7] << 16) | (rovercommData.wiflyBuffer[8] << 8) | rovercommData.wiflyBuffer[9];
                    clearWiflyBuffer();
                    //clearLastRoverMsg();
                    
                    // Prepare to receive the next message
                    rovercommData.sequenceNumberRx = 0x00;
                    clearWiflyBuffer();
                    xQueueReset(rovercommData.rxQueue);
                    
                    

                    if ( uxQueueMessagesWaiting(rovercommData.txQueue) != 0 )
                    {
                       rovercommData.state = ROVERCOMM_STATE_TXCHAR;
                    }
                    else
                    {
                       rovercommData.state = ROVERCOMM_STATE_RXCHAR;
                    }

                    break;
                }

                /* The default state should never be executed. */
                default:
                {
                    // Handling error in application's state machine. */
                    crash("roverComm task entered undefined state.");
                    break;
                }
            }
        }
        else // The app was not initialized properly.
        {
            crash("roverComm task message queue not initialized.");
        }
    }
}
 

/*******************************************************************************
 End of File
 */
