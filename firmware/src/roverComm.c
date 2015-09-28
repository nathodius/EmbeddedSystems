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

char sendToControl(int command, int durration)
{
    char origin = 0x00;
    char sequence = rovercommData.sequenceNumberTx;
    

    DRV_USART1_WriteByte(origin);
    // need an interrupt?
    DRV_USART1_WriteByte(sequence);
    
    DRV_USART1_WriteByte(command << 24);
    DRV_USART1_WriteByte(command << 16);
    DRV_USART1_WriteByte(command << 8);
    DRV_USART1_WriteByte(command);
    
    DRV_USART1_WriteByte(durration << 24);
    DRV_USART1_WriteByte(durration << 16);
    DRV_USART1_WriteByte(durration << 8);
    DRV_USART1_WriteByte(durration);
    
    return 0x01; // success
    
}

char sendToController_ISR(int command, int durration)
{
    char origin = 0x00;
    char sequence = rovercommData.sequenceNumberTx;
    
    xQueueSendToBackFromISR(rovercommData.msgQueue, (void*)&(origin), 0);
    // interrupt
    xQueueSendToBackFromISR(rovercommData.msgQueue, (void*)&(sequence), 0);
    
    DRV_USART1_WriteByte(command << 24);
    DRV_USART1_WriteByte(command << 16);
    DRV_USART1_WriteByte(command << 8);
    DRV_USART1_WriteByte(command);
    
    DRV_USART1_WriteByte(durration << 24);
    DRV_USART1_WriteByte(durration << 16);
    DRV_USART1_WriteByte(durration << 8);
    DRV_USART1_WriteByte(durration);
    
    return 0x01; // success
  
}

// Local functions.

void clearBuffer()
{
    int i = 0;
    for( i; i < 10; i++ )
    {
        rovercommData.bufferedMsg[i] = '\0';
    }
}

void clearLastMsg()
{
    rovercommData.completeMsg.msgOrigin = 0x00;
    rovercommData.completeMsg.sequenceNumber = '\0';
    rovercommData.completeMsg.command = 0;
    rovercommData.completeMsg.durration = 0; 
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
    
    rovercommData.msgQueue = xQueueCreate(10, sizeof(char)); // UART queues data by byte (char).
    if ( rovercommData.msgQueue == 0 ) // The queue was not initialized.
    {
        crash("roverComm task message queue not initialized.");
    }
        
    rovercommData.rxChar = '\0';  
    clearBuffer();
    clearLastMsg(); 
    
    rovercommData.sequenceNumberTx = 0x00;
    rovercommData.sequenceNumberRx = 0x00;
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
        if( rovercommData.msgQueue != 0 ) // The queue was initialized.
        {
            if(xQueueReceive(rovercommData.msgQueue, &(rovercommData.rxChar), portMAX_DELAY ) != pdTRUE)
            {
                /* Check the application's current state. */
                switch ( rovercommData.state )
                {
                    /* Application's initial state. */
                    case ROVERCOMM_STATE_RXCHAR:
                    {
                        if ( xQueueReceive(rovercommData.msgQueue, &(rovercommData.rxChar), portMAX_DELAY ) != pdTRUE )
                        {
                            crash("issue receiving byte of message in wifly thread.");
                        }
                        
                        rovercommData.bufferedMsg[rovercommData.sequenceNumberRx] = rovercommData.rxChar;
                        rovercommData.sequenceNumberRx++;
                        
                        if ( rovercommData.sequenceNumberRx == 10 ) // There is a full buffer
                        {
                           rovercommData.state = ROVERCOMM_STATE_FULLBUF;
                        }
                        break;
                    }

                    case ROVERCOMM_STATE_FULLBUF:
                    {
                        // Assemble message.
                        rovercommData.completeMsg.msgOrigin = rovercommData.bufferedMsg[0];
                        rovercommData.completeMsg.sequenceNumber = rovercommData.bufferedMsg[1];
                        rovercommData.completeMsg.command = (rovercommData.bufferedMsg[2] << 24) | 
                                (rovercommData.bufferedMsg[3] << 16) | (rovercommData.bufferedMsg[4] << 8) | rovercommData.bufferedMsg[5];
                        
                        clearBuffer();
                        clearLastMsg();
                        
                        rovercommData.state = ROVERCOMM_STATE_RXCHAR;
                        
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
