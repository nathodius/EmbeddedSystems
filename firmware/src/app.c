// mapping and control threads are 1

/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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

#include "app.h"
#include "app1_public.h"

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

APP_DATA appData;

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

/* TODO:  Add any necessary local functions.
*/


void sendQApp1(int message)
{
	if(appData.theQueue != 0)
	{
		xQueueSend(appData.theQueue, (void*)&(message), portMAX_DELAY);	
	}
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
	appData.rxMessage = 0;
	appData.nextState = APP_STATE_INIT;
	appData.theQueue = xQueueCreate(10, sizeof(APP_STATES));
	if(appData.theQueue == 0)
	{
		//failed to create queue
	}
	theTimerInit(50);
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

/* WHAT WE WANT FOR APP###########
forever loop here
	if receive message
		change state after receive
		block and wait for message forever
 */

void APP_Tasks ( void )
{
	while(1)
	{
		//check if queue exists
		if(appData.theQueue != 0)	
		{
			//receive a message and store into rxMessage ... block 5 ticks if empty queue
			if(xQueueReceive(appData.theQueue, &(appData.rxMessage), portMAX_DELAY ))
			{
				//run timer interrupt state change if message is 0x00000001
				if(appData.rxMessage == 1)
				{
					//update state
					appData.state = appData.nextState;
					//clear the message
					appData.rxMessage = 0;
					
					//run the new state and update next state
					switch ( appData.state )
					{
						/* Application's initial state. */
						case APP_STATE_INIT:
						{
							//set ports output
							debugCharInit();
							debugChar(0x20);
							appData.nextState = APP_STATE_1R;
							break;
						}

						/* TODO: implement your application state machine.*/
						case APP_STATE_1R:
						{
							debugU("Old people can't drive");
							debugChar(0x52);
							appData.nextState = APP_STATE_2O;
							break;
						}
						case APP_STATE_2O:
						{
							debugChar(0x6F);
							appData.nextState = APP_STATE_3B;
							break;
						}
						case APP_STATE_3B:
						{
							debugChar(0x62);
							appData.nextState = APP_STATE_4I;
							break;
						}
						case APP_STATE_4I:
						{
							debugChar(0x69);
							appData.nextState = APP_STATE_5N;
							break;
						}
						case APP_STATE_5N:
						{
							debugChar(0x6E);
							appData.nextState = APP_STATE_6_;
							break;
						}
						case APP_STATE_6_:
						{
							debugChar(0x20);
							appData.nextState = APP_STATE_7Y;
							break;
						}
						case APP_STATE_7Y:
						{
							debugChar(0x59);
							appData.nextState = APP_STATE_8A;
							break;
						}
						case APP_STATE_8A:
						{
							debugChar(0x61);
							appData.nextState = APP_STATE_9N;
							break;
						}
						case APP_STATE_9N:
						{
							debugChar(0x6E);
							appData.nextState = APP_STATE_10G;
							break;
						}
						case APP_STATE_10G:
						{
							debugChar(0x67);
							appData.nextState = APP_STATE_11_;
							break;
						}
						case APP_STATE_11_:
						{
							debugChar(0x20);
							appData.nextState = APP_STATE_1R;
							break;
						}
						/* The default state should never be executed. */
						default:
						{
							/* TODO: Handle error in application's state machine. */
							break;
						}
					}//end of case
				}//end of if message == 1
			}//end of receive message
		}//end of check message queue != 0
		else	//attempt to create the queue again if it doesn't exist
		{
			//something terrible happens normally so have it exit rather than recreate
			debugChar(0xFF);
			//run exit interrupt to OS... or forever loop message
			return;
			//appData.theQueue = xQueueCreate(10, sizeof(APP_STATES));
		}
	}//end of while(1)
}
 

/*******************************************************************************
 End of File
 */
