/*
  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/


#ifndef _APP_H
#define _APP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "queue.h"
#include "app_wifly_public.h"

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,
	APP_STATE_READ=1,
	APP_STATE_WRITE=2,
	APP_STATE_RECEIVE=3
} APP_STATES;


/* Application Data
 */

typedef struct
{
    APP_STATES state;
	QueueHandle_t theQueue;
	APP_WIFLY_MESSAGE rxMessage;
	int rxBufferSize;
	int rxBufferIndex;
	char rxChar;
	char rxBuffer[100];
	APP_WIFLY_MESSAGE txMessage;
} APP_DATA;


// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************

/*******************************************************************************
  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").
  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void app_wifly_UartTx(char* string);
void APP_WIFLY_Initialize ( void );
void app_wifly_UartTxChar( char theChar );

/******************************
  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.
  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_WIFLY_Tasks( void );


#endif /* _APP_H */
