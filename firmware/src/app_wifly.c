
#include "app_wifly.h"
//#include "app_wifly_public.h"

APP_DATA appData;


// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************

void app_wifly_sendmsg(int type, char* string)
{
	APP_WIFLY_MESSAGE theMessage;
	theMessage.type = type;
	theMessage.string = string;
	if(appData.theQueue != 0)
	{
		xQueueSend(appData.theQueue, (void*)&(theMessage), portMAX_DELAY);	
	}
}
void app_wifly_sendEchoChar(char theChar)
{
	if(appData.theQueue != 0)
		xQueueSendFromISR(appData.theQueue, (void*)&(theChar), 0);	
}

void app_wifly_UartTx( char* string )
{
	int i = 0;
	char getChar = ' ';
	while(i < string[i] != 0)
	{
		getChar = string[i];
		DRV_USART1_WriteByte(getChar);
		i++;
	}		
}

////####################
//put transmit in interrupt handler. interrupt when queue has room/can send and then pull characters
//the interrupt handly checks its own queue so tha tother threads can use it. 
//make a seperate transmit app file for this... not in system_interrupt.c
//when i want to send a message, enable the interrupt. turn off interrupt when there are no messages in thequeue.
//try to avoid blocks as much as possible for ISRs. also prevents blockings on xmit
void app_wifly_UartTxChar( char theChar )
{
	DRV_USART1_WriteByte(theChar);
}

char* app_wifly_UartRx()
{
	
}

// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
void APP_WIFLY_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
	appData.rxMessage.type = 0;
	appData.rxMessage.string = " ";
	appData.txMessage.type = 0;
	appData.txMessage.string = " ";
	appData.rxBufferIndex = 0;
	appData.rxChar = ' ';
	char test = ' ';
	appData.rxBufferSize = 100;
//	appData.rxBuffer;
	appData.theQueue = xQueueCreate(10, sizeof(char)); //sizeof(appData.rxMessage));
	if(appData.theQueue == 0)
	{
		//failed to create queue
	}
	DRV_USART1_Initialize();
}


void APP_WIFLY_Tasks ( void )
{
	while(1)
	{
//		debugChar(0x22);
//		app_wifly_sendmsg(2,"TEST");
		//check if queue exists
		if(appData.theQueue != 0)	
		{
			//receive a message and store into rxMessage ... block 5 ticks if empty queue
			if(xQueuePeek(appData.theQueue, &(appData.rxChar), portMAX_DELAY ))
			{
				xQueueReceive(appData.theQueue, &(appData.rxChar), portMAX_DELAY );
				//received messageQ message will tell use the state which will be used
				//##appData.state = appData.rxMessage.type;
				appData.state = 2;
				
				//run the state according to the message's state
				switch ( appData.state )
				{
					/* Application's initial state. */
					case APP_STATE_INIT:	//0
						break;

					case APP_STATE_READ:	//1
					{
						debugU("Read");
						debugChar(0x0F);
						app_wifly_UartRx();
						appData.state = 0;
						app_wifly_sendmsg(2,appData.rxBuffer);
						appData.rxMessage.type = 0;
						appData.rxMessage.string = "";
						break;
					}

					case APP_STATE_WRITE:	//2
					{
						debugU("Write");
						debugChar(0xF0);
						app_wifly_UartTxChar(appData.rxChar);
						appData.state = 0;
						appData.rxMessage.type = 0;
						appData.rxMessage.string = "";
						//appData.rxChar = ' ';
						break;
					}
					case APP_STATE_RECEIVE:	//3
					{
						debugU("Receiving");
						debugChar(appData.rxBufferIndex);
						if(appData.rxBufferIndex < appData.rxBufferSize)	//if we are within buffer bounds, copy char
							appData.rxBuffer[appData.rxBufferIndex] = appData.rxMessage.string[0];
						appData.rxBufferIndex++;
						appData.state = 0;
						appData.rxMessage.type = 0;
						appData.rxMessage.string = "";
						break;
					}

					/* The default state should never be executed. */
					default:
					{
						/* TODO: Handle error in application's state machine. */
						break;
					}
				}//end of case
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
 
