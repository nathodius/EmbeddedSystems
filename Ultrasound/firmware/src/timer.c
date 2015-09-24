#include "timer.h"

void theTimerInit(int msCount)
{
    count =0;
	TickType_t timertime = (msCount/portTICK_PERIOD_MS);
	TimerHandle_t theTimer = xTimerCreate("theTimer", timertime , pdTRUE, 0, theTimerCallback );
	if( xTimerStart(theTimer, 0) != pdPASS )
	{
		debugU("Timer failed to start");
	}

}

void theTimerCallback(TimerHandle_t pxTimer)
{
    count++;
	sendQApp1(1);
}

int returnCount()
{
    return count;
}
