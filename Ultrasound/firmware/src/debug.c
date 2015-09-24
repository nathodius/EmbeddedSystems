#include "debug.h"

void debugCharInit()
{
	PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_A, 0x0070);
	PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_D, 0x2280);
	PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, 0x0003);

}

//output character
void debugChar(int asciiLetter)
{
	PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_4, (asciiLetter & 0x0001));
	PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_5, ((asciiLetter >> 1) & 0x0001));
	PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_9, ((asciiLetter >> 2) & 0x0001));
	PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_13, ((asciiLetter >> 3) & 0x0001));
	PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_7, ((asciiLetter >> 4) & 0x0001));
	PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, ((asciiLetter >> 5) & 0x0001));
	PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_0, ((asciiLetter >> 6) & 0x0001));
	PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_6, ((asciiLetter >> 7) & 0x0001));
}

//USART0 Driver is connected to USART ID 2
void initDebugU()
{
	DRV_USART0_Initialize();
}


void debugU(char* debugMessage)
{
	int i = 0;
	char test = ' ';
	while(i < debugMessage[i] != 0)
	{
		test = debugMessage[i];
		DRV_USART0_WriteByte(test);
		i++;
	}		
}
