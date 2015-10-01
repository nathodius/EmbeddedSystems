/* 
 * File:   app_wifly_public.h
 * Author: lucun_000
 *
 * Created on September 23, 2015, 6:58 PM
 */

	
void app_wifly_sendmsg(int type, char* string);
void app_wifly_sendEchoChar(char theChar);


#ifndef APP_WIFLY_PUBLIC_H
#define	APP_WIFLY_PUBLIC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "FreeRTOS.h"


#ifdef	__cplusplus
extern "C" {
#endif


typedef struct
{
	int type;
	char* string;
} APP_WIFLY_MESSAGE;


#ifdef	__cplusplus
}
#endif

#endif	/* APP_WIFLY_PUBLIC_H */

