#ifndef _SYSTEM_CLOCK_H_
#define _SYSTEM_CLOCK_H_

#define SYSTEM_CLOCK_MAX    60000   // in milliseconds

typedef uint16_t SYSTEM_CLOCK_T;

extern volatile SYSTEM_CLOCK_T g_system_clock;

void System_Clock_Init( void );

int32_t Clock_Diff(SYSTEM_CLOCK_T time1, SYSTEM_CLOCK_T time2);

#endif
