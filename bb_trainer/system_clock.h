#ifndef _SYSTEM_CLOCK_H_
#define _SYSTEM_CLOCK_H_

#ifndef SYSTEM_CLOCK_MAX
  #define SYSTEM_CLOCK_MAX (65536U)
#endif
#if SYSTEM_CLOCK_MAX < 2
  #error Rx buffer must be a minimum length of 2
#endif
#define SYSTEM_CLOCK_MASK  (SYSTEM_CLOCK_MAX - 1)
#if ( SYSTEM_CLOCK_MAX & SYSTEM_CLOCK_MASK )
  #error Rx buffer size is not a power of 2
#endif

typedef uint16_t SYSTEM_CLOCK_T;

extern volatile SYSTEM_CLOCK_T g_system_clock;

void System_Clock_Init( void );

int32_t Clock_Diff(SYSTEM_CLOCK_T time1, SYSTEM_CLOCK_T time2);

#endif
