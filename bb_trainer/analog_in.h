
#define ANALOG_CHANNEL_FIRST  1
#define ANALOG_CHANNEL_LAST   2
#define ANALOG_NUM_CHANNELS   2

#define ANALOG_VOLTAGE_INDEX  0
#define ANALOG_CURRENT_INDEX  1

extern volatile uint16_t g_analog_values[ANALOG_NUM_CHANNELS];

void AnalogInInit( void );
