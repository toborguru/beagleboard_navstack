/*
 * Copyright Chorus Motors
 * Author: Sawyer Larkin
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "analog_in.h"
#include "macros.h"

volatile uint16_t g_analog_values[ANALOG_NUM_CHANNELS];

// INTERRUPT SERVICE ROUTINES

ISR(ADC_vect)
{
    static const uint8_t admux_settings = BIT(REFS0);

    static uint8_t current_channel = 0;

    // Store reading
    g_analog_values[current_channel] = ADC;

    // Increment channel for next reading
    ++current_channel;

    if ( current_channel >= ANALOG_NUM_CHANNELS )
    {
      current_channel = 0;
    }

    // Set new channel first so that it takes effect for next reading.
    ADMUX = admux_settings + current_channel + ANALOG_CHANNEL_FIRST;

    BIT_SET(ADCSRA, ADSC);
}

// Initialize the Analog to Digital Conversion port
void AnalogInInit()
{
    BIT_CLEAR(PRR, PRADC);

    BITS_SET(ADCSRA, BIT(ADEN)  |   // ADC Enable
                     BIT(ADIE)  |   // ADC Interupt Enable
                     BIT(ADSC)  |   // ADC Start Conversion (need to trigger the first reading)
                     BIT(ADPS2) |   // Clk Div 128
                     BIT(ADPS1) |
                     BIT(ADPS0));

    BITS_SET(DIDR0, 0x06);          // Disable the digital I/O on used analog pins
}
