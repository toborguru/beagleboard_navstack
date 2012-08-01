#include "PowerStateProcessor.hpp"

// f(x) = Mx + B
#define CURRENT_M 0.03759
#define CURRENT_B (-3.8492)

// f(x) = Mx + B
#define VOLTAGE_M 0.02
#define VOLTAGE_B 0.0

namespace data_robot_core
{
/** Default constructor.
 */
PowerStateProcessor::PowerStateProcessor()
                    : _avg_current( 0.0 ),
                      _avg_voltage( 0.0 ),
                      _avg_index( 0 )
{
}

/** Function to 
 */
void PowerStateProcessor::AddNewData( uint16_t current,  uint16_t voltage, uint16_t milli_seconds ) 
{
  int32_t total_current = 0;
  int32_t total_voltage = 0;

  int32_t avg_current;
  int32_t avg_voltage;

  _currents[ _avg_index ] = current;
  _voltages[ _avg_index ] = voltage;

  for ( int i = 0; i < AVERAGE_NUM_READINGS; i++ )
  {
    total_current += _currents[ i ];
    total_voltage += _voltages[ i ];
  }
  
  _avg_index += 1;
  _avg_index %= AVERAGE_NUM_READINGS;

  avg_current = total_current >> AVERAGE_2N_READINGS;
  avg_voltage = total_voltage >> AVERAGE_2N_READINGS;

  _power_state.current_draw = ( CURRENT_M * (float) avg_current ) + CURRENT_B;
  _power_state.battery_voltage = (VOLTAGE_M * (float) avg_voltage ) + VOLTAGE_B;
  _power_state.power_draw = _power_state.current_draw * _power_state.battery_voltage;
}

data_robot::PowerState PowerStateProcessor::GetPowerState()
{
  return _power_state;
}
}
