#include "PowerStateProcessor.hpp"

namespace data_robot_core
{
/** Default constructor.
 */
PowerStateProcessor::PowerStateProcessor()
                    : _current( 0.0 ),
                      _voltage( 0.0 )
{
}

/** Function to 
 */
void PowerStateProcessor::AddNewData( uint16_t current,  uint16_t voltage ) 
{
  _current = (float)current;
  _voltage = (float)voltage;

  _power_state.battery_voltage = _voltage;
  _power_state.current_draw = _current;
}

data_robot::PowerState PowerStateProcessor::GetPowerState()
{
  return _power_state;
}
}
