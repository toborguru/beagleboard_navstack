// PowerStateProcessor.hpp
 
#ifndef GUARD_PowerStateProcessor
#define GUARD_PowerStateProcessor

#include "data_robot/PowerState.h"

#define AVERAGE_2N_READINGS   4

#define AVERAGE_NUM_READINGS  (1<<AVERAGE_2N_READINGS)

namespace data_robot_core
{ 
class PowerStateProcessor
{
public:

  PowerStateProcessor();

  void AddNewData( uint16_t current,  uint16_t voltage, uint16_t milli_seconds );

  data_robot::PowerState GetPowerState();

private:
  data_robot::PowerState _power_state;

  float _avg_current;
  float _avg_voltage;

  int _avg_index;

  uint16_t  _currents[ AVERAGE_NUM_READINGS ];
  uint16_t  _voltages[ AVERAGE_NUM_READINGS ];
};
}
 
#endif /* GUARD_PowerStateProcessor */
