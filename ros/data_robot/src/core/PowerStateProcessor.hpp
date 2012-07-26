// PowerStateProcessor.hpp
 
#ifndef GUARD_PowerStateProcessor
#define GUARD_PowerStateProcessor

#include "data_robot/PowerState.h"

namespace data_robot_core
{ 
class PowerStateProcessor
{
public:

  PowerStateProcessor();

  void AddNewData( uint16_t current,  uint16_t voltage );

  data_robot::PowerState GetPowerState();

private:
  data_robot::PowerState _power_state;

  float _voltage;
  float _current;
};
}
 
#endif /* GUARD_PowerStateProcessor */
