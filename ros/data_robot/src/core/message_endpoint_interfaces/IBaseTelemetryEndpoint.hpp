// IBaseTelemetryEndpoint.hpp

#ifndef GUARD_IBaseTelemetryEndpoint
#define GUARD_IBaseTelemetryEndpoint

#include "IBaseTelemetryListener.hpp"

namespace data_robot_core
{
class IBaseTelemetryEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IBaseTelemetryEndpoint() {}
  virtual void Subscribe() = 0;
  virtual void Unsubscribe() = 0;
  virtual void Attach( IBaseTelemetryListener& front_telemetry_listener ) = 0; 
};
}

#endif /* GUARD_IBaseTelemetryEndpoint */
