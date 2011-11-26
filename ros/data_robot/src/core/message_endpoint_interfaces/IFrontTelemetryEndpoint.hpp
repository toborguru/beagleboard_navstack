// IFrontTelemetryEndpoint.hpp

#ifndef GUARD_IFrontTelemetryEndpoint
#define GUARD_IFrontTelemetryEndpoint

#include "IFrontTelemetryListener.hpp"

namespace data_robot_core
{
class IFrontTelemetryEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IFrontTelemetryEndpoint() {}
  virtual void Subscribe() = 0;
  virtual void Unsubscribe() = 0;
  virtual void Attach( IFrontTelemetryListener& front_telemetry_listener ) = 0; 
};
}

#endif /* GUARD_IFrontTelemetryEndpoint */
