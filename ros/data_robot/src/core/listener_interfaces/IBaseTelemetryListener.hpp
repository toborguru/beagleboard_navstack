// IBaseTelemetryListener.hpp
 
#ifndef GUARD_IBaseTelemetryListener
#define GUARD_IBaseTelemetryListener

#include "Telemetry.hpp"
 
namespace data_robot_core
{
  class IBaseTelemetryListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IBaseTelemetryListener() {}
      virtual void OnBaseTelemetryAvailableEvent( const BaseTelemetry_T& telemetry ) = 0;
  };
}
 
#endif /* GUARD_IBaseTelemetryListener */
