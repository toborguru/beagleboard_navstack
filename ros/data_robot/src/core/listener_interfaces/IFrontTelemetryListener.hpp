// IFrontTelemetryListener.hpp
 
#ifndef GUARD_IFrontTelemetryListener
#define GUARD_IFrontTelemetryListener

#include "Telemetry.hpp"
 
namespace data_robot_core
{
  class IFrontTelemetryListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IFrontTelemetryListener() {}
      virtual void OnFrontTelemetryAvailableEvent( const FrontShellTelemetry_T& telemetry ) = 0;
  };
}
 
#endif /* GUARD_IFrontTelemetryListener */
