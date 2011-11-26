#ifndef GUARD_FrontTelemetryReader
#define GUARD_FrontTelemetryReader
 
#include <pthread.h>
#include <vector>

#include "Telemetry.hpp"

#include "IFrontTelemetryListener.hpp"
#include "IExternalBusEndpoint.hpp"
 
namespace data_robot_core
{
class FrontTelemetryReader
{
public:
  FrontTelemetryReader();
  FrontTelemetryReader( IExternalBusEndpoint *p_external_bus );
  
  ~FrontTelemetryReader();

  void BeginReading();
  void StopReading();

  // Provides an interface to request telemetry from
  void SetExternalBus( IExternalBusEndpoint *p_external_bus );

  bool GetBlockForBusRequest() const { return _block_for_request; }
  void SetBlockForBusRequest( bool block ) { _block_for_request = block; }

  // Provides a call-back mechanism for objects interested in receiving bumper information 
  void Attach( IFrontTelemetryListener& telemetry_listener );

private:
  void ReadFrontTelemetry();

  void NotifyFrontTelemetryListeners( const FrontShellTelemetry_T& telemetry );

  IExternalBusEndpoint *_p_external_bus;

  std::vector<IFrontTelemetryListener*> _telemetry_listeners;

  bool _block_for_request;

  // Basic threading support as suggested by Jeremy Friesner at
  // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
  volatile bool _stop_requested;
  volatile bool _running;
  pthread_t _thread;

  static void * ReadFrontTelemetryFunction(void * This) {
    ((FrontTelemetryReader *)This)->ReadFrontTelemetry();
    return 0;
  }
};
}
 
#endif /* GUARD_FrontTelemetryReader */
