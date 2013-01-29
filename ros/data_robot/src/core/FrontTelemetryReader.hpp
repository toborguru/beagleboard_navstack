#ifndef GUARD_FrontTelemetryReader
#define GUARD_FrontTelemetryReader
 
#include <pthread.h>
#include <vector>

#include "Telemetry.hpp"

#include "IFrontTelemetryListener.hpp"
#include "IBusRequestProcessorEndpoint.hpp"
 
namespace data_robot_core
{
class FrontTelemetryReader
{
public:
  FrontTelemetryReader();
  FrontTelemetryReader( IBusRequestProcessorEndpoint *p_external_bus );
  
  ~FrontTelemetryReader();

  void BeginReading();
  void StopReading();

  // Provides an interface to request telemetry from
  void SetExternalBus( IBusRequestProcessorEndpoint *p_external_bus );

  bool GetBlockForBusRequest() const { return _block_for_request; }
  void SetBlockForBusRequest( bool block ) { _block_for_request = block; }

  // Provides a call-back mechanism for objects interested in receiving bumper information 
  void Attach( IFrontTelemetryListener& telemetry_listener );

private:
  void ReadFrontTelemetry();

  void NotifyFrontTelemetryListeners( const FrontShellTelemetry_T& telemetry );

  std::vector<IFrontTelemetryListener*> _telemetry_listeners;

  // Basic threading support as suggested by Jeremy Friesner at
  // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
  volatile bool _stop_requested;
  volatile bool _running;
  pthread_t _thread;

  bool _block_for_request;

  IBusRequestProcessorEndpoint *_p_external_bus;

  static void * ReadFrontTelemetryFunction(void * This) {
    ((FrontTelemetryReader *)This)->ReadFrontTelemetry();
    return 0;
  }
};
}
 
#endif /* GUARD_FrontTelemetryReader */
