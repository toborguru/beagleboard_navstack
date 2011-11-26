#ifndef GUARD_BaseTelemetryReader
#define GUARD_BaseTelemetryReader
 
#include <pthread.h>
#include <vector>

#include "IBaseTelemetryListener.hpp"
#include "IExternalBusEndpoint.hpp"

#include "Telemetry.hpp"

namespace data_robot_core
{
class BaseTelemetryReader
{
public:
  BaseTelemetryReader();
  BaseTelemetryReader( IExternalBusEndpoint *p_external_bus );
  
  ~BaseTelemetryReader();

  void BeginReading();
  void StopReading();

  // Provides an interface to request counts from
  void SetExternalBus( IExternalBusEndpoint *p_external_bus );

  bool GetBlockForBusRequest() const { return _block_for_request; }
  void SetBlockForBusRequest( bool block ) { _block_for_request = block; }

  // Provides a call-back mechanism for objects interested in receiving encoder counts
  void Attach( IBaseTelemetryListener& telemetry_listener );

private:
  void ReadBaseTelemetry();

  void NotifyBaseTelemetryListeners( const BaseTelemetry_T& telemetry );

  IExternalBusEndpoint *_p_external_bus;

  std::vector<IBaseTelemetryListener*> _telemetry_listeners;

  bool _block_for_request;

  // Basic threading support as suggested by Jeremy Friesner at
  // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
  volatile bool _stop_requested;
  volatile bool _running;
  pthread_t _thread;

  static void * ReadBaseTelemetryFunction(void * This) {
    ((BaseTelemetryReader *)This)->ReadBaseTelemetry();
    return 0;
  }
};
}
 
#endif /* GUARD_BaseTelemetryReader */