#ifndef GUARD_EncoderCountsReader
#define GUARD_EncoderCountsReader
 
#include <pthread.h>
#include <vector>

#include "diff_drive/EncoderCounts.h"
#include "IEncoderCountsListener.hpp"
#include "IExternalBusInterface.hpp"
 
namespace data_robot_core
{
class EncoderCountsReader
{
public:
  EncoderCountsReader();

  EncoderCountsReader( IExternalBusInterface *p_external_bus );

  void BeginReading();
  void StopReading();

  // Provides an interface to request counts from
  void SetExternalBus( IExternalBusInterface *p_external_bus );

  bool GetBlockForBusRequest() const { return _block_for_request; }
  void SetBlockForBusRequest( bool block ) {_block_for_request = block; }

  // Provides a call-back mechanism for objects interested in receiving encoder counts
  void Attach( IEncoderCountsListener& encoder_counts_listener );

  int32_t  DifferentiateEncoderReading( int32_t old_reading, int32_t new_reading ) const;
  int16_t  DifferentiateEncoderReading( int16_t old_reading, int16_t new_reading ) const;
  int8_t   DifferentiateEncoderReading( int8_t  old_reading,  int8_t new_reading ) const;

private:
  void ReadEncoderCounts();
  void NotifyEncoderCountsListeners( const diff_drive::EncoderCounts& encoder_counts );

  IExternalBusInterface *_p_external_bus;

  std::vector<IEncoderCountsListener*> _encoder_counts_listeners;

  bool _block_for_request;

  // Basic threading support as suggested by Jeremy Friesner at
  // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
  volatile bool _stop_requested;
  volatile bool _running;
  pthread_t _thread;

  static void * ReadEncoderCountsFunction(void * This) {
    ((EncoderCountsReader *)This)->ReadEncoderCounts();
    return 0;
  }
};
}
 
#endif /* GUARD_EncoderCountsReader */
