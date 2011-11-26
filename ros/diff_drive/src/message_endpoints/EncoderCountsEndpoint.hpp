// EncoderCountsEndpoint.hpp
 
#ifndef GUARD_EncoderCountsEndpoint
#define GUARD_EncoderCountsEndpoint
 
#include <ros/ros.h>
#include <pthread.h>
#include <vector>

#include "IEncoderCountsEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
class EncoderCountsEndpoint : public diff_drive_core::IEncoderCountsEndpoint
{ 
public:
  EncoderCountsEndpoint();
  ~EncoderCountsEndpoint();

  void Subscribe();
  void Unsubscribe();
  void Attach( diff_drive_core::IEncoderCountsListener& encoder_counts_listener );

  void NewEncoderCountsReceived( const diff_drive::EncoderCounts& encoder_counts );

private:
  void ReceiveEncoderCountsMessages();

  void NotifyEncoderCountsListeners( const diff_drive::EncoderCounts& encoder_counts );

  std::vector<diff_drive_core::IEncoderCountsListener*> _encoder_counts_listeners;

  // Create handle to node
  ros::NodeHandle _encoder_counts_node;

  ros::Subscriber _encoder_counts_subscriber;

  // Basic threading support as suggested by Jeremy Friesner at
  // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
  volatile bool _stopRequested;
  volatile bool _running;

  pthread_t _thread;

  static void * ReceiveEncoderCountsMessagesFunction(void * This) {
    ((EncoderCountsEndpoint*)This)->ReceiveEncoderCountsMessages();
    return 0;
  }
};
}
 
#endif /* GUARD_EncoderCountsEndpoint */
