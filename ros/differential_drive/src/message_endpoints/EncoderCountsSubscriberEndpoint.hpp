// EncoderCountsSubscriberEndpoint.hpp
 
#ifndef GUARD_EncoderCountsSubscriberEndpoint
#define GUARD_EncoderCountsSubscriberEndpoint
 
#include <ros/ros.h>
#include <pthread.h>
#include <vector>

#include "IEncoderCountsSubscriberEndpoint.hpp"
 
namespace differential_drive_message_endpoints
{
class EncoderCountsSubscriberEndpoint : public differential_drive_core::IEncoderCountsSubscriberEndpoint
{ 
public:
  EncoderCountsSubscriberEndpoint();
  ~EncoderCountsSubscriberEndpoint();

  void Subscribe();
  void Unsubscribe();
  bool IsSubscribed();
  void Attach( differential_drive_core::IEncoderCountsListener& encoder_counts_listener );

  void NewEncoderCountsReceived( const differential_drive::EncoderCounts& encoder_counts );

private:
  void ReceiveEncoderCountsMessages();

  void NotifyEncoderCountsListeners( const differential_drive::EncoderCounts& encoder_counts );

  std::vector<differential_drive_core::IEncoderCountsListener*> _encoder_counts_listeners;

  // Create handle to node
  ros::NodeHandle _encoder_counts_node;

  ros::Subscriber _encoder_counts_subscriber;

  ros::AsyncSpinner _spinner;

  // Basic threading support as suggested by Jeremy Friesner at
  // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
  volatile bool _stopRequested;
  volatile bool _running;

  pthread_t _thread;

  static void * ReceiveEncoderCountsMessagesFunction(void * This) {
    ((EncoderCountsSubscriberEndpoint*)This)->ReceiveEncoderCountsMessages();
    return 0;
  }
};
}
 
#endif /* GUARD_EncoderCountsSubscriberEndpoint */
