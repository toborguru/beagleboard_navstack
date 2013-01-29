// TwistI2CEndpoint.hpp
 
#ifndef GUARD_TwistSubscriberEndpoint
#define GUARD_TwistSubscriberEndpoint
 
#include <ros/ros.h>
#include <pthread.h>
#include <vector>

#include "ITwistSubscriberEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
class TwistSubscriberEndpoint : public diff_drive_core::ITwistSubscriberEndpoint
{ 
public:
  TwistSubscriberEndpoint();
  ~TwistSubscriberEndpoint();

  void Subscribe();
  void Unsubscribe();
  bool IsSubscribed();
  void Attach( diff_drive_core::ITwistListener& twist_listener );

  void NewTwistReceived( const geometry_msgs::Twist& twist );

private:
  void ReceiveTwistMessages();

  void NotifyTwistListeners( const geometry_msgs::Twist& twist );

  std::vector<diff_drive_core::ITwistListener*> _twist_listeners;

  // Create handle to node
  ros::NodeHandle _twist_node;

  ros::Subscriber _twist_subscriber;

  // Basic threading support as suggested by Jeremy Friesner at
  // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
  volatile bool _stop_requested;
  volatile bool _running;

  pthread_t _thread;

  static void * ReceiveTwistMessagesFunction(void * This) {
    ((TwistSubscriberEndpoint*)This)->ReceiveTwistMessages();
    return 0;
  }

};
}
 
#endif /* GUARD_TwistSubscriberEndpoint */
