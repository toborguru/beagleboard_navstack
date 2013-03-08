// TwistI2CEndpoint.hpp
 
#ifndef GUARD_TwistSubscriberEndpoint
#define GUARD_TwistSubscriberEndpoint
 
#include <ros/ros.h>
#include <vector>

#include "ITwistSubscriberEndpoint.hpp"
 
namespace differential_drive_message_endpoints
{
class TwistSubscriberEndpoint : public differential_drive_core::ITwistSubscriberEndpoint
{ 
public:
  TwistSubscriberEndpoint();
  ~TwistSubscriberEndpoint();

  void Subscribe();
  void Unsubscribe();
  bool IsSubscribed();
  void Attach( differential_drive_core::ITwistListener& twist_listener );
  void Detach( differential_drive_core::ITwistListener& twist_listener );

  void NewTwistReceived( const geometry_msgs::Twist& twist );

private:
  void ReceiveTwistMessages();

  void NotifyTwistListeners( const geometry_msgs::Twist& twist );

  std::vector<differential_drive_core::ITwistListener*> _twist_listeners;

  bool _is_subscribed;

  // Create handle to node
  ros::NodeHandle _twist_node;

  ros::Subscriber _twist_subscriber;
};
}
 
#endif /* GUARD_TwistSubscriberEndpoint */
