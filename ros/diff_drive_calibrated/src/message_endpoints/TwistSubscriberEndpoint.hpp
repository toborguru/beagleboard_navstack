// TwistI2CEndpoint.hpp
 
#ifndef GUARD_TwistSubscriberEndpoint
#define GUARD_TwistSubscriberEndpoint
 
#include <ros/ros.h>
#include <vector>

#include "ITwistSubscriberEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
class TwistSubscriberEndpoint : public diff_drive_core::ITwistSubscriberEndpoint
{ 
public:
  TwistSubscriberEndpoint();
  ~TwistSubscriberEndpoint();

  void subscribe();
  void unsubscribe();
  bool isSubscribed();
  void attach( diff_drive_core::ITwistListener& twist_listener );
  void detach( diff_drive_core::ITwistListener& twist_listener );

  void newTwistReceived( const geometry_msgs::Twist& twist );

private:
//  void receiveTwistMessages();

  void notifyTwistListeners( const geometry_msgs::Twist& twist );

  std::vector<diff_drive_core::ITwistListener*> _twist_listeners;

  bool _is_subscribed;

  // Create handle to node
  ros::NodeHandle _twist_node;

  ros::Subscriber _twist_subscriber;
};
}
 
#endif /* GUARD_TwistSubscriberEndpoint */
