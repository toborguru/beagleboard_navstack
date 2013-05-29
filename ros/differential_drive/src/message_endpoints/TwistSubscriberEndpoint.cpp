/** @file
 *  ROS Node which subscribes to the @e "cmd_vel" topic, and passes any
 *  received messages along to all attached listeners.
 */
 
#include <fcntl.h>
#include <ros/ros.h>

#include "TwistSubscriberEndpoint.hpp"

using namespace differential_drive_core;
 
namespace differential_drive_message_endpoints
{
/** Default Constructor
 */
TwistSubscriberEndpoint::TwistSubscriberEndpoint()    
              : _is_subscribed(false) 
{
  _twist_listeners.reserve(1);
}

/** Destructor
 */
TwistSubscriberEndpoint::~TwistSubscriberEndpoint()
{
  Unsubscribe();
}

/** Connect and subscribe to ROS topic. 
 */
void TwistSubscriberEndpoint::Subscribe()
{
  if (! _is_subscribed) 
  {
    _is_subscribed = true;
    _twist_subscriber = _twist_node.subscribe( "cmd_vel", 
                                               1, 
                                               &TwistSubscriberEndpoint::NewTwistReceived,
                                               this );
  }
}

/** Stop processing incoming messages.
 */
void TwistSubscriberEndpoint::Unsubscribe()
{
  if (_is_subscribed) 
  {
    _is_subscribed = false;
    _twist_subscriber.shutdown();
  }
}

/** Access Function.
 */
bool TwistSubscriberEndpoint::IsSubscribed()
{
  return _is_subscribed;
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  messages when they are available.
 */
void TwistSubscriberEndpoint::attach( ITwistListener& twist_listener )
{
  _twist_listeners.push_back(&twist_listener);

  // If this is the first one to attach, automatically subscribe.
  if ( !_is_subscribed && (_twist_listeners.size() == 1) )
  {
    Subscribe();
  }
}

/** Allows a listener to stop receiving call-backs. If this is the last listener
 *  the class will automatically call Unsubscribe.
 */
void TwistSubscriberEndpoint::detach( ITwistListener& twist_listener )
{
  // Using the remove-erase idiom
  std::vector<ITwistListener*>& vec = _twist_listeners; // use shorter name
  vec.erase( std::remove(vec.begin(), vec.end(), &twist_listener), vec.end() );

  if ( _twist_listeners.size() == 0 )
  {
    Unsubscribe();
  }
}

/** Notifies the endpoint that there there is a new message @p twist.
 */
void TwistSubscriberEndpoint::NewTwistReceived( const geometry_msgs::Twist& twist )
{
  notifyTwistListeners( twist );

  ROS_DEBUG(  "Twist received: linear: %f angular %f",
              twist.linear.x, twist.angular.z );
}

/** When called all attached listeners will be notified and sent a copy of @p encoder_counts.
 */
void TwistSubscriberEndpoint::notifyTwistListeners( const geometry_msgs::Twist& twist )
{
  for (unsigned int i= 0; i < _twist_listeners.size(); ++i)
  {
    _twist_listeners[i]->onTwistAvailableEvent( twist );
  }
}
}
