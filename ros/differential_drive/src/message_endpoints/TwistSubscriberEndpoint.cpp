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
              : _stop_requested(false), 
                _running(false) 
{
  _twist_listeners.reserve(1);
}

/** Destructor
 */
TwistSubscriberEndpoint::~TwistSubscriberEndpoint()
{
  Unsubscribe();
}

/** Spawns the worker thread to connect and subscribe to ROS topic. 
 */
void TwistSubscriberEndpoint::Subscribe()
{
  if (! _running) 
  {
    _running = true;
    _stop_requested = false;

    // Spawn async thread for reading laser scans
    pthread_create(&_thread, 0, ReceiveTwistMessagesFunction, this);
  }
}

/** Requests thread stop, and joins worker thread.
 */
void TwistSubscriberEndpoint::Unsubscribe()
{
  if (_running) 
  {
    _running = false;
    _stop_requested = true;

    // Wait to return until _thread has completed
    pthread_join(_thread, 0);
  }
}

/** Returns the thread status.
 */
bool TwistSubscriberEndpoint::IsSubscribed()
{
  return _running;
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  messages when they are available.
 */
void TwistSubscriberEndpoint::Attach( ITwistListener& twist_listener )
{
  _twist_listeners.push_back(&twist_listener);
}

/** Allows a listener to stop receiving call-backs. If this is the last listener
 *  the class will automatically call Unsubscribe.
 */
void TwistSubscriberEndpoint::Detach( ITwistListener& twist_listener )
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
  NotifyTwistListeners( twist );

  ROS_DEBUG(  "Twist received: linear: %f angular %f",
              twist.linear.x, twist.angular.z );
}

/** Worker thread. Subscribes to the ROS topic and checks for ROS or class stop request.
 */
void TwistSubscriberEndpoint::ReceiveTwistMessages()
{
  ros::Subscriber twist_subscriber = _twist_node.subscribe( "cmd_vel", 
                                                            1, 
                                                            &TwistSubscriberEndpoint::NewTwistReceived,
                                                            this );

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate r(5);
  while (!_stop_requested && ros::ok()) 
  {
    // Sleep most of the time
    r.sleep();
  }

  spinner.stop();
}

/** When called all attached listeners will be notified and sent a copy of @p encoder_counts.
 */
void TwistSubscriberEndpoint::NotifyTwistListeners( const geometry_msgs::Twist& twist )
{
  for (unsigned int i= 0; i < _twist_listeners.size(); i++)
  {
    _twist_listeners[i]->OnTwistAvailableEvent( twist );
  }
}
}
