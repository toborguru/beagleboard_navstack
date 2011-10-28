/** @file
 *  ROS Node which subscribes to the @e "cmd_vel" topic, and passes any
 *  received messages along to all attached listeners.
 */
 
#include <fcntl.h>
#include <ros/ros.h>

#include "TwistEndpoint.hpp"

using namespace diff_drive_core;
 
namespace diff_drive_message_endpoints
{
/** Default Constructor
 */
TwistEndpoint::TwistEndpoint()    
              : _stop_requested(false), 
                _running(false) 
{
  _twist_listeners.reserve(1);
}

/** Destructor
 */
TwistEndpoint::~TwistEndpoint()
{
}

/** Spawns the worker thread to connect and subscribe to ROS topic. 
 */
void TwistEndpoint::Subscribe()
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
void TwistEndpoint::Unsubscribe()
{
  if (_running) 
  {
    _running = false;
    _stop_requested = true;

    // Wait to return until _thread has completed
    pthread_join(_thread, 0);
  }
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  messages when they are available.
 */
void TwistEndpoint::Attach( ITwistListener& twist_listener )
{
  _twist_listeners.push_back(&twist_listener);
}

/** Notifies the endpoint that there there is a new message @p twist.
 */
void TwistEndpoint::NewTwistReceived( const geometry_msgs::Twist& twist )
{
  NotifyTwistListeners( twist );

  ROS_DEBUG(  "Twist received: linear: %f angular %f",
              twist.linear.x, twist.angular.z );
}

/** Worker thread. Subscribes to the ROS topic and checks for ROS or class stop request.
 */
void TwistEndpoint::ReceiveTwistMessages()
{
  ros::Subscriber twist_subscriber = _twist_node.subscribe( "cmd_vel", 
                                                            1, 
                                                            &TwistEndpoint::NewTwistReceived,
                                                            this );

  ros::Rate r(100); // 100 hz

  while (!_stop_requested && ros::ok()) 
  {
    ros::spinOnce();
    r.sleep();
  }

  _running = false;
}

/** When called all attached listeners will be notified and sent a copy of @p encoder_counts.
 */
void TwistEndpoint::NotifyTwistListeners( const geometry_msgs::Twist& twist )
{
  for (int i= 0; i < _twist_listeners.size(); i++)
  {
    _twist_listeners[i]->OnTwistAvailableEvent( twist );
  }
}
}
