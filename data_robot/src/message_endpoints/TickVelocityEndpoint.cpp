/** @file
 *  ROS Node which subscribes to the @e "cmd_vel" topic, and passes any
 *  received messages along to all attached listeners.
 */
 
#include <fcntl.h>
#include <ros/ros.h>

#include "TickVelocityEndpoint.hpp"

using namespace data_robot_core;
 
namespace data_robot_message_endpoints
{
/** Default Constructor
 */
TickVelocityEndpoint::TickVelocityEndpoint()    
              : _stop_requested(false), 
                _running(false) 
{
  _tick_velocity_listeners.reserve(1);
}

/** Destructor
 */
TickVelocityEndpoint::~TickVelocityEndpoint()
{
}

/** Spawns the worker thread to connect and subscribe to ROS topic. 
 */
void TickVelocityEndpoint::Subscribe()
{
  if (! _running) 
  {
    _running = true;
    _stop_requested = false;
    // Spawn async thread for reading laser scans
    pthread_create(&_thread, 0, ReceiveTickVelocityMessagesFunction, this);
  }
}

/** Requests thread stop, and joins worker thread.
 */
void TickVelocityEndpoint::Unsubscribe()
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
void TickVelocityEndpoint::Attach( ITickVelocityListener& tick_velocity_listener )
{
  _tick_velocity_listeners.push_back(&tick_velocity_listener);
}

/** Notifies the endpoint that there there is a new message @p tick_velocity.
 */
void TickVelocityEndpoint::NewTickVelocityReceived( const diff_drive::TickVelocity& tick_velocity )
{
  NotifyTickVelocityListeners( tick_velocity );

  ROS_DEBUG(  "TickVelocity received: linear: %d angular %d",
              tick_velocity.linear_ticks_sec, tick_velocity.angular_ticks_sec );
}

/** Worker thread. Subscribes to the ROS topic and checks for ROS or class stop request.
 */
void TickVelocityEndpoint::ReceiveTickVelocityMessages()
{
  ros::Subscriber tick_velocity_subscriber = _tick_velocity_node.subscribe( "cmd_vel", 
                                                            1, 
                                                            &TickVelocityEndpoint::NewTickVelocityReceived,
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
void TickVelocityEndpoint::NotifyTickVelocityListeners( const diff_drive::TickVelocity& tick_velocity )
{
  for (int i= 0; i < _tick_velocity_listeners.size(); i++)
  {
    _tick_velocity_listeners[i]->OnTickVelocityAvailableEvent( tick_velocity );
  }
}
}
