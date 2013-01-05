/** @file
 *  ROS Node which subscribes to the @e "encoder_counts" topic, and passes any
 *  received messages along to all attached listeners.
 */
 
#include <fcntl.h>

#include <ros/ros.h>

#include "EncoderCountsEndpoint.hpp"

using namespace diff_drive_core;
 
namespace diff_drive_message_endpoints
{
/** Default Constructor
 */
EncoderCountsEndpoint::EncoderCountsEndpoint()    
              : _stopRequested(false), 
                _running(false) 
{
  _encoder_counts_listeners.reserve(1);
}

/** Destructor
 */
EncoderCountsEndpoint::~EncoderCountsEndpoint()
{ 
  Unsubscribe();
}

/** Spawns the worker thread to connect and subscribe to ROS topic. 
 */
void EncoderCountsEndpoint::Subscribe()
{
  if (! _running) 
  {
    _running = true;
    _stopRequested = false;
    // Spawn async thread for reading laser scans
    pthread_create(&_thread, 0, ReceiveEncoderCountsMessagesFunction, this);
  }
}

/** Requests thread stop, and joins worker thread.
 */
void EncoderCountsEndpoint::Unsubscribe()
{
  if (_running) 
  {
    _running = false;
    _stopRequested = true;

    // Wait to return until _thread has completed
    pthread_join(_thread, 0);
  }
}

/** Returns the status of the thread.
 */
bool EncoderCountsEndpoint::IsSubscribed()
{
  return _running;
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  messages when they are available.
 */
void EncoderCountsEndpoint::Attach( IEncoderCountsListener& encoder_counts_listener )
{
  _encoder_counts_listeners.push_back(&encoder_counts_listener);
}

/** Notifies the endpoint that there there is a new message @p encoder_counts.
 */
void EncoderCountsEndpoint::NewEncoderCountsReceived( const diff_drive::EncoderCounts& encoder_counts )
{
  NotifyEncoderCountsListeners( encoder_counts );

  ROS_DEBUG_NAMED(  "EncoderCountsEndpoint", "Counts received: left: %d right %d stasis: %d dt: %d",
              encoder_counts.left_count, encoder_counts.right_count, encoder_counts.stasis_count, 
              encoder_counts.dt_ms );
}

/** Worker thread. Subscribes to the ROS topic and checks for ROS or class stop request.
 */
void EncoderCountsEndpoint::ReceiveEncoderCountsMessages()
{
  ros::Subscriber encoder_counts_subscriber = _encoder_counts_node.subscribe( "encoder_counts", 
                                                            1, 
                                                            &EncoderCountsEndpoint::NewEncoderCountsReceived,
                                                            this );

  ros::Rate r(100); // 100 hz

  while (!_stopRequested && ros::ok()) 
  {
    ros::spinOnce();
    r.sleep();
  }

  _running = false;
}

/** When called all attached listeners will be notified and sent a copy of @a encoder_counts.
 */
void EncoderCountsEndpoint::NotifyEncoderCountsListeners( const diff_drive::EncoderCounts& encoder_counts )
{
  for (unsigned int i= 0; i < _encoder_counts_listeners.size(); i++)
  {
    _encoder_counts_listeners[i]->OnEncoderCountsAvailableEvent( encoder_counts );
  }
}
}
