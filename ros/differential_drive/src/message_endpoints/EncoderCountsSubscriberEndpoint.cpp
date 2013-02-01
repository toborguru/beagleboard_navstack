/** @file
 *  ROS Node which subscribes to the @e "encoder_counts" topic, and passes any
 *  received messages along to all attached listeners.
 */
 
#include "EncoderCountsSubscriberEndpoint.hpp"
#include "AsyncSpinnerWrapper.hpp"

using namespace differential_drive_core;
 
namespace differential_drive_message_endpoints
{
/** Default Constructor
 */
EncoderCountsSubscriberEndpoint::EncoderCountsSubscriberEndpoint()    
              : _stopRequested(false), 
                _running(false) 
{
  _encoder_counts_listeners.reserve(1);
}

/** Destructor
 */
EncoderCountsSubscriberEndpoint::~EncoderCountsSubscriberEndpoint()
{ 
  Unsubscribe();
}

/** Spawns the worker thread to connect and subscribe to ROS topic. 
 */
void EncoderCountsSubscriberEndpoint::Subscribe()
{
  if (! _running) 
  {
    _running = true;
    _stopRequested = false;

    // Spawn async worker thread
    pthread_create(&_thread, 0, ReceiveEncoderCountsMessagesFunction, this);
  }
}

/** Requests thread stop, and joins worker thread.
 */
void EncoderCountsSubscriberEndpoint::Unsubscribe()
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
bool EncoderCountsSubscriberEndpoint::IsSubscribed()
{
  return _running;
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  messages when they are available.
 */
void EncoderCountsSubscriberEndpoint::Attach( IEncoderCountsListener& encoder_counts_listener )
{
  _encoder_counts_listeners.push_back(&encoder_counts_listener);
}

/** Allows a listener to stop receiving call-backs. If this is the last listener
 *  the class will automatically call Unsubscribe.
 */
void EncoderCountsSubscriberEndpoint::Detach( IEncoderCountsListener& encoder_counts_listener )
{
  // Using the remove-erase idiom
  std::vector<IEncoderCountsListener*>& vec = _encoder_counts_listeners; // use shorter name
  vec.erase( std::remove(vec.begin(), vec.end(), &encoder_counts_listener), vec.end() );

  if ( _encoder_counts_listeners.size() == 0 )
  {
    Unsubscribe();
  }
}

/** Notifies the endpoint that there there is a new message @p encoder_counts.
 */
void EncoderCountsSubscriberEndpoint::NewEncoderCountsReceived( const differential_drive::EncoderCounts& encoder_counts )
{
  NotifyEncoderCountsListeners( encoder_counts );

  ROS_DEBUG_NAMED(  "EncoderCountsSubscriberEndpoint", "Counts received: left: %d right %d stasis: %d dt: %d",
              encoder_counts.left_count, encoder_counts.right_count, encoder_counts.stasis_count, 
              encoder_counts.dt_ms );
}

/** Worker thread. Subscribes to the ROS topic and checks for ROS or class stop request.
 */
void EncoderCountsSubscriberEndpoint::ReceiveEncoderCountsMessages()
{
  ros::Subscriber encoder_counts_subscriber = _encoder_counts_node.subscribe( "encoder_counts", 
                                                            1, 
                                                            &EncoderCountsSubscriberEndpoint::NewEncoderCountsReceived,
                                                            this );

  AsyncSpinnerWrapper spinner;

  spinner.Start();

  ros::Rate r(5);
  while (!_stopRequested && ros::ok()) 
  {
    // Sleep most of the time
    r.sleep();
  }

  spinner.Stop();
}

/** When called all attached listeners will be notified and sent a copy of @a encoder_counts.
 */
void EncoderCountsSubscriberEndpoint::NotifyEncoderCountsListeners( const differential_drive::EncoderCounts& encoder_counts )
{
  for (unsigned int i= 0; i < _encoder_counts_listeners.size(); i++)
  {
    _encoder_counts_listeners[i]->OnEncoderCountsAvailableEvent( encoder_counts );
  }
}
}
