/** @file
 *  ROS Node which subscribes to the @e "encoder_counts" topic, and passes any
 *  received messages along to all attached listeners.
 */
 
#include "EncoderCountsSubscriberEndpoint.hpp"

using namespace differential_drive_core;
 
namespace differential_drive_message_endpoints
{
/** Default Constructor
 */
EncoderCountsSubscriberEndpoint::EncoderCountsSubscriberEndpoint()    
              : _is_subscribed(false)
{
  _encoder_counts_listeners.reserve(1);
}

/** Destructor
 */
EncoderCountsSubscriberEndpoint::~EncoderCountsSubscriberEndpoint()
{ 
}

/** Connect and subscribe to ROS topic. 
 */
void EncoderCountsSubscriberEndpoint::Subscribe()
{ 
  if ( !_is_subscribed )
  {
    _is_subscribed = true;
    _encoder_counts_subscriber = _encoder_counts_node.subscribe( "encoder_counts",
                                                                 10,
                                                                 &EncoderCountsSubscriberEndpoint::NewEncoderCountsReceived,
                                                                 this );  
  }
}

/** Stop processing incoming messages.
 */
void EncoderCountsSubscriberEndpoint::Unsubscribe()
{
  if ( _is_subscribed )
  {
    _is_subscribed = false;
    _encoder_counts_subscriber.shutdown();
  }
}

/** Access function.
 */
bool EncoderCountsSubscriberEndpoint::IsSubscribed()
{
  return _is_subscribed;
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

/** Notifies the endpoint that there there is a new message.
 */
void EncoderCountsSubscriberEndpoint::NewEncoderCountsReceived( const differential_drive::EncoderCounts& encoder_counts )
{
  NotifyEncoderCountsListeners( encoder_counts );

  ROS_DEBUG_NAMED(  "EncoderCountsSubscriberEndpoint", "Counts received: left: %d right %d stasis: %d dt: %d",
              encoder_counts.left_count, encoder_counts.right_count, encoder_counts.stasis_count, 
              encoder_counts.dt_ms );
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
