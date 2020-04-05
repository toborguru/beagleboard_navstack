/** @file
 *  ROS Node which subscribes to the @e "encoder_counts" topic, and passes any
 *  received messages along to all attached listeners.
 */
 
#include "EncoderCountsSubscriberEndpoint.hpp"

using namespace diff_drive_core;
 
namespace diff_drive_message_endpoints
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
void EncoderCountsSubscriberEndpoint::subscribe()
{ 
  if ( !_is_subscribed )
  {
    _is_subscribed = true;
    _encoder_counts_subscriber = _encoder_counts_node.subscribe( "encoder_counts",
                                                                 10,
                                                                 &EncoderCountsSubscriberEndpoint::newEncoderCountsReceived,
                                                                 this );  
  }
}

/** Stop processing incoming messages.
 */
void EncoderCountsSubscriberEndpoint::unsubscribe()
{
  if ( _is_subscribed )
  {
    _is_subscribed = false;
    _encoder_counts_subscriber.shutdown();
  }
}

/** Access function.
 */
bool EncoderCountsSubscriberEndpoint::isSubscribed()
{
  return _is_subscribed;
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  messages when they are available.
 */
void EncoderCountsSubscriberEndpoint::attach( IEncoderCountsListener& encoder_counts_listener )
{
  _encoder_counts_listeners.push_back(&encoder_counts_listener);

  // If this is the first one to attach, automatically subscribe.
  if ( !_is_subscribed && (_encoder_counts_listeners.size() == 1) )
  {
    subscribe();
  }
}

/** Allows a listener to stop receiving call-backs. If this is the last listener
 *  the class will automatically call unsubscribe.
 */
void EncoderCountsSubscriberEndpoint::detach( IEncoderCountsListener& encoder_counts_listener )
{
  // Using the remove-erase idiom
  std::vector<IEncoderCountsListener*>& vec = _encoder_counts_listeners; // use shorter name
  vec.erase( std::remove(vec.begin(), vec.end(), &encoder_counts_listener), vec.end() );

  if ( _encoder_counts_listeners.size() == 0 )
  {
    unsubscribe();
  }
}

/** Notifies the endpoint that there there is a new message.
 */
void EncoderCountsSubscriberEndpoint::newEncoderCountsReceived( const diff_drive_calibrated::EncoderCounts& encoder_counts )
{
  notifyEncoderCountsListeners( encoder_counts );

  ROS_DEBUG_NAMED(  "EncoderCountsSubscriberEndpoint", "Counts received: left: %d right %d stasis: %d dt: %d",
              encoder_counts.left_count, encoder_counts.right_count, encoder_counts.stasis_count, 
              encoder_counts.dt_ms );
}

/** When called all attached listeners will be notified and sent a copy of @a encoder_counts.
 */
void EncoderCountsSubscriberEndpoint::notifyEncoderCountsListeners( const diff_drive_calibrated::EncoderCounts& encoder_counts )
{
  for (unsigned int i= 0; i < _encoder_counts_listeners.size(); ++i)
  {
    _encoder_counts_listeners[i]->onEncoderCountsAvailableEvent( encoder_counts );
  }
}
}
