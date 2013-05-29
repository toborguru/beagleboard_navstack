// EncoderCountsSubscriberEndpoint.hpp
 
#ifndef GUARD_EncoderCountsSubscriberEndpoint
#define GUARD_EncoderCountsSubscriberEndpoint
 
#include <ros/ros.h>
#include <vector>

#include "IEncoderCountsSubscriberEndpoint.hpp"
 
namespace differential_drive_message_endpoints
{
class EncoderCountsSubscriberEndpoint : public differential_drive_core::IEncoderCountsSubscriberEndpoint
{ 
public:
  EncoderCountsSubscriberEndpoint();
  ~EncoderCountsSubscriberEndpoint();

  void Subscribe();
  void Unsubscribe();
  bool IsSubscribed();
  void attach( differential_drive_core::IEncoderCountsListener& encoder_counts_listener );
  void detach( differential_drive_core::IEncoderCountsListener& encoder_counts_listener );

  void NewEncoderCountsReceived( const differential_drive::EncoderCounts& encoder_counts );

private:
  void notifyEncoderCountsListeners( const differential_drive::EncoderCounts& encoder_counts );

  bool _is_subscribed;

  std::vector<differential_drive_core::IEncoderCountsListener*> _encoder_counts_listeners;

  // Create handle to node
  ros::NodeHandle _encoder_counts_node;

  ros::Subscriber _encoder_counts_subscriber;
};
}
 
#endif /* GUARD_EncoderCountsSubscriberEndpoint */
