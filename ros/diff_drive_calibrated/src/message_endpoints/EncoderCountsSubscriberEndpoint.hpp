// EncoderCountsSubscriberEndpoint.hpp
 
#ifndef GUARD_EncoderCountsSubscriberEndpoint
#define GUARD_EncoderCountsSubscriberEndpoint
 
#include <ros/ros.h>
#include <vector>

#include "IEncoderCountsSubscriberEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
class EncoderCountsSubscriberEndpoint : public diff_drive_core::IEncoderCountsSubscriberEndpoint
{ 
public:
  EncoderCountsSubscriberEndpoint();
  ~EncoderCountsSubscriberEndpoint();

  void subscribe();
  void unsubscribe();
  bool isSubscribed();
  void attach( diff_drive_core::IEncoderCountsListener& encoder_counts_listener );
  void detach( diff_drive_core::IEncoderCountsListener& encoder_counts_listener );

  void newEncoderCountsReceived( const diff_drive_calibrated::EncoderCounts& encoder_counts );

private:
  void notifyEncoderCountsListeners( const diff_drive_calibrated::EncoderCounts& encoder_counts );

  bool _is_subscribed;

  std::vector<diff_drive_core::IEncoderCountsListener*> _encoder_counts_listeners;

  // Create handle to node
  ros::NodeHandle _encoder_counts_node;

  ros::Subscriber _encoder_counts_subscriber;
};
}
 
#endif /* GUARD_EncoderCountsSubscriberEndpoint */
