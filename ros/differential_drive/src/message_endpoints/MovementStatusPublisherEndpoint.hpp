// MovementStatusPublisherEndpoint.hpp
 
#ifndef GUARD_MovementStatusPublisherEndpoint
#define GUARD_MovementStatusPublisherEndpoint
 
#include <ros/ros.h>

#include "differential_drive/MovementStatus.h"

#include "IMovementStatusPublisherEndpoint.hpp"
 
namespace differential_drive_message_endpoints
{
class MovementStatusPublisherEndpoint : public differential_drive_core::IMovementStatusPublisherEndpoint
{ 
public:
  MovementStatusPublisherEndpoint();

  ~MovementStatusPublisherEndpoint();

  void onMovementStatusAvailableEvent(  const differential_drive::MovementStatus& movement_status );

  void publish( const differential_drive::MovementStatus& status );

private:
  // Create handle to node
  ros::NodeHandle _status_node;

  ros::Publisher _status_publisher;
};
}
 
#endif /* GUARD_MovementStatusPublisherEndpoint */
