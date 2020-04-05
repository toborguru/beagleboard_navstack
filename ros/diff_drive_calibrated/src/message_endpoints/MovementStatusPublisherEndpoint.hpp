// MovementStatusPublisherEndpoint.hpp
 
#ifndef GUARD_MovementStatusPublisherEndpoint
#define GUARD_MovementStatusPublisherEndpoint
 
#include <ros/ros.h>

#include "diff_drive_calibrated/MovementStatus.h"

#include "IMovementStatusPublisherEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
class MovementStatusPublisherEndpoint : public diff_drive_core::IMovementStatusPublisherEndpoint
{ 
public:
  MovementStatusPublisherEndpoint();

  ~MovementStatusPublisherEndpoint();

  void onMovementStatusAvailableEvent(  const diff_drive_calibrated::MovementStatus& movement_status );

  void publish( const diff_drive_calibrated::MovementStatus& status );

private:
  // Create handle to node
  ros::NodeHandle _status_node;

  ros::Publisher _status_publisher;
};
}
 
#endif /* GUARD_MovementStatusPublisherEndpoint */
