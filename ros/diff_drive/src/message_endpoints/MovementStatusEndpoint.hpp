// MovementStatusEndpoint.hpp
 
#ifndef GUARD_MovementStatusEndpoint
#define GUARD_MovementStatusEndpoint
 
#include <ros/ros.h>

#include "diff_drive/MovementStatus.h"

#include "IMovementStatusEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
  class MovementStatusEndpoint : public diff_drive_core::IMovementStatusEndpoint
  { 
    public:
      MovementStatusEndpoint();

      virtual ~MovementStatusEndpoint();
 
      virtual void Publish( const diff_drive::MovementStatus& status ) const;
 
    private:
      // Create handle to node
      ros::NodeHandle _status_node;
 
      ros::Publisher _status_publisher;
  };
}
 
#endif /* GUARD_MovementStatusEndpoint */
