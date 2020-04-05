// ITwistListener.hpp
 
#ifndef GUARD_ITwistListener
#define GUARD_ITwistListener

#include "geometry_msgs/Twist.h"
 
namespace diff_drive_core
{
  class ITwistListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~ITwistListener() {}
      virtual void onTwistAvailableEvent( const geometry_msgs::Twist& twist ) = 0;
  };
}
 
#endif /* GUARD_ITwistListener */
