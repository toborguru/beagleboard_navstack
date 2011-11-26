// IBumpersEndpoint.hpp
 
#ifndef GUARD_IBumpersEndpoint
#define GUARD_IBumpersEndpoint
 
#include "data_robot/Bumpers.h"
 
namespace data_robot_core
{
  class IBumpersEndpoint
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IBumpersEndpoint() {}
      virtual void Publish( const data_robot::Bumpers& bumpers ) = 0;
  };
}
 
#endif /* GUARD_IBumpersEndpoint */
