// IBumpersListener.hpp
 
#ifndef GUARD_IBumpersListener
#define GUARD_IBumpersListener

#include "data_robot/Bumpers.h"
 
namespace data_robot_core
{
  class IBumpersListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IBumpersListener() {}
      virtual void OnBumpersAvailableEvent( const data_robot::Bumpers& bumpers_state ) = 0;
  };
}
 
#endif /* GUARD_IBumpersListener */
