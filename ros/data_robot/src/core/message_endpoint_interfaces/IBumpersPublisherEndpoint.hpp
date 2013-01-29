// IBumpersPublisherEndpoint.hpp
 
#ifndef GUARD_IBumpersPublisherEndpoint
#define GUARD_IBumpersPublisherEndpoint
 
#include "data_robot/Bumpers.h"
 
namespace data_robot_core
{
class IBumpersPublisherEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IBumpersPublisherEndpoint() {}
  virtual void Publish( const data_robot::Bumpers& bumpers ) = 0;
};
}
 
#endif /* GUARD_IBumpersPublisherEndpoint */
