// IBusRequestProcessorEndpoint.hpp
 
#ifndef GUARD_IBusRequestProcessorEndpoint
#define GUARD_IBusRequestProcessorEndpoint
 
#include "BusRequest.hpp"
 
namespace data_robot_core
{
class IBusRequestProcessorEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IBusRequestProcessorEndpoint() {}
  virtual void ProcessRequest( BusRequest *p_bus_request ) = 0;
};
}
 
#endif /* GUARD_IBusRequestProcessorEndpoint */
