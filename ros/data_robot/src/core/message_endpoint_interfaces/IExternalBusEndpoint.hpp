// IExternalBusEndpoint.hpp
 
#ifndef GUARD_IExternalBusEndpoint
#define GUARD_IExternalBusEndpoint
 
#include "BusRequest.hpp"
 
namespace data_robot_core
{
class IExternalBusEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IExternalBusEndpoint() {}
  virtual void ProcessRequest( BusRequest *p_bus_request ) = 0;
};
}
 
#endif /* GUARD_IExternalBusEndpoint */
