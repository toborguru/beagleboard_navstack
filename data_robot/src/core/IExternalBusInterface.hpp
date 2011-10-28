// IExternalBusInterface.hpp
 
#ifndef GUARD_IExternalBusInterface
#define GUARD_IExternalBusInterface
 
#include "BusRequest.hpp"
 
namespace data_robot_core
{
  class IExternalBusInterface
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IExternalBusInterface() {}
      virtual void ProcessRequest( BusRequest *p_bus_request ) = 0;
  };
}
 
#endif /* GUARD_IExternalBusInterface */
