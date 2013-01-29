// BusRequestProcessorEndpointStub.hpp
 
#ifndef GUARD_BusRequestProcessorEndpointStub
#define GUARD_BusRequestProcessorEndpointStub
 
#include "BusRequest.hpp"
#include "IBusRequestProcessorEndpoint.hpp"
 
namespace data_robot_test_message_endpoints_test_doubles
{
class BusRequestProcessorEndpointStub : public data_robot_core::IBusRequestProcessorEndpoint
{ 
public:
  BusRequestProcessorEndpointStub()
    : _count_of_bus_requests_processed(0)
  { }

  int _count_of_bus_requests_processed;

  void ProcessRequest( data_robot_core::BusRequest *p_bus_request )
  {
    _count_of_bus_requests_processed++;

    if ( p_bus_request->IsLockable() )
    {
      p_bus_request->Lock();
    }

    // We have the Lock, alter the request
    for ( unsigned int i = 0; i < p_bus_request->GetDataBufferSize(); i++ )
    {
      p_bus_request->GetDataBuffer()[i] += i;
    }

    p_bus_request->SetRequestComplete( true );
    // Done altering the request

    if ( p_bus_request->IsBlocked() )
    {
      p_bus_request->Unblock();
    }

    if ( p_bus_request->IsLocked() )
    {
      p_bus_request->Unlock();
    }
  }
};
}
 
#endif /* GUARD_BusRequestProcessorEndpointStub */
