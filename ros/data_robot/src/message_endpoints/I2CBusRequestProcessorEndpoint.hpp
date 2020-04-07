// I2CBusI2CEndpoint.hpp
 
#ifndef GUARD_I2CBusRequestProcessorEndpoint
#define GUARD_I2CBusRequestProcessorEndpoint
 
#include <ros/ros.h>
#include <pthread.h>
#include <queue>

#include "IBusRequestProcessorEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
class I2CBusRequestProcessorEndpoint : public data_robot_core::IBusRequestProcessorEndpoint
{ 
  const int MESSAGE_ERROR_LIMIT = 100;
public:
  I2CBusRequestProcessorEndpoint();
  I2CBusRequestProcessorEndpoint( const char* dev_name );
  ~I2CBusRequestProcessorEndpoint();

  int   Open( const char* dev_name );
  int   Open();
  void  Close();

  void  SetDeviceName( const char* dev_name ) { _device_name = dev_name; };
  std::string GetDeviceName() const { return _device_name; };

  void  ProcessRequest( data_robot_core::BusRequest* p_bus_request );

  void  StartProcessingThread();
  void  StopProcessingThread();

private:
  void  ProcessBusMessages();

  int   ExecuteBusRequest( data_robot_core::BusRequest* p_bus_request );

  std::queue<data_robot_core::BusRequest*> _bus_request_queue;

  // Basic threading support as suggested by Jeremy Friesner at
  // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
  volatile bool _stop_requested;
  volatile bool _running;

  pthread_t _thread;

  int _i2c_fd;
  std::string _device_name;

  static void * ProcessBusMessagesFunction(void * This) 
  {
    ((I2CBusRequestProcessorEndpoint*)This)->ProcessBusMessages();
    return 0;
  }
};
}
 
#endif /* GUARD_I2CBusRequestProcessorEndpoint */
