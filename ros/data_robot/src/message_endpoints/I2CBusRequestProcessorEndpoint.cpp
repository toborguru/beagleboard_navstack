/** @file
 *  ROS Node which subscribes to the @e "cmd_vel" topic, and passes any
 *  received messages along to all attached listeners.
 */
 
#include <fcntl.h>
#include <ros/ros.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "I2CBusRequestProcessorEndpoint.hpp"

#include "i2c_api.h"

using namespace data_robot_core;
 
namespace data_robot_message_endpoints
{
/** Default Constructor
 */
I2CBusRequestProcessorEndpoint::I2CBusRequestProcessorEndpoint()    
              : _stop_requested(false), 
                _running(false),
                _i2c_fd(-1)
{
}

I2CBusRequestProcessorEndpoint::I2CBusRequestProcessorEndpoint( const char* dev_name )    
              : _stop_requested(false), 
                _running(false),
                _i2c_fd(-1),
                _device_name( dev_name )
{
}

/** Destructor
 */
I2CBusRequestProcessorEndpoint::~I2CBusRequestProcessorEndpoint()
{
  Close();
}

/** Opens the previously set device.
 */
int I2CBusRequestProcessorEndpoint::Open()
{
  return Open( _device_name.c_str() );
}

/** Opens the previously set device.
 */
int I2CBusRequestProcessorEndpoint::Open( const char* dev_name )
{
  int ret_val;

  _device_name = dev_name;

  _i2c_fd = open(_device_name.c_str(), O_RDWR);  

  if ( _i2c_fd < 0 )
  {
    ret_val = -1;
  }
  else
  {
    ret_val = 0;
  }

  if (! _running)
  {
    StartProcessingThread();
  }

  return ret_val;
}

/** Closes the previously opened device.
 */
void I2CBusRequestProcessorEndpoint::Close()
{
  if ( _running )
  {
    StopProcessingThread();
  }

  if ( _i2c_fd >= 0 )
  {
    close(_i2c_fd);
    _i2c_fd = -1;
  }
}

/** Spawns the worker thread to wait for and process bus requests.
 */
void I2CBusRequestProcessorEndpoint::StartProcessingThread()
{
  if (! _running) 
  {
    _running = true;
    _stop_requested = false;
    // Spawn async thread for reading laser scans
    pthread_create(&_thread, 0, ProcessBusMessagesFunction, this);
  }
}

/** Requests thread stop, and joins worker thread.
 */
void I2CBusRequestProcessorEndpoint::StopProcessingThread()
{
  if (_running) 
  {
    _running = false;
    _stop_requested = true;

    // Wait to return until _thread has completed
    pthread_join(_thread, 0);
  }
}

/** Provides a way for other objects to pass requests for data transfer messages 
 *  when the bus is free.
 */
void I2CBusRequestProcessorEndpoint::ProcessRequest( BusRequest *p_bus_request )
{
  _bus_request_queue.push( p_bus_request );
}

/** Worker thread. 
 */
void I2CBusRequestProcessorEndpoint::ProcessBusMessages()
{
  int error;
  int num_consec_errors = 0;

  ros::Rate r(1000); // 1000 hz

  while ( !_stop_requested && (_i2c_fd > 0) )
  {
    while ( ! _bus_request_queue.empty() )
    {
      // Complete the request
      error = ExecuteBusRequest( _bus_request_queue.front() );

      if ( error )
      {
        num_consec_errors++;
      }
      else
      {
        num_consec_errors = 0;
      }

      if ( num_consec_errors > MESSAGE_ERROR_LIMIT )
      {
        // Send error message
        printf("Consecutive error limit reached. Exiting.\n");
        exit( 1 );
      }
      
      // Remove the completed request
      _bus_request_queue.pop();
    }

    r.sleep();
  }

  _running = false;
}

/** Performs the data transaction described in @a p_bus_request.
 */
int I2CBusRequestProcessorEndpoint::ExecuteBusRequest( BusRequest *p_bus_request )
{
  int request_type;
  uint8_t board_addr;
  uint8_t register_addr;

  int ret_val = -1;

  if ( p_bus_request != NULL )
  {
    if ( p_bus_request->IsLockable() )
    {
      p_bus_request->Lock();
    }

    // Sanity check
    if ( 2 == p_bus_request->GetAddressBufferSize() ) 
    {
      request_type = p_bus_request->GetRequestType();
      register_addr = p_bus_request->GetAddressBuffer()[0];
      board_addr = p_bus_request->GetAddressBuffer()[1];

      if ( REQUEST_READ == request_type )
      {
        ret_val = i2c_read_bytes( _i2c_fd, board_addr, register_addr,
              p_bus_request->GetDataBuffer(), 
              p_bus_request->GetDataBufferSize() );
      }
      else if ( REQUEST_WRITE == request_type )
      {
        ret_val = i2c_write_bytes( _i2c_fd, board_addr, register_addr,
              p_bus_request->GetDataBuffer(), 
              p_bus_request->GetDataBufferSize() );
      }

      p_bus_request->SetRequestComplete( true );
    }
  
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

  return ret_val;
}
}
