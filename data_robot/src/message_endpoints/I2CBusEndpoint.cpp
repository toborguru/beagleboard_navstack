/** @file
 *  ROS Node which subscribes to the @e "cmd_vel" topic, and passes any
 *  received messages along to all attached listeners.
 */
 
#include <fcntl.h>
#include <ros/ros.h>

#include "I2CBusEndpoint.hpp"

#include "i2c-api.h"

using namespace data_robot_core;
 
namespace data_robot_message_endpoints
{
/** Default Constructor
 */
I2CBusEndpoint::I2CBusEndpoint()    
              : _stop_requested(false), 
                _running(false),
                _i2c_fd(-1)
{
}

I2CBusEndpoint::I2CBusEndpoint( const char* dev_name )    
              : _stop_requested(false), 
                _running(false),
                _i2c_fd(-1),
                _device_name( dev_name )
{
}

/** Destructor
 */
I2CBusEndpoint::~I2CBusEndpoint()
{
  Close();
}

/** Opens the previously set device.
 */
int I2CBusEndpoint::Open()
{
  return Open( _device_name.c_str() );
}

/** Opens the previously set device.
 */
int I2CBusEndpoint::Open( const char* dev_name )
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
void I2CBusEndpoint::Close()
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
void I2CBusEndpoint::StartProcessingThread()
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
void I2CBusEndpoint::StopProcessingThread()
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
void I2CBusEndpoint::ProcessRequest( BusRequest *p_bus_request )
{
  _bus_request_queue.push( p_bus_request );
}

/** Worker thread. Subscribes to the ROS topic and checks for ROS or class stop request.
 */
void I2CBusEndpoint::ProcessBusMessages()
{
  ros::Rate r(1000); // 100 hz

  while ( !_stop_requested && (_i2c_fd > 0) )
  {
    while ( ! _bus_request_queue.empty() )
    {
      // Complete the request
      ExecuteBusRequest( _bus_request_queue.front() );
      
      // Remove the completed request
      _bus_request_queue.pop();
    }

    r.sleep();
  }

  _running = false;
}

/** Performs the data transaction described in @a p_bus_request.
 */
void I2CBusEndpoint::ExecuteBusRequest( BusRequest *p_bus_request )
{
  int request_type;
  uint8_t addr;
  uint8_t register_addr;

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
      addr = p_bus_request->GetAddressBuffer()[0];
      register_addr = p_bus_request->GetAddressBuffer()[1];

      if ( REQUEST_READ == request_type )
      {
        I2cSetSlaveAddress( _i2c_fd, addr, 0 );

        I2cReadBytes( _i2c_fd, register_addr,
                      p_bus_request->GetDataBuffer(),
                      p_bus_request->GetDataBufferSize() );
      }
      else if ( REQUEST_WRITE == request_type )
      {
        I2cSetSlaveAddress( _i2c_fd, addr, 0 );

        I2cWriteBytes(  _i2c_fd, register_addr,
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
}
}
