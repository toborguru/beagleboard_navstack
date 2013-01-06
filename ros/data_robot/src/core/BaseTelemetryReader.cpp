#include <ros/ros.h>

#include "ReadBaseTelemetryRequest.hpp"
#include "BaseTelemetryReader.hpp"

namespace data_robot_core
{
BaseTelemetryReader::BaseTelemetryReader()
                    : _stop_requested( false ), 
                      _running( false ),
                      _block_for_request( true ),
                      _p_external_bus( NULL ) 
{
  _telemetry_listeners.reserve( 1 );
}

/** Constructor which takes an IExternalBusEndpoint to communicate on.
 */
BaseTelemetryReader::BaseTelemetryReader( IExternalBusEndpoint *p_external_bus )
                    : _stop_requested( false ), 
                      _running( false ),
                      _block_for_request( true ),
                      _p_external_bus( p_external_bus ) 
{
  _telemetry_listeners.reserve(1);
}

BaseTelemetryReader::~BaseTelemetryReader()
{
  StopReading();
}

/** Sets the bus to send the request to.
 *  
 *  Setting this value to NULL will disable the reader.
 */
void BaseTelemetryReader::SetExternalBus( IExternalBusEndpoint *p_external_bus )
{
  _p_external_bus = p_external_bus; 
}

/** Provides a call-back mechanism for objects interested in receiving
 *  encoder count messages when they are available.
 */
void BaseTelemetryReader::Attach( IBaseTelemetryListener& telemetry_listener ) 
{
  _telemetry_listeners.push_back( &telemetry_listener );
}

/** Starts the encoder count worker thread.
 */
void BaseTelemetryReader::BeginReading() 
{
  if (! _running) 
  {
    _running = true;
    _stop_requested = false;
    // Spawn async thread for reading laser scans
    pthread_create(&_thread, 0, ReadBaseTelemetryFunction, this);
  }
}

/** Stops the worker thread.
 */
void BaseTelemetryReader::StopReading() 
{
  if ( _running ) 
  {
    _running = false;
    _stop_requested = true;
    // Wait to return until _thread has completed
    pthread_join(_thread, 0);
  }
}

/** Worker thread to regularly generate BusRequests to read the current encoder 
 *  counts from the robot.
 */
void BaseTelemetryReader::ReadBaseTelemetry() 
{
  ReadBaseTelemetryRequest bus_request;
  BaseTelemetry_T telemetry;

  ros::Rate r( 10.0 );

  while ( ! _stop_requested ) 
  {
    if ( _p_external_bus != NULL )
    {
      // Exectute our request
      bus_request.SetRequestComplete( false );
      _p_external_bus->ProcessRequest( &bus_request );

      // Check if request is being accessed
      bus_request.Lock();

      // We have the lock but the request is not finished
      while ( ! bus_request.GetRequestComplete() )
      {
        if ( _block_for_request )
        {
          // Hang the thread until bus_request.Unblock() is called
          bus_request.Block();
        }
        else
        {
          bus_request.Unlock();
          r.sleep();
          bus_request.Lock();
        }
      }

      // Now copy the data out
      telemetry = bus_request.GetTelemetry();

      bus_request.Unlock();

      NotifyBaseTelemetryListeners( telemetry );

      ROS_DEBUG(  "Base Telemetry Reader: Expected Period: %.3f Actual: %.3f L: %d R: %d S: %d Dt: %d", 
          r.expectedCycleTime().toSec(), r.cycleTime().toSec(), telemetry.left_encoder,
          telemetry.right_encoder, telemetry.stasis_encoder, telemetry.encoder_time );
    }
    else
    {
      ROS_WARN( "Encoder Counts Reader: External bus has not been assigned." );
    }

    r.sleep();
  }
}

/** Calls the callback function for all registered odometry listeners.
 */
void BaseTelemetryReader::NotifyBaseTelemetryListeners( const BaseTelemetry_T& telemetry ) 
{
  for (unsigned int i= 0; i < _telemetry_listeners.size(); i++) 
  {
    _telemetry_listeners[i]->OnBaseTelemetryAvailableEvent( telemetry );
  }
}
}
