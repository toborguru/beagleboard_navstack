/* @file
 *
 * Cyclic worker class that reads telemetry from the Front Shell of D.A.T.A.,
 * processes it and publishes the results.
 */
#include <ros/ros.h>

#include "FrontTelemetryReader.hpp"
#include "ReadFrontTelemetryRequest.hpp"

namespace data_robot_core
{
FrontTelemetryReader::FrontTelemetryReader()
                    : _stop_requested( false ), 
                      _running( false ),
                      _block_for_request( true ),
                      _p_external_bus( NULL ) 
{
  _telemetry_listeners.reserve( 1 );
}

/** Constructor which takes an IBusRequestProcessorEndpoint to communicate on.
 */
FrontTelemetryReader::FrontTelemetryReader( IBusRequestProcessorEndpoint *p_external_bus )
                    : _stop_requested( false ), 
                      _running( false ),
                      _block_for_request( true ),
                      _p_external_bus( p_external_bus ) 
{
  _telemetry_listeners.reserve(1);
}

FrontTelemetryReader::~FrontTelemetryReader()
{
  StopReading();
}

/** Sets the bus to send the request to.
 *  
 *  Setting this value to NULL will disable the reader.
 */
void FrontTelemetryReader::SetExternalBus( IBusRequestProcessorEndpoint *p_external_bus )
{
  _p_external_bus = p_external_bus; 
}

/** Provides a call-back mechanism for objects interested in receiving
 *  bumper state messages when they are available.
 */
void FrontTelemetryReader::Attach( IFrontTelemetryListener& telemetry_listener ) 
{
  _telemetry_listeners.push_back( &telemetry_listener );
}

/** Starts the encoder count worker thread.
 */
void FrontTelemetryReader::BeginReading() 
{
  if (! _running) 
  {
    _running = true;
    _stop_requested = false;
    // Spawn async thread for reading laser scans
    pthread_create(&_thread, 0, ReadFrontTelemetryFunction, this);
  }
}

/** Stops the worker thread.
 */
void FrontTelemetryReader::StopReading() 
{
  if ( _running ) 
  {
    _running = false;
    _stop_requested = true;
    // Wait to return until _thread has completed
    pthread_join(_thread, 0);
  }
}

/** Worker thread to regularly generate BusRequests to read the current telemetry 
 *  from the robot.
 */
void FrontTelemetryReader::ReadFrontTelemetry() 
{
  ReadFrontTelemetryRequest bus_request;
  FrontShellTelemetry_T telemetry;

  ros::Rate r( 12.0 );

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

      NotifyFrontTelemetryListeners( telemetry );

      ROS_DEBUG(  "Front Telemetry Reader: Period: %.3f Actual: %.3f Bumpers: %d", 
                  r.expectedCycleTime().toSec(), r.cycleTime().toSec(), telemetry.bumpers);
    }
    else
    {
      ROS_WARN( "Encoder Counts Reader: External bus has not been assigned." );
    }

    r.sleep();
  }
}

/** Calls the callback function for all registered bumper listeners.
 */
void FrontTelemetryReader::NotifyFrontTelemetryListeners( const FrontShellTelemetry_T& telemetry ) 
{
  for (unsigned int i= 0; i < _telemetry_listeners.size(); i++) 
  {
      _telemetry_listeners[i]->OnFrontTelemetryAvailableEvent( telemetry );
  }
}
}
