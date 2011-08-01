#include <ros/ros.h>

#include "ReadEncodersRequest.hpp"
#include "EncoderCountsReader.hpp"

extern "C"
{
  #include "RollOverHelpers.h"
}

namespace data_robot_core
{
EncoderCountsReader::EncoderCountsReader()
                    : _stop_requested( false ), 
                      _running( false ),
                      _block_for_request( true ),
                      _p_external_bus( NULL ) 
{
  _encoder_counts_listeners.reserve( 1 );
}

/** Constructor which takes an IExternalBusEndpoint to communicate on.
 */
EncoderCountsReader::EncoderCountsReader( IExternalBusEndpoint *p_external_bus )
                    : _stop_requested( false ), 
                      _running( false ),
                      _block_for_request( true ),
                      _p_external_bus( p_external_bus ) 
{
  _encoder_counts_listeners.reserve(1);
}

EncoderCountsReader::~EncoderCountsReader()
{
  StopReading();
}

/** Sets the bus to send the request to.
 *  
 *  Setting this value to NULL will disable the reader.
 */
void EncoderCountsReader::SetExternalBus( IExternalBusEndpoint *p_external_bus )
{
  _p_external_bus = p_external_bus; 
}

/** Provides a call-back mechanism for objects interested in receiving
 *  encoder count messages when they are available.
 */
void EncoderCountsReader::Attach( IEncoderCountsListener& encoder_counts_listener ) 
{
  _encoder_counts_listeners.push_back( &encoder_counts_listener );
}

/** Starts the encoder count worker thread.
 */
void EncoderCountsReader::BeginReading() 
{
  if (! _running) 
  {
    _running = true;
    _stop_requested = false;
    // Spawn async thread for reading laser scans
    pthread_create(&_thread, 0, ReadEncoderCountsFunction, this);
  }
}

/** Stops the worker thread.
 */
void EncoderCountsReader::StopReading() 
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
void EncoderCountsReader::ReadEncoderCounts() 
{
  ReadEncodersRequest bus_request;
  diff_drive::EncoderCounts encoder_counts;

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
      encoder_counts = bus_request.GetEncoderCounts();

      bus_request.Unlock();

      NotifyEncoderCountsListeners( encoder_counts );

      ROS_DEBUG(  "Encoder Counts Reader: Period: %.3f Actual: %.3f L: %d R: %d S: %d Dt: %d", 
          r.expectedCycleTime().toSec(), r.cycleTime().toSec(), encoder_counts.left_count,
          encoder_counts.right_count, encoder_counts.stasis_count, encoder_counts.dt_ms );
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
void EncoderCountsReader::NotifyEncoderCountsListeners( const diff_drive::EncoderCounts& encoder_counts ) 
{
  for (int i= 0; i < _encoder_counts_listeners.size(); i++) 
  {
      _encoder_counts_listeners[i]->OnEncoderCountsAvailableEvent( encoder_counts );
  }
}
}
