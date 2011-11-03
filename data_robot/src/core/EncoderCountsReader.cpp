#include <ros/ros.h>
#include <limits.h>

#include "BusAddresses.h"
#include "EncoderCountsReader.hpp"

namespace data_robot_core
{
EncoderCountsReader::EncoderCountsReader()
                    : _stop_requested( false ), 
                      _running( false ),
                      _block_for_request( false ),
                      _p_external_bus( NULL ) 
{
  _encoder_counts_listeners.reserve( 1 );
}

/** Constructor which takes an IExternalBusInterface to communicate on.
 */
EncoderCountsReader::EncoderCountsReader( IExternalBusInterface *p_external_bus )
                    : _stop_requested( false ), 
                      _running( false ),
                      _p_external_bus( p_external_bus ) 
{
  _encoder_counts_listeners.reserve(1);
}

/** Sets the bus to send the request to.
 *  
 *  Setting this value to NULL will disable the reader.
 */
void EncoderCountsReader::SetExternalBus( IExternalBusInterface *p_external_bus )
{
  _p_external_bus = p_external_bus; 
}

/** Provides the difference in encoder counts while trying to detect and correct 
 *  for roll-over.
 */
int32_t  EncoderCountsReader::DifferentiateEncoderReading( int32_t last_reading, int32_t new_reading ) const
{
  long difference;

  difference = (long)new_reading - (long)last_reading;

  if ( difference > INT_MAX )
  {
    difference = ( (new_reading - INT_MAX) + (INT_MIN - last_reading) );
  }
  else if ( difference < INT_MIN )
  {
    difference = ( (INT_MAX - last_reading) + (new_reading - INT_MIN) );
  }

  return (int32_t)difference;
}

/** Provides the difference in encoder counts while trying to detect and correct 
 *  for roll-over.
 */
int16_t  EncoderCountsReader::DifferentiateEncoderReading( int16_t last_reading, int16_t new_reading ) const
{
  int32_t difference;

  difference = (int32_t)new_reading - (int32_t)last_reading;

  if ( difference > SHRT_MAX )
  {
    difference = ( (new_reading - SHRT_MAX) + (SHRT_MIN - last_reading) );
  }
  else if ( difference < SHRT_MIN )
  {
    difference = ( (SHRT_MAX - last_reading) + (new_reading - SHRT_MIN) );
  }

  return (int16_t)difference;
}

/** Provides the difference in encoder counts while trying to detect and correct 
 *  for roll-over.
 */
int8_t   EncoderCountsReader::DifferentiateEncoderReading( int8_t  last_reading,  int8_t new_reading ) const
{
  int16_t difference;

  difference = (int16_t)new_reading - (int16_t)last_reading;

  if ( difference > SCHAR_MAX )
  {
    difference = ( (new_reading - SCHAR_MAX) + (SCHAR_MIN - last_reading) );
  }
  else if ( difference < SCHAR_MIN )
  {
    difference = ( (SCHAR_MAX - last_reading) + (new_reading - SCHAR_MIN) );
  }

  return (int8_t)difference;
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

/** Worker thread to regularly generate BusRequests to read the current enocder 
 *  counts from the robot.
 */
void EncoderCountsReader::ReadEncoderCounts() 
{
  int16_t left_count;
  int16_t right_count;
  int16_t stasis_count;
  uint16_t millis;

  BusRequest bus_request;

  uint16_t address = ENCODER_COUNTS_ADDRESS;
  uint8_t *p_data_buffer;

  // Setup bus request to read the encoder counts
  bus_request.Lock();
  bus_request.SetAddress( (uint8_t*)&address, ADDRESS_SIZE );
  bus_request.SetRequestType( REQUEST_READ );
  bus_request.SetDataBufferSize( 8 ); // How many bytes to request
  bus_request.Unlock();

  p_data_buffer = bus_request.GetDataBuffer();

  ros::Time::init();
  ros::Rate r( 10.0 );

  while ( ! _stop_requested ) 
  {
    if ( _p_external_bus != NULL )
    {
      diff_drive::EncoderCounts encoder_counts;

      bus_request.Lock();

      // Exectute our request
      _p_external_bus->ProcessRequest( &bus_request );

      // Hang the thread until bus_request.Unblock() is called
      bus_request.Block();
  
      // Now copy the data out
      bus_request.Lock();
      millis = *(int16_t*)&p_data_buffer[0];
      left_count = *(int16_t*)&p_data_buffer[2];
      right_count = *(int16_t*)&p_data_buffer[4];
      stasis_count = *(int16_t*)&p_data_buffer[6];
      bus_request.Unlock();

      NotifyEncoderCountsListeners( encoder_counts );

      ROS_INFO( "Encoder Counts Reader: Period: %.3f Actual: %.3f", 
          r.expectedCycleTime().toSec(), r.cycleTime().toSec() );
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
