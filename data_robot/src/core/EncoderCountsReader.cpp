#include <ros/ros.h>

#include "BusAddresses.h"
#include "EncoderCountsReader.hpp"
 
namespace data_robot_core
{
EncoderCountsReader::EncoderCountsReader()
                    : _stop_requested( false ), 
                      _running( false ),
                      _p_external_bus( NULL ) 
{
  _encoder_counts_listeners.reserve( 1 );
}

/** Constructor which takes an ExternalBusInterface to communicate on.
 */
EncoderCountsReader::EncoderCountsReader( IExternalBusInterface *p_external_bus )
                    : _stop_requested( false ), 
                      _running( false ),
                      _p_external_bus( p_external_bus ) 
{
  _encoder_counts_listeners.reserve(1);
}

void EncoderCountsReader::Attach( IEncoderCountsListener& encoder_counts_listener ) 
{
  _encoder_counts_listeners.push_back( &encoder_counts_listener );
}

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

void EncoderCountsReader::StopReading() 
{
  if (_running) 
  {
    _running = false;
    _stop_requested = true;
    // Wait to return until _thread has completed
    pthread_join(_thread, 0);
  }
}

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

  ros::Rate r( 10.0 );

  while (! _stop_requested) 
  {
    if ( _p_external_bus != NULL )
    {
      diff_drive::EncoderCounts encoder_counts;

      bus_request.Lock();
      _p_external_bus->ProcessRequest( &bus_request );

      // Hang the thread until bus_request.Unblock() is called
      bus_request.Block();
  
      millis = *(int16_t*)&p_data_buffer[0];
      left_count = *(int16_t*)&p_data_buffer[2];
      right_count = *(int16_t*)&p_data_buffer[4];
      stasis_count = *(int16_t*)&p_data_buffer[6];

      NotifyEncoderCountsListeners( encoder_counts );
    }

    r.sleep();
  }
}

void EncoderCountsReader::NotifyEncoderCountsListeners( const diff_drive::EncoderCounts& encoder_counts ) 
{
  for (int i= 0; i < _encoder_counts_listeners.size(); i++) 
  {
      _encoder_counts_listeners[i]->OnEncoderCountsAvailableEvent( encoder_counts );
  }
}
}
