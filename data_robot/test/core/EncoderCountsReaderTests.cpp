#include <gtest/gtest.h>
#include "EncoderCountsReader.hpp"
#include "IEncoderCountsListener.hpp"
#include "diff_drive/EncoderCounts.h"
 
using namespace data_robot_core;
 
namespace data_robot_test_core
{
// Will be used by unit test to handle encoder events
struct EncoderCountsReceiver : public data_robot_core::IEncoderCountsListener
{
  EncoderCountsReceiver()
    : count_of_encoder_counts_received(0) { }

  mutable int count_of_encoder_counts_received;

  void OnEncoderCountsAvailableEvent(const diff_drive::EncoderCounts& encoder_counts)
  {
    count_of_encoder_counts_received++;

#if 0
    // Output the encoder count information, this isn't the
    // unit test, but merely a helpful means to show what's going on.
    std::cout << "Encoder Counts L: " << encoder_counts.left_count 
              << " R: " << encoder_counts.right_count
              << " S: " << encoder_counts.stasis_count
              << " DT: " << encoder_counts.dt_ms
              << std::endl;
#endif
  }
};

// Will be used by unit test to handle bus request events
struct BusRequestReceiver : public data_robot_core::IExternalBusEndpoint
{
  BusRequestReceiver()
    : count_of_bus_requests_received(0) { }

  mutable int count_of_bus_requests_received;

  void ProcessRequest( BusRequest *p_bus_request )
  {
    count_of_bus_requests_received++;

    if ( p_bus_request->IsLockable() )
    {
      p_bus_request->Lock();
    }

    // We have the Lock, alter the request
    for ( int i = 0; i < p_bus_request->GetDataBufferSize(); i++ )
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

// Define the unit test to verify ability to start and stop the reader
TEST(EncoderCountsReaderTests, canStartAndStopEncoderCountsReader) 
{
  // Establish Context
  EncoderCountsReader encoder_counts_reader;
  EncoderCountsReceiver encoder_counts_receiver;
  BusRequestReceiver bus_request_receiver;

  // Wire up the reader to the handler of laser scan reports
  encoder_counts_reader.Attach(encoder_counts_receiver);
  encoder_counts_reader.SetExternalBus(&bus_request_receiver);

  // Act
  encoder_counts_reader.BeginReading();

  // Let it perform readings for a couple of seconds
  sleep(2);
  encoder_counts_reader.StopReading();

  // Assert

  // Since we just ran the reader for 2 seconds, we should expect a few readings.
  // Arguably, this test is a bit light but makes sure encoder counts are being reported.
  EXPECT_EQ( 20, encoder_counts_receiver.count_of_encoder_counts_received );
  EXPECT_EQ( 20, bus_request_receiver.count_of_bus_requests_received );
}
}
