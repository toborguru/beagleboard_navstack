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
    // Output the laser scan seq number to the terminal; this isn't the
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
struct BusRequestReceiver : public data_robot_core::IExternalBusInterface
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

// Define the unit test to verify ability to differentiate around roll-overs
TEST(EncoderCountsReaderTests, canDifferentiateEncoderCountsReader) 
{
  int32_t i32count1;
  int32_t i32count2;
  int32_t i32diff1;
  int32_t i32diff2;
  int32_t i32diff3;
  int32_t i32diff4;
  int32_t i32diff5;
 
  int16_t i16count1;
  int16_t i16count2;
  int16_t i16diff1;
  int16_t i16diff2;
  int16_t i16diff3;
  int16_t i16diff4;
  int16_t i16diff5;
 
  int8_t i8count1;
  int8_t i8count2;
  int8_t i8diff1;
  int8_t i8diff2;
  int8_t i8diff3;
  int8_t i8diff4;
  int8_t i8diff5;
 
  // Establish Context
  EncoderCountsReader encoder_counts_reader;

  // Act
  i8count1 = 0;
  i8count2 = 45;
  i8diff1 = encoder_counts_reader.DifferentiateEncoderReading( i8count1, i8count2 );

  i8count1 = 0;
  i8count2 = -45;
  i8diff2 = encoder_counts_reader.DifferentiateEncoderReading( i8count1, i8count2 );

  i8count1 = 45;
  i8count2 = -45;
  i8diff3 = encoder_counts_reader.DifferentiateEncoderReading( i8count1, i8count2 );

  i8count1 = 120;
  i8count2 = 120 + 20;
  i8diff4 = encoder_counts_reader.DifferentiateEncoderReading( i8count1, i8count2 );

  i8count1 = -120;
  i8count2 = -120 - 20;
  i8diff5 = encoder_counts_reader.DifferentiateEncoderReading( i8count1, i8count2 );

  i16count1 = 0;
  i16count2 = 3E4;
  i16diff1 = encoder_counts_reader.DifferentiateEncoderReading( i16count1, i16count2 );

  i16count1 = 0;
  i16count2 = -3E4;
  i16diff2 = encoder_counts_reader.DifferentiateEncoderReading( i16count1, i16count2 );

  i16count1 = -1000;
  i16count2 = 1000;
  i16diff3 = encoder_counts_reader.DifferentiateEncoderReading( i16count1, i16count2 );

  i16count1 = 3E4;
  i16count2 =(int32_t) 3E4 + 5000;
  i16diff4 = encoder_counts_reader.DifferentiateEncoderReading( i16count1, i16count2 );

  i16count1 = -3E4;
  i16count2 = (int32_t) -3E4 - 5000;
  i16diff5 = encoder_counts_reader.DifferentiateEncoderReading( i16count1, i16count2 );

  i32count1 = 0;
  i32count2 = 2E9;
  i32diff1 = encoder_counts_reader.DifferentiateEncoderReading( i32count1, i32count2 );

  i32count1 = 0;
  i32count2 = -2E9;
  i32diff2 = encoder_counts_reader.DifferentiateEncoderReading( i32count1, i32count2 );

  i32count1 = -2E8;
  i32count2 = 2E8;
  i32diff3 = encoder_counts_reader.DifferentiateEncoderReading( i32count1, i32count2 );

  i32count1 = 2E9;
  i32count2 = (long) 2E9 + (long)3E8;
  i32diff4 = encoder_counts_reader.DifferentiateEncoderReading( i32count1, i32count2 );

  i32count1 = -2E9;
  i32count2 = (long) -2E9 - (long)3E8;
  i32diff5 = encoder_counts_reader.DifferentiateEncoderReading( i32count1, i32count2 );

  // Assert

  EXPECT_EQ( 45, (int)i8diff1 );
  EXPECT_EQ( -45, (int)i8diff2 );
  EXPECT_EQ( -90, (int)i8diff3 );
  EXPECT_EQ( 20, (int)i8diff4 );
  EXPECT_EQ( -20, (int)i8diff5 );

  EXPECT_EQ( 3E4, i16diff1 );
  EXPECT_EQ( -3E4, i16diff2 );
  EXPECT_EQ( 2000, i16diff3 );
  EXPECT_EQ( 5000, i16diff4 );
  EXPECT_EQ( -5000, i16diff5 );

  EXPECT_EQ( 2E9, i32diff1 );
  EXPECT_EQ( -2E9, i32diff2 );
  EXPECT_EQ( 4E8, i32diff3 );
  EXPECT_EQ( 3E8, i32diff4 );
  EXPECT_EQ( -3E8, i32diff5 );
}

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
