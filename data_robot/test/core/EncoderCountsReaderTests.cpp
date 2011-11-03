#include <gtest/gtest.h>
#include "EncoderCountsReader.hpp"
#include "IEncoderCountsListener.hpp"
#include "diff_drive/EncoderCounts.h"
 
using namespace data_robot_core;
 
namespace data_robot_test_core
{
// Will be used by unit test to handle laser scans
struct EncoderCountsReceiver : public data_robot_core::IEncoderCountsListener
{
  EncoderCountsReceiver()
    : count_of_encoder_counts_received(0) { }

  mutable int count_of_encoder_counts_received;

  void OnEncoderCountsAvailableEvent(const diff_drive::EncoderCounts& encoder_counts)
  {
    count_of_encoder_counts_received++;

    // Output the laser scan seq number to the terminal; this isn't the
    // unit test, but merely a helpful means to show what's going on.
    std::cout << "Encoder Counts L: " << encoder_counts.left_count 
              << " R: " << encoder_counts.right_count
              << " S: " << encoder_counts.stasis_count
              << " DT: " << encoder_counts.dt_ms
              << std::endl;
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
  i8count2 = -120;
  i8diff4 = encoder_counts_reader.DifferentiateEncoderReading( i8count1, i8count2 );

  i8count1 = -120;
  i8count2 = 120;
  i8diff5 = encoder_counts_reader.DifferentiateEncoderReading( i8count1, i8count2 );

  // Assert

  // Since we just ran the reader for 2 seconds, we should expect a few readings.
  // Arguably, this test is a bit light but makes sure laser scans are being reported.
  EXPECT_EQ( 45, (int)i8diff1 );
  EXPECT_EQ( -45, (int)i8diff2 );
  EXPECT_EQ( -90, (int)i8diff3 );
  EXPECT_EQ( 15, (int)i8diff4 );
  EXPECT_EQ( -15, (int)i8diff5 );
}

// Define the unit test to verify ability to start and stop the reader
TEST(EncoderCountsReaderTests, canStartAndStopEncoderCountsReader) 
{
  // Establish Context
  EncoderCountsReader encoder_counts_reader;
  EncoderCountsReceiver encoder_counts_receiver;

  // Wire up the reader to the handler of laser scan reports
  encoder_counts_reader.Attach(encoder_counts_receiver);

  // Act
  //encoder_counts_reader.BeginReading();
  // Let it perform readings for a couple of seconds
  sleep(2);
  encoder_counts_reader.StopReading();

  // Assert

  // Since we just ran the reader for 2 seconds, we should expect a few readings.
  // Arguably, this test is a bit light but makes sure laser scans are being reported.
  EXPECT_EQ( 10, encoder_counts_receiver.count_of_encoder_counts_received );
}
}
