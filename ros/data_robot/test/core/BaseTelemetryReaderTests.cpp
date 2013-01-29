#include <gtest/gtest.h>
#include <ros/ros.h>

#include "BaseTelemetryReader.hpp"
#include "IBaseTelemetryListener.hpp"
#include "Telemetry.hpp"
 
using namespace data_robot_core;
 
namespace data_robot_test_core
{
// Will be used by unit test to handle encoder events
struct BaseTelemetryReceiver : public data_robot_core::IBaseTelemetryListener
{
  BaseTelemetryReceiver()
    : count_of_telemetry_received(0) { }

  mutable int count_of_telemetry_received;

  void OnBaseTelemetryAvailableEvent(const BaseTelemetry_T& telemetry)
  {
    count_of_telemetry_received++;

#if 0
    // Output the encoder count information, this isn't the
    // unit test, but merely a helpful means to show what's going on.
    std::cout << "Encoder Counts L: " << telemetry.left_count 
              << " R: " << telemetry.right_count
              << " S: " << telemetry.stasis_count
              << " DT: " << telemetry.dt_ms
              << std::endl;
#endif
  }
};

// Will be used by unit test to handle bus request events
struct BusRequestReceiver : public data_robot_core::IBusRequestProcessorEndpoint
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
    for ( unsigned int i = 0; i < p_bus_request->GetDataBufferSize(); i++ )
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
TEST(BaseTelemetryReaderTests, canStartAndStopBaseTelemetryReader) 
{
  // Establish Context

  ros::Time::init();

  BaseTelemetryReader telemetry_reader;
  BaseTelemetryReceiver telemetry_receiver;
  BusRequestReceiver bus_request_receiver;

  // Wire up the reader to the handler of laser scan reports
  telemetry_reader.Attach(telemetry_receiver);
  telemetry_reader.SetExternalBus(&bus_request_receiver);

  // Act
  telemetry_reader.BeginReading();

  // Let it perform readings for a couple of seconds
  sleep(2);
  telemetry_reader.StopReading();

  // Assert

  // Since we just ran the reader for 2 seconds, we should expect a few readings.
  // Arguably, this test is a bit light but makes sure encoder counts are being reported.
  EXPECT_EQ( 20, telemetry_receiver.count_of_telemetry_received );
  EXPECT_EQ( 20, bus_request_receiver.count_of_bus_requests_received );
}
}
