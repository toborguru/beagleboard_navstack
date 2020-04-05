// TickVelocitySubscriberEndpointTests.cpp
 
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "TickVelocitySubscriberEndpoint.hpp"
//#include "TickVelocityCommandService.hpp"
 
//using namespace data_robot_application_services;
using namespace data_robot_message_endpoints;
 
namespace data_robot_test_message_endpoints
{
// Will be used by unit test to check tick_velocity
struct TickVelocityReceiver : public data_robot_core::ITickVelocityListener
{
  TickVelocityReceiver()
    : _count_of_tick_velocities_received(0),
      _linear(0),
      _angular(0)
  { }

  int _count_of_tick_velocities_received;
  int _linear;
  int _angular;

  void OnTickVelocityAvailableEvent(const diff_drive_calibrated::TickVelocity& tick_velocity)
  {
    _count_of_tick_velocities_received++;

    _linear = tick_velocity.linear_ticks_sec;
    _angular = tick_velocity.angular_ticks_sec;

    // Output the read values to the terminal; this isn't the
    // unit test, but merely a helpful means to show what's going on.

#if 0
    std::cout << "TickVelocity sent to TickVelocityReceiver with linear: " 
              << _linear
              << ", angular: "
              << _angular
              << std::endl;
#endif
  }
};

  // Define unit test to verify ability to publish laser scans 
  // to ROS using the concrete message endpoint.
  TEST(TickVelocitySubscriberEndpointTests, canSubscribeAndUnsubscribeToTickVelocityWithEndpoint) 
  {
    // Establish Context
    std::string name("tick_velocity_endpoint_tester");
    int argc = 0;
    ros::init( argc, (char**)NULL, name );

    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<diff_drive_calibrated::TickVelocity>("cmd_ticks", 12);
    usleep( 25000 );

    int linear1;
    int linear2;
    int linear3;
    
    int angular1;
    int angular2;
    int angular3;

    int count1;
    int count2;
    int count3;
    
    TickVelocitySubscriberEndpoint tick_velocity_endpoint;
    TickVelocityReceiver tick_velocity_receiver;

    diff_drive_calibrated::TickVelocity tick_velocity;
    
    tick_velocity_endpoint.Attach( tick_velocity_receiver );
   
    // ACT
    usleep( 25000 );

    tick_velocity.linear_ticks_sec = 25;
    tick_velocity.angular_ticks_sec = -5;

    pub.publish( tick_velocity );
    usleep( 25000 );
    pub.publish( tick_velocity );
    usleep( 25000 );
    pub.publish( tick_velocity );
    usleep( 25000 );
    pub.publish( tick_velocity );
    usleep( 25000 );

    count1 = tick_velocity_receiver._count_of_tick_velocities_received; 
    linear1 = tick_velocity_receiver._linear; 
    angular1 = tick_velocity_receiver._angular; 

    tick_velocity_endpoint.Subscribe();

    usleep( 25000 );

    pub.publish( tick_velocity );
    usleep( 25000 );
    pub.publish( tick_velocity );
    usleep( 25000 );
    pub.publish( tick_velocity );
    usleep( 25000 );
    pub.publish( tick_velocity );
    usleep( 25000 );

    count2 = tick_velocity_receiver._count_of_tick_velocities_received; 
    linear2 = tick_velocity_receiver._linear; 
    angular2 = tick_velocity_receiver._angular; 

    tick_velocity_endpoint.Unsubscribe();

    usleep( 25000 );

    pub.publish( tick_velocity );
    usleep( 25000 );
    pub.publish( tick_velocity );
    usleep( 25000 );
    pub.publish( tick_velocity );
    usleep( 25000 );
    pub.publish( tick_velocity );
    usleep( 25000 );

    count3 = tick_velocity_receiver._count_of_tick_velocities_received; 
    linear3 = tick_velocity_receiver._linear; 
    angular3 = tick_velocity_receiver._angular; 

    // Assert
    // Nothing to assert other than using terminal windows to 
    // watch publication activity. Alternatively, for better testing, 
    // you could create a subscriber and subscribe to the reports 
    // You could then track how many reports were received and 
    // assert checks, accordingly.
    EXPECT_EQ( 0, count1 ); 
    EXPECT_EQ( 0, linear1 ); 
    EXPECT_EQ( 0, angular1 ); 

    EXPECT_EQ( 4, count2 ); 
    EXPECT_EQ( 25, linear2 ); 
    EXPECT_EQ( -5, angular2 ); 

    EXPECT_EQ( 4, count3 ); 
    EXPECT_EQ( 25, linear3 ); 
    EXPECT_EQ( -5, angular3 ); 
  }
#if 0 
  // Define unit test to verify ability to leverage the reporting 
  // service using the concrete message endpoint. This is more of a 
  // package integration test than a unit test, making sure that all 
  // of the pieces are playing together nicely within the package.
  TEST(TickVelocitySubscriberEndpointTests, canStartAndStopTickVelocityCommandServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<OdometryEndpoint> odometryEndpoint =
      boost::shared_ptr<OdometryEndpoint>(new OdometryEndpoint());

    TickVelocitySubscriberEndpoint* tick_velocity_Endpoint = new TickVelocitySubscriberEndpoint();

    OdometryCommandService OdometryCommandService(odometryEndpoint, *tick_velocity_Endpoint);
 
    // Give ROS time to fully initialize and for the tick_velocity_Endpoint to advertise
    sleep(1);
 
    // Act
    OdometryCommandService.beginCommand();
    sleep(4);
    OdometryCommandService.stopCommand();
 
    // Assert
    // See assertion note above from 
    // TickVelocitySubscriberEndpointTests.canPublishTickVelocityWithEndpoint
  }
#endif
}
