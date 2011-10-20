// TwistEndpointTests.cpp
 
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "TwistEndpoint.hpp"
#include "TwistCommandService.hpp"
 
using namespace diff_drive_application_services;
using namespace diff_drive_message_endpoints;
 
namespace diff_drive_test_message_endpoints
{
// Will be used by unit test to check twist
struct TwistReceiver : public diff_drive_core::ITwistListener
{
  TwistReceiver()
    : _count_of_twists_received(0),
      _linear(0),
      _angular(0)
  { }

  int _count_of_twists_received;
  double _linear;
  double _angular;

  void OnTwistAvailableEvent(const geometry_msgs::Twist& twist)
  {
    _count_of_twists_received++;

    _linear = twist.linear.x;
    _angular = twist.angular.z;

    // Output the read values to the terminal; this isn't the
    // unit test, but merely a helpful means to show what's going on.

#if 0
    std::cout << "Twist sent to TwistReceiver with linear: " 
              << _linear
              << ", angular: "
              << _angular
              << std::endl;
#endif
  }
};

  // Define unit test to verify ability to publish laser scans 
  // to ROS using the concrete message endpoint.
  TEST(TwistEndpointTests, canSubscribeAndUnsubscribeToTwistWithEndpoint) 
  {
    // Establish Context
    std::string name("twist_endpoint_tester");
    int argc = 0;
    ros::init( argc, (char**)NULL, name );

    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 12);
    sleep( 1 );

    double linear1;
    double linear2;
    double linear3;
    
    double angular1;
    double angular2;
    double angular3;

    int count1;
    int count2;
    int count3;
    
    TwistEndpoint twist_endpoint;
    TwistReceiver twist_receiver;

    geometry_msgs::Twist twist;
    
    twist_endpoint.Attach( twist_receiver );
   
    // ACT
    sleep( 1 );

    twist.linear.x = 2.5;
    twist.angular.z = -0.5;

    pub.publish( twist );
    sleep( 1 );
    pub.publish( twist );
    sleep( 1 );
    pub.publish( twist );
    sleep( 1 );
    pub.publish( twist );
    sleep( 1 );

    count1 = twist_receiver._count_of_twists_received; 
    linear1 = twist_receiver._linear; 
    angular1 = twist_receiver._angular; 

    twist_endpoint.Subscribe();

    sleep(1);

    pub.publish( twist );
    sleep( 1 );
    pub.publish( twist );
    sleep( 1 );
    pub.publish( twist );
    sleep( 1 );
    pub.publish( twist );
    sleep( 1 );

    count2 = twist_receiver._count_of_twists_received; 
    linear2 = twist_receiver._linear; 
    angular2 = twist_receiver._angular; 

    twist_endpoint.Unsubscribe();

    sleep(1);

    pub.publish( twist );
    sleep(1);
    pub.publish( twist );
    sleep(1);
    pub.publish( twist );
    sleep(1);
    pub.publish( twist );
    sleep(1);

    count3 = twist_receiver._count_of_twists_received; 
    linear3 = twist_receiver._linear; 
    angular3 = twist_receiver._angular; 

    // Assert
    // Nothing to assert other than using terminal windows to 
    // watch publication activity. Alternatively, for better testing, 
    // you could create a subscriber and subscribe to the reports 
    // You could then track how many reports were received and 
    // assert checks, accordingly.
    EXPECT_EQ( 0, count1 ); 
    EXPECT_FLOAT_EQ( 0.0, linear1 ); 
    EXPECT_FLOAT_EQ( 0.0, angular1 ); 

    EXPECT_EQ( 4, count2 ); 
    EXPECT_FLOAT_EQ( 2.5, linear2 ); 
    EXPECT_FLOAT_EQ( -0.5, angular2 ); 

    EXPECT_EQ( 4, count3 ); 
    EXPECT_FLOAT_EQ( 2.5, linear3 ); 
    EXPECT_FLOAT_EQ( -0.5, angular3 ); 

  }
#if 0 
  // Define unit test to verify ability to leverage the reporting 
  // service using the concrete message endpoint. This is more of a 
  // package integration test than a unit test, making sure that all 
  // of the pieces are playing together nicely within the package.
  TEST(TwistEndpointTests, canStartAndStopTwistCommandServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<OdometryEndpoint> odometryEndpoint =
      boost::shared_ptr<OdometryEndpoint>(new OdometryEndpoint());

    TwistEndpoint* twist_Endpoint = new TwistEndpoint();

    OdometryCommandService OdometryCommandService(odometryEndpoint, *twist_Endpoint);
 
    // Give ROS time to fully initialize and for the twist_Endpoint to advertise
    sleep(1);
 
    // Act
    OdometryCommandService.beginCommand();
    sleep(4);
    OdometryCommandService.stopCommand();
 
    // Assert
    // See assertion note above from 
    // TwistEndpointTests.canPublishTwistWithEndpoint
  }
#endif
}
