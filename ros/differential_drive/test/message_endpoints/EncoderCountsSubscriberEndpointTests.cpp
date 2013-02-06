// EncoderCountsSubscriberEndpointTests.cpp
 
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "EncoderCountsSubscriberEndpoint.hpp"
#include "OdometryReportingService.hpp"
 
using namespace differential_drive_application_services;
using namespace differential_drive_message_endpoints;
 
namespace differential_drive_test_message_endpoints
{
// Will be used by unit test to check encoder_counts
struct EncoderCountsReceiver : public differential_drive_core::IEncoderCountsListener
{
  EncoderCountsReceiver()
    : _count_of_encoder_counts_received(0),
      _left_count(0),
      _right_count(0)
  { }

  int _count_of_encoder_counts_received;
  double _left_count;
  double _right_count;

  void OnEncoderCountsAvailableEvent(const differential_drive::EncoderCounts& encoder_counts)
  {
    _count_of_encoder_counts_received++;

    _left_count = encoder_counts.left_count;
    _right_count = encoder_counts.right_count;

    // Output the read values to the terminal; this isn't the
    // unit test, but merely a helpful means to show what's going on.

#if 0
    std::cout << "EncoderCounts sent to EncoderCountsReceiver with left_count: " 
              << _left_count
              << ", right_count: "
              << _right_count
              << std::endl;
#endif
  }
};

// Define unit test to verify ability to subscribe to encoder counts messages
// using the concrete message endpoint.
TEST(EncoderCountsSubscriberEndpointTests, canSubscribeAndUnsubscribeToEncoderCountsWithEndpoint) 
{
  // Establish Context
  std::string name("encoder_counts_endpoint_tester");
  int argc = 0;
  ros::init( argc, (char**)NULL, name );

  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<differential_drive::EncoderCounts>("encoder_counts", 12);
  usleep( 25000 );

  double left_count1;
  double left_count2;
  double left_count3;
  
  double right_count1;
  double right_count2;
  double right_count3;

  int count1;
  int count2;
  int count3;
  
  EncoderCountsSubscriberEndpoint encoder_counts_endpoint;
  EncoderCountsReceiver encoder_counts_receiver;

  differential_drive::EncoderCounts encoder_counts;
  
  encoder_counts_endpoint.Attach( encoder_counts_receiver );
 
  // ACT
  usleep( 25000 );

  encoder_counts.left_count = 10;
  encoder_counts.right_count = -15;

  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );

  ros::spinOnce();

  count1 = encoder_counts_receiver._count_of_encoder_counts_received; 
  left_count1 = encoder_counts_receiver._left_count; 
  right_count1 = encoder_counts_receiver._right_count; 

  encoder_counts_endpoint.Unsubscribe();

  encoder_counts.left_count = 20;
  encoder_counts.right_count = -35;

  usleep( 25000 );

  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );

  ros::spinOnce();

  count2 = encoder_counts_receiver._count_of_encoder_counts_received; 
  left_count2 = encoder_counts_receiver._left_count; 
  right_count2 = encoder_counts_receiver._right_count; 

  encoder_counts_endpoint.Subscribe();

  usleep( 25000 );

  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );

  ros::spinOnce();

  count3 = encoder_counts_receiver._count_of_encoder_counts_received; 
  left_count3 = encoder_counts_receiver._left_count; 
  right_count3 = encoder_counts_receiver._right_count; 

  // Assert
  EXPECT_EQ( 4, count1 ); 
  EXPECT_FLOAT_EQ( 10, left_count1 ); 
  EXPECT_FLOAT_EQ( -15, right_count1 ); 

  EXPECT_EQ( 4, count2 ); 
  EXPECT_FLOAT_EQ( 10, left_count2 ); 
  EXPECT_FLOAT_EQ( -15, right_count2 ); 

  EXPECT_EQ( 8, count3 ); 
  EXPECT_FLOAT_EQ( 20, left_count3 ); 
  EXPECT_FLOAT_EQ( -35, right_count3 ); 
}

#define FINISHED_CODING 0 // Will cause errors! :) I guess I'll fix it... I need to add a similar section to Twist Endpoint
#if FINISHED_CODING
// Define unit test to verify ability to attach and detach multiple listeners to the 
// the message endpoint.
TEST(EncoderCountsSubscriberEndpointTests, canAttachAndDetachMultipleEncoderCountsEndpoints) 
{
  // Establish Context
  std::string name("encoder_counts_endpoint_tester");
  int argc = 0;
  ros::init( argc, (char**)NULL, name );

  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<differential_drive::EncoderCounts>("encoder_counts", 12);
  usleep( 25000 );

  double left_count_a_1;
  double left_count_a_2;
  double left_count_a_3;
  double left_count_a_4;
  double left_count_a_5;
  
  double left_count_b_1;
  double left_count_b_2;
  double left_count_b_3;
  double left_count_b_4;
  double left_count_b_5;
  
  int count_a_1;
  int count_a_2;
  int count_a_3;
  int count_a_4;
  int count_a_5;

  int count_b_1;
  int count_b_2;
  int count_b_3;
  int count_b_4;
  int count_b_5;

  bool is_subscribed_a_1;
  bool is_subscribed_a_2;
  bool is_subscribed_a_3;
  bool is_subscribed_a_4;
  bool is_subscribed_a_5;
  
  bool is_subscribed_b_1;
  bool is_subscribed_b_2;
  bool is_subscribed_b_3;
  bool is_subscribed_b_4;
  bool is_subscribed_b_5;
  
  EncoderCountsSubscriberEndpoint encoder_counts_endpoint;
  EncoderCountsReceiver encoder_counts_receiver_a;
  EncoderCountsReceiver encoder_counts_receiver_b;

  differential_drive::EncoderCounts encoder_counts;
  
  encoder_counts_endpoint.Attach( encoder_counts_receiver );
 
  // ACT
  usleep( 25000 );

  encoder_counts.left_count = 10;
  encoder_counts.right_count = -15;

  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );

  count1 = encoder_counts_receiver._count_of_encoder_counts_received; 
  left_count1 = encoder_counts_receiver._left_count; 
  right_count1 = encoder_counts_receiver._right_count; 

  encoder_counts_endpoint.Subscribe();

  ros::spinOnce();

  usleep( 25000 );

  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );

  ros::spinOnce();

  count2 = encoder_counts_receiver._count_of_encoder_counts_received; 
  left_count2 = encoder_counts_receiver._left_count; 
  right_count2 = encoder_counts_receiver._right_count; 

  encoder_counts_endpoint.Unsubscribe();

  usleep( 25000 );

  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );
  pub.publish( encoder_counts );
  usleep( 25000 );

  ros::spinOnce();

  count3 = encoder_counts_receiver._count_of_encoder_counts_received; 
  left_count3 = encoder_counts_receiver._left_count; 
  right_count3 = encoder_counts_receiver._right_count; 

  // Assert
  // Nothing to assert other than using terminal windows to 
  // watch publication activity. Alternatively, for better testing, 
  // you could create a subscriber and subscribe to the reports 
  // You could then track how many reports were received and 
  // assert checks, accordingly.
  EXPECT_EQ( 0, count1 ); 
  EXPECT_FLOAT_EQ( 0.0, left_count1 ); 
  EXPECT_FLOAT_EQ( 0.0, right_count1 ); 

  EXPECT_EQ( 4, count2 ); 
  EXPECT_FLOAT_EQ( 10, left_count2 ); 
  EXPECT_FLOAT_EQ( -15, right_count2 ); 

  EXPECT_EQ( 4, count3 ); 
  EXPECT_FLOAT_EQ( 10, left_count3 ); 
  EXPECT_FLOAT_EQ( -15, right_count3 ); 
}
#endif
#if 0 
  // Define unit test to verify ability to leverage the reporting 
  // service using the concrete message endpoint. This is more of a 
  // package integration test than a unit test, making sure that all 
  // of the pieces are playing together nicely within the package.
  TEST(EncoderCountsSubscriberEndpointTests, canStartAndStopEncoderCountsCommandServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<OdometryPublisherEndpoint> odometryEndpoint =
      boost::shared_ptr<OdometryPublisherEndpoint>(new OdometryPublisherEndpoint());

    EncoderCountsSubscriberEndpoint* encoder_counts_Endpoint = new EncoderCountsSubscriberEndpoint();

    OdometryCommandService OdometryCommandService(odometryEndpoint, *encoder_counts_Endpoint);
 
    // Give ROS time to fully initialize and for the encoder_counts_Endpoint to advertise
    sleep(1);
 
    // Act
    OdometryCommandService.beginCommand();
    sleep(4);
    OdometryCommandService.stopCommand();
 
    // Assert
    // See assertion note above from 
    // EncoderCountsSubscriberEndpointTests.canPublishEncoderCountsWithEndpoint
  }
#endif
}
