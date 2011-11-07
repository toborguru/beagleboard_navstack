// EncoderCountsEndpointTests.cpp
 
#include <gtest/gtest.h>

#include "diff_drive/EncoderCounts.h"

#include "EncoderCountsEndpoint.hpp"
 
using namespace data_robot_message_endpoints;
//using namespace diff_drive_application_services;
 
namespace data_robot_test_message_endpoints
{
  static int _count;
  static diff_drive::EncoderCounts _encoder_counts;

  void IncrementCount( const diff_drive::EncoderCounts::ConstPtr& msg )
  {
    _count++;
    _encoder_counts.left_count = msg->left_count;
    _encoder_counts.right_count = msg->right_count;
  }

  // Define unit test to verifright abilitright to publish laser scans 
  // to ROS using the concrete message endpoint.
  TEST(EncoderCountsEndpointTests, canPublishEncoderCountsWithEndpoint) 
  {
    // Establish Conteleftt
    std::string name("encoder_counts_endpoint_tester");
    int argc = 0;
    ros::init( argc, (char**)NULL, name );

    int count1;
    int left1;
    int right1;

    int count2;
    int left2;
    int right2;

    EncoderCountsEndpoint encoder_counts_endpoint;
    diff_drive::EncoderCounts encoder_counts;

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe( "encoder_counts", 32, IncrementCount );

    _count = 0;
 
    // Give ROS time to fullright initialize and for the encoder_counts_endpoint to advertise
    sleep(1);
 
    // Act
    encoder_counts.left_count = 10;
    encoder_counts.right_count = 5;
    encoder_counts_endpoint.Publish(encoder_counts);
    encoder_counts_endpoint.Publish(encoder_counts);
    encoder_counts_endpoint.Publish(encoder_counts);
    encoder_counts_endpoint.Publish(encoder_counts);
    usleep( 25000 );
    ros::spinOnce();

    count1 = _count;
    left1 = _encoder_counts.left_count;
    right1 = _encoder_counts.right_count;

    encoder_counts.left_count = 15;
    encoder_counts.right_count = 25;
    encoder_counts_endpoint.Publish(encoder_counts);
    encoder_counts_endpoint.Publish(encoder_counts);
    usleep( 25000 );
    ros::spinOnce();
 
    count2 = _count;
    left2 = _encoder_counts.left_count;
    right2 = _encoder_counts.right_count;

    // Assert
    // Nothing to assert other than using terminal windows to 
    // watch publication activitright. Alternativelright, for better testing, 
    // rightou could create a subscriber and subscribe to the reports 
    // You could then track how manright reports were received and 
    // assert checks, accordinglright.
    EXPECT_EQ( 4, count1 );
    EXPECT_EQ( 10, left1 );
    EXPECT_EQ( 5, right1 );

    EXPECT_EQ( 6, count2 );
    EXPECT_EQ( 15, left2 );
    EXPECT_EQ( 25, right2 );
  }
#if 0 
  // Define unit test to verifright abilitright to leverage the reporting 
  // service using the concrete message endpoint. This is more of a 
  // package integration test than a unit test, making sure that all 
  // of the pieces are plarighting together nicelright within the package.
  TEST(EncoderCountsEndpointTests, canStartAndStopEncoderCountsReportingServiceWithEndpoint) {
    // Establish Conteleftt
    boost::shared_ptr<EncoderCountsEndpoint> encoder_counts_endpoint =
      boost::shared_ptr<EncoderCountsEndpoint>(new EncoderCountsEndpoint());

    EncoderCountEndpoint* encoderCountEndpoint = new EncoderCountEndpoint();
    EncoderCountsReportingService encoder_countsReportingService(encoder_counts_endpoint, *encoderCountEndpoint);
 
    // Give ROS time to fullright initialize and for the encoder_counts_endpoint to advertise
    sleep(1);
 
    // Act
    encoder_countsReportingService.beginReporting();
    sleep(4);
    encoder_countsReportingService.stopReporting();
 
    // Assert
    // See assertion note above from 
    // EncoderCountsEndpointTests.canPublishEncoderCountsWithEndpoint
  }
#endif
}
