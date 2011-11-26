/* Unit test for OdometryIntegrator class.
 *
 * @author Sawyer Larkin (SJL toborguru)
 */

#include <gtest/gtest.h>

#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"

#include "OdometryIntegrator.hpp"
#include "IOdometryListener.hpp"
#include "IEncoderCountsEndpoint.hpp"
#include "BaseModel.hpp"

using namespace diff_drive_core;
 
namespace diff_drive_core_test_core
{
// Will be used by unit test to check odometry
struct OdometryReceiver : public diff_drive_core::IOdometryListener
{
  OdometryReceiver()
    : _count_of_odometrys_received(0),
      _x(0.0),
      _y(0.0),
      _theta(0.0),
      _linear(0.0),
      _angular(0.0),
      _covariance(0.0)
  { }

  int _count_of_odometrys_received;
  double _x;
  double _y;
  double _theta;
  double _linear;
  double _angular;
  double _covariance;

  void OnOdometryAvailableEvent(const nav_msgs::Odometry& odometry)
  {
    _count_of_odometrys_received++;

    _x = odometry.pose.pose.position.x;
    _y = odometry.pose.pose.position.y;
    _theta = tf::getYaw(odometry.pose.pose.orientation);
    _covariance = odometry.pose.covariance[7];
    _linear = odometry.twist.twist.linear.x;
    _angular = odometry.twist.twist.angular.z;

#if 0
    // Output the read values to the terminal; this isn't the
    // unit test, but merely a helpful means to show what's going on.
    std::cout << "Odometry sent to OdometryReceiver with x: " 
              << _x
              << ", y: "
              << _y
              << ", theta: "
              << _theta
              << std::endl;
#endif
  }
};

// Will be used by the unit test to produce encoder ticks
struct EncoderCountsGenerator : public diff_drive_core::IEncoderCountsEndpoint
{
  EncoderCountsGenerator() 
    : _subscribed(false)
  {}

  void AddTicks( const diff_drive::EncoderCounts encoder_counts )
  {
    for (unsigned int i= 0; i < _encoder_counts_listeners.size(); i++) 
    {
      _encoder_counts_listeners[i]->OnEncoderCountsAvailableEvent(encoder_counts);
    }
  }

  void Subscribe()
  {
    _subscribed = true;
  }
  
  void Unsubscribe()
  {
    _subscribed = false;
  }
  
  void Attach( IEncoderCountsListener& encoder_counts_listener )
  {
    _encoder_counts_listeners.push_back(&encoder_counts_listener);
  }

  bool _subscribed;
  std::vector<IEncoderCountsListener*> _encoder_counts_listeners;
};

// Define the unit test to verify ability to listen for Ticks and generate Odometry
TEST( OdometryIntegratorTests, canSendCountsAndReceiveOdometry ) 
{
  // Establish Context
  OdometryIntegrator odometry_integrator;
  EncoderCountsGenerator count_generator;
  OdometryReceiver odometry_receiver;
  diff_drive::EncoderCounts new_counts;

  odometry_integrator.Attach(odometry_receiver);
  count_generator.Attach(odometry_integrator);

  // Act
  new_counts.left_count = 100;
  new_counts.right_count = 100;
  new_counts.dt_ms = 100;
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);

  // Assert

  // tCheck the results of adding four counts
  EXPECT_TRUE(odometry_receiver._count_of_odometrys_received == 4);
}

// Define the unit test to verify ability to integrate and estimate position
TEST( OdometryIntegratorTests, canCalculateEstimatedPosition) 
{
  // Establish Context
  double x;
  double y;
  double theta;
  double linear;
  double angular;

  OdometryIntegrator odometry_integrator;
  EncoderCountsGenerator count_generator;
  OdometryReceiver odometry_receiver;
  diff_drive::EncoderCounts new_counts;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );

  odometry_integrator.Attach(odometry_receiver);
  odometry_integrator.SetBaseModel(base_model);

  count_generator.Attach(odometry_integrator);

  // Act
  new_counts.left_count = 100;
  new_counts.right_count = 100;
  new_counts.dt_ms = 100;
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);

  x = odometry_receiver._x;
  linear = odometry_receiver._linear;

  new_counts.left_count = 0;
  new_counts.right_count = 50;
  new_counts.dt_ms = 100;
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);

  y = odometry_receiver._y;
  theta = odometry_receiver._theta;
  angular = odometry_receiver._angular;

  // Normilization check on -pi/pi (roll over on theta = 4.0)
  count_generator.AddTicks(new_counts);

  // Assert

  EXPECT_FLOAT_EQ( x, 4.0 );
  EXPECT_GT( y, 0.5 );
  EXPECT_FLOAT_EQ( theta, 3.0 );
  EXPECT_FLOAT_EQ( linear, 10.0 );
  EXPECT_FLOAT_EQ( angular, 10.0 );

  // Check roll over
  EXPECT_LT( odometry_receiver._theta, -2.25 );
}

// Define the unit test to verify ability to set and adjust the covariance
TEST( OdometryIntegratorTests, canSetAndReadCovariance) 
{
  // Establish Context
  double cov1;
  double cov2;

  OdometryIntegrator odometry_integrator;
  EncoderCountsGenerator count_generator;
  OdometryReceiver odometry_receiver;
  diff_drive::EncoderCounts new_counts;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );

  odometry_integrator.Attach(odometry_receiver);

  count_generator.Attach(odometry_integrator);

  // Act
  
  new_counts.left_count = 0;
  new_counts.right_count = 50;
  new_counts.dt_ms = 100;
  count_generator.AddTicks(new_counts);
  cov1 = odometry_receiver._covariance;

  odometry_integrator.SetBaseModel(base_model);

  count_generator.AddTicks(new_counts);
  cov2 = odometry_receiver._covariance;

  // Assert

  EXPECT_FLOAT_EQ(cov1, 1e-1);
  EXPECT_FLOAT_EQ(cov2, 1e-9);
}
}
