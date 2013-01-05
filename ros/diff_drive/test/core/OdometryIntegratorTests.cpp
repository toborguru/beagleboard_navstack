/* Unit test for OdometryIntegrator class.
 *
 * @author Sawyer Larkin (SJL toborguru)
 */

#include <gtest/gtest.h>

#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"

#include "OdometryIntegrator.hpp"
#include "IOdometryListener.hpp"
#include "IMovementStatusListener.hpp"
#include "IEncoderCountsEndpoint.hpp"
#include "BaseModel.hpp"

using namespace diff_drive_core;
 
namespace diff_drive_core_test_core
{
// Will be used by unit test to check odometry
struct OdometryReceiver : public diff_drive_core::IOdometryListener
{
  OdometryReceiver()
    : _count_of_messages_received(0),
      _x(0.0),
      _y(0.0),
      _theta(0.0),
      _linear(0.0),
      _angular(0.0),
      _covariance(0.0)
  { }

  int _count_of_messages_received;
  double _x;
  double _y;
  double _theta;
  double _linear;
  double _angular;
  double _covariance;

  void OnOdometryAvailableEvent(const nav_msgs::Odometry& odometry)
  {
    _count_of_messages_received++;

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

#if 0
    std::cout << std::endl << std::endl << "Covariance:";
    for (int i = 0; i < 36; i++)
    {
      if ( !(i % 6) )
      {
        std::cout << std::endl;
      }

      std::cout << odometry.pose.covariance[i] << "\t";
    }
#endif
  }
};

// Will be used by unit test to check 
struct MovementStatusReceiver : public diff_drive_core::IMovementStatusListener
{
  MovementStatusReceiver()
    : _count_of_messages_received(0),
      _linear(0.0),
      _stasis(0.0),
      _state(0),
      _stasis_enabled(false)
  { }

  int _count_of_messages_received;
  double _linear;
  double _linear_average;
  double _stasis;
  double _stasis_average;
  int _state;
  bool _stasis_enabled;

  void OnMovementStatusAvailableEvent( const diff_drive::MovementStatus& status )
  {
    _count_of_messages_received++;

    _linear = status.linear_velocity;
    _linear_average = status.linear_velocity_average;
    _stasis = status.stasis_velocity;
    _stasis_average = status.stasis_velocity_average;
    _state  = status.motors_state;
    _stasis_enabled = status.stasis_wheel_enabled;

#if 0
    // Output the read values to the terminal; this isn't the
    // unit test, but merely a helpful means to show what's going on.
    std::cout << "MovementStatus sent to MovementStatusReceiver with state: " 
              << _state
              << ", linear velocity: "
              << _linear
              << ", average: "
              << _linear_average
              << ", stasis velocity: "
              << _stasis
              << ", average: "
              << _stasis_average
              << ", stasis enabled: "
              << _stasis_enabled
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

  bool IsSubscribed()
  {
    return _subscribed;
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
  EXPECT_EQ(odometry_receiver._count_of_messages_received, 4);
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
TEST( OdometryIntegratorTests, canReadAndChangeCovariance ) 
{
  // Establish Context
  double cov1;
  double cov2;
  double cov3;

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

  base_model.SetStasisRadius( 0.5 / M_PI );
  base_model.SetStasisTicks( 100 );

  count_generator.AddTicks(new_counts);
  cov3 = odometry_receiver._covariance;

  // Assert

  EXPECT_FLOAT_EQ(cov1, 1e-1);
  EXPECT_FLOAT_EQ(cov2, 1e-9);
  EXPECT_FLOAT_EQ(cov3, 1e-1);
}

// Define the unit test to verify ability to receive MovementStatus messages when counts are added
TEST( OdometryIntegratorTests, canSendCountsAndReceiveMovementStatus ) 
{
  // Establish Context
  OdometryIntegrator odometry_integrator;
  EncoderCountsGenerator count_generator;
  MovementStatusReceiver movement_status_receiver;
  diff_drive::EncoderCounts new_counts;

  odometry_integrator.Attach(movement_status_receiver);
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

  // Check the results of adding four counts
  EXPECT_TRUE(movement_status_receiver._count_of_messages_received == 4);
}

// Define the unit test to verify ability to set and adjust the covariance
TEST( OdometryIntegratorTests, canConfigureStasisWheel ) 
{
  // Establish Context
  bool stasis_enabled_1;
  bool stasis_enabled_2;
  bool stasis_enabled_3;

  int state_1;
  int state_2;
  int state_3;

  OdometryIntegrator odometry_integrator;
  EncoderCountsGenerator count_generator;
  MovementStatusReceiver movement_status_receiver;
  diff_drive::EncoderCounts new_counts;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );

  odometry_integrator.Attach(movement_status_receiver);

  count_generator.Attach(odometry_integrator);

  new_counts.left_count = 50;
  new_counts.right_count = 50;
  new_counts.stasis_count = 50;
  new_counts.dt_ms = 100;

  // Act
  
  count_generator.AddTicks(new_counts);
  state_1 = movement_status_receiver._state;  
  stasis_enabled_1 = movement_status_receiver._stasis_enabled;  

  odometry_integrator.SetBaseModel(base_model);
  count_generator.AddTicks(new_counts);
  state_2 = movement_status_receiver._state;  
  stasis_enabled_2 = movement_status_receiver._stasis_enabled;  

  base_model.SetStasisRadius( 0.5 / M_PI );
  base_model.SetStasisTicks( 100 );
  count_generator.AddTicks(new_counts);
  state_3 = movement_status_receiver._state;  
  stasis_enabled_3 = movement_status_receiver._stasis_enabled;  

  // Assert

  EXPECT_EQ( state_1, diff_drive::MovementStatus::SETUP_ERROR );
  EXPECT_EQ( state_2, diff_drive::MovementStatus::CORRECT );
  EXPECT_EQ( state_3, diff_drive::MovementStatus::CORRECT );

  EXPECT_EQ( stasis_enabled_1, false );
  EXPECT_EQ( stasis_enabled_2, false );
  EXPECT_EQ( stasis_enabled_3, true );
}

// Define the unit test to verify ability to integrate and estimate stasis wheel velocity
TEST( OdometryIntegratorTests, canCalculateStasisVelocity ) 
{
  // Establish Context
  double linear_1;
  double linear_2;
  double linear_3;
  double linear_4;
  
  double stasis_1;
  double stasis_2;
  double stasis_3;
  double stasis_4;

  int state_1;
  int state_2;
  int state_3;
  int state_4;

  OdometryIntegrator odometry_integrator;
  EncoderCountsGenerator count_generator;
  MovementStatusReceiver movement_status_receiver;
  diff_drive::EncoderCounts new_counts;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5, 1.0, 0.5 / M_PI, 100);

  odometry_integrator.Attach(movement_status_receiver);

  count_generator.Attach(odometry_integrator);

  odometry_integrator.SetBaseModel(base_model);

  new_counts.left_count = 50;
  new_counts.right_count = 50;
  new_counts.stasis_count = 50;
  new_counts.dt_ms = 100;

  // Act
 
  // All Wheels same speed, size and count
  count_generator.AddTicks(new_counts);
  state_1 = movement_status_receiver._state;  
  linear_1 = movement_status_receiver._linear;  
  stasis_1 = movement_status_receiver._stasis;  

  // No movement of the main drive wheels
  new_counts.left_count = 0;
  new_counts.right_count = 0;
  base_model.SetStasisTicks( 50 );
  count_generator.AddTicks(new_counts);
  state_2 = movement_status_receiver._state;  
  linear_2 = movement_status_receiver._linear;  
  stasis_2 = movement_status_receiver._stasis;  

  // Stasis wheel spinning slow
  new_counts.left_count = 50;
  new_counts.right_count = 50;
  base_model.SetStasisTicks( 200 );
  count_generator.AddTicks(new_counts);
  state_3 = movement_status_receiver._state;  
  linear_3 = movement_status_receiver._linear;  
  stasis_3 = movement_status_receiver._stasis;  

  // Stasis wheel not spinning
  new_counts.stasis_count = 0;
  count_generator.AddTicks(new_counts);
  state_4 = movement_status_receiver._state;  
  linear_4 = movement_status_receiver._linear;  
  stasis_4 = movement_status_receiver._stasis;  

  // Assert

  EXPECT_EQ( diff_drive::MovementStatus::CORRECT, state_1 );
  EXPECT_EQ( diff_drive::MovementStatus::FREE_WHEELING, state_2 );
  EXPECT_EQ( diff_drive::MovementStatus::CORRECT, state_3 );
  EXPECT_EQ( diff_drive::MovementStatus::STASIS, state_4 );

  EXPECT_FLOAT_EQ( 5.0, linear_1 );
  EXPECT_FLOAT_EQ( 0.0, linear_2 );
  EXPECT_FLOAT_EQ( 5.0, linear_3 );
  EXPECT_FLOAT_EQ( 5.0, linear_4 );

  EXPECT_FLOAT_EQ( 5.0, stasis_1 );
  EXPECT_FLOAT_EQ( 10.0, stasis_2 );
  EXPECT_FLOAT_EQ( 2.5, stasis_3 );
  EXPECT_FLOAT_EQ( 0.0, stasis_4 );
}

}
