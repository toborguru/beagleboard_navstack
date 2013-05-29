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
#include "IEncoderCountsSubscriberEndpoint.hpp"
#include "BaseModel.hpp"

using namespace differential_drive_core;
 
namespace differential_drive_core_test_core
{
// Will be used by unit test to check odometry
struct OdometryReceiver : public differential_drive_core::IOdometryListener
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
    ++_count_of_messages_received;

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
    for (int i = 0; i < 36; ++i)
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
struct MovementStatusReceiver : public differential_drive_core::IMovementStatusListener
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

  void OnMovementStatusAvailableEvent( const differential_drive::MovementStatus& status )
  {
    ++_count_of_messages_received;

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
struct EncoderCountsGenerator : public differential_drive_core::IEncoderCountsSubscriberEndpoint
{
  EncoderCountsGenerator() 
    : _subscribed(false)
  {}

  void AddTicks( const differential_drive::EncoderCounts encoder_counts )
  {
    for ( unsigned int i= 0; i < _encoder_counts_listeners.size(); ++i ) 
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

  void Detach( IEncoderCountsListener& encoder_counts_listener )
  { 
    // Using the remove-erase idiom
    std::vector<IEncoderCountsListener*>& vec = _encoder_counts_listeners; // use shorter name
    vec.erase( std::remove(vec.begin(), vec.end(), &encoder_counts_listener), vec.end() );

    if ( _encoder_counts_listeners.size() == 0 )
    { 
      Unsubscribe();
    }
  }

  bool _subscribed;
  std::vector<IEncoderCountsListener*> _encoder_counts_listeners;
};

// Define the unit test to verify ability to listen for Ticks and generate Odometry
TEST( OdometryIntegratorTests, canSendCountsAndReceiveOdometry ) 
{
  // Establish Context
  BaseModel base_model;
  OdometryIntegrator odometry_integrator;
  EncoderCountsGenerator count_generator;
  OdometryReceiver odometry_receiver;
  differential_drive::EncoderCounts new_counts;

  odometry_integrator.Attach(odometry_receiver);
  odometry_integrator.setBaseModel( base_model );
  count_generator.Attach(odometry_integrator);

  // Act
  new_counts.left_count = 100;
  new_counts.right_count = 100;
  new_counts.dt_ms = 100;
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);

  usleep(2500);

  // Assert

  // tCheck the results of adding four counts
  EXPECT_EQ( 4, odometry_receiver._count_of_messages_received );
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
  differential_drive::EncoderCounts new_counts;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );

  odometry_integrator.Attach(odometry_receiver);
  odometry_integrator.setBaseModel(base_model);

  count_generator.Attach(odometry_integrator);

  usleep(2500);

  // Act
  new_counts.left_count = 100;
  new_counts.right_count = 100;
  new_counts.dt_ms = 100;
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);

  usleep(2500);

  x = odometry_receiver._x;
  linear = odometry_receiver._linear;

  new_counts.left_count = 0;
  new_counts.right_count = 50;
  new_counts.dt_ms = 100;
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);
  count_generator.AddTicks(new_counts);

  usleep(2500);

  y = odometry_receiver._y;
  theta = odometry_receiver._theta;
  angular = odometry_receiver._angular;

  // Normilization check on -pi/pi (roll over (@ Pi) on theta = 4.0)
  count_generator.AddTicks(new_counts);

  usleep(2500);

  // Assert

  EXPECT_FLOAT_EQ( 4.0, x );
  EXPECT_LT( 0.5, y );
  EXPECT_FLOAT_EQ( 3.0,  theta );
  EXPECT_FLOAT_EQ( 10.0, linear );
  EXPECT_FLOAT_EQ( 10.0, angular );

  // Check roll over
  EXPECT_GT( -2.25, odometry_receiver._theta );
}

// Define the unit test to verify ability to set and adjust the covariance
TEST( OdometryIntegratorTests, canReadAndChangeCovariance ) 
{
  // Establish Context
  double cov1;
  double cov2;

  OdometryIntegrator odometry_integrator;
  EncoderCountsGenerator count_generator;
  OdometryReceiver odometry_receiver;
  differential_drive::EncoderCounts new_counts;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );

  odometry_integrator.Attach(odometry_receiver);

  odometry_integrator.setBaseModel( base_model );

  count_generator.Attach(odometry_integrator);

  // Act
  
  new_counts.left_count = 0;
  new_counts.right_count = 50;
  new_counts.dt_ms = 100;
  count_generator.AddTicks(new_counts);

  usleep(2500);

  cov1 = odometry_receiver._covariance;

  count_generator.AddTicks(new_counts);

  usleep(2500);

  base_model.setStasisRadius( 0.5 / M_PI );
  base_model.setStasisTicks( 100 );

  count_generator.AddTicks(new_counts);

  usleep(2500);

  cov2 = odometry_receiver._covariance;

  // Assert

  EXPECT_FLOAT_EQ( 1e-9, cov1 );
  EXPECT_FLOAT_EQ( 1e-1, cov2 );
}

// Define the unit test to verify ability to receive MovementStatus messages when counts are added
TEST( OdometryIntegratorTests, canSendCountsAndReceiveMovementStatus ) 
{
  // Establish Context
  OdometryIntegrator odometry_integrator;
  EncoderCountsGenerator count_generator;
  MovementStatusReceiver movement_status_receiver;
  differential_drive::EncoderCounts new_counts;

  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );

  odometry_integrator.setBaseModel(base_model);

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

  usleep(2500);

  // Assert

  // Check the results of adding four counts
  EXPECT_EQ( 4, movement_status_receiver._count_of_messages_received );
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
  differential_drive::EncoderCounts new_counts;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );

  odometry_integrator.setBaseModel(base_model);

  odometry_integrator.Attach(movement_status_receiver);
  odometry_integrator.setAverage2nReadings(0); // Only average 1 reading

  count_generator.Attach(odometry_integrator);

  new_counts.left_count = 50;
  new_counts.right_count = 50;
  new_counts.dt_ms = 100;

  // Act
  
  count_generator.AddTicks(new_counts);

  usleep(2500);

  state_1 = movement_status_receiver._state;  
  stasis_enabled_1 = movement_status_receiver._stasis_enabled;  

  base_model.setStasisRadius( 1.0 / M_PI );
  base_model.setStasisTicks( 100 );
  count_generator.AddTicks(new_counts);

  usleep(2500);

  state_2 = movement_status_receiver._state;  
  stasis_enabled_2 = movement_status_receiver._stasis_enabled;  

  new_counts.stasis_count = 25;
  
  count_generator.AddTicks(new_counts);

  usleep(2500);

  state_3 = movement_status_receiver._state;  
  stasis_enabled_3 = movement_status_receiver._stasis_enabled;  

  // Assert

  EXPECT_EQ( differential_drive::MovementStatus::CORRECT, state_1  );
  EXPECT_EQ( differential_drive::MovementStatus::STASIS, state_2 );
  EXPECT_EQ( differential_drive::MovementStatus::CORRECT, state_3 );

  EXPECT_FALSE( stasis_enabled_1 );
  EXPECT_TRUE( stasis_enabled_2 );
  EXPECT_TRUE( stasis_enabled_3 );
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
  differential_drive::EncoderCounts new_counts;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5, 1.0, 0.5 / M_PI, 100);

  odometry_integrator.Attach(movement_status_receiver);
  odometry_integrator.setAverage2nReadings(0); // Only average 1 reading

  count_generator.Attach(odometry_integrator);

  odometry_integrator.setBaseModel(base_model);

  new_counts.left_count = 50;
  new_counts.right_count = 50;
  new_counts.stasis_count = 50;
  new_counts.dt_ms = 100;

  // Act
 
  // All Wheels same speed, size and count
  count_generator.AddTicks(new_counts);

  usleep(2500);

  state_1 = movement_status_receiver._state;  
  linear_1 = movement_status_receiver._linear;  
  stasis_1 = movement_status_receiver._stasis;  

  // No movement of the main drive wheels
  new_counts.left_count = 0;
  new_counts.right_count = 0;
  base_model.setStasisTicks( 50 );
  count_generator.AddTicks(new_counts);

  usleep(2500);

  state_2 = movement_status_receiver._state;  
  linear_2 = movement_status_receiver._linear;  
  stasis_2 = movement_status_receiver._stasis;  

  // Stasis wheel spinning slow
  new_counts.left_count = 50;
  new_counts.right_count = 50;
  base_model.setStasisTicks( 200 );
  count_generator.AddTicks(new_counts);

  usleep(2500);

  state_3 = movement_status_receiver._state;  
  linear_3 = movement_status_receiver._linear;  
  stasis_3 = movement_status_receiver._stasis;  

  // Stasis wheel not spinning
  new_counts.stasis_count = 0;
  count_generator.AddTicks(new_counts);

  usleep(2500);

  state_4 = movement_status_receiver._state;  
  linear_4 = movement_status_receiver._linear;  
  stasis_4 = movement_status_receiver._stasis;  

  // Assert

  EXPECT_EQ( differential_drive::MovementStatus::CORRECT, state_1 );
  EXPECT_EQ( differential_drive::MovementStatus::FREE_WHEELING, state_2 );
  EXPECT_EQ( differential_drive::MovementStatus::STASIS, state_3 );
  EXPECT_EQ( differential_drive::MovementStatus::STASIS, state_4 );

  EXPECT_FLOAT_EQ( 5.0, linear_1 );
  EXPECT_FLOAT_EQ( 0.0, linear_2 );
  EXPECT_FLOAT_EQ( 5.0, linear_3 );
  EXPECT_FLOAT_EQ( 5.0, linear_4 );

  EXPECT_FLOAT_EQ( 5.0, stasis_1 );
  EXPECT_FLOAT_EQ( 10.0, stasis_2 );
  EXPECT_FLOAT_EQ( 2.5, stasis_3 );
  EXPECT_FLOAT_EQ( 0.0, stasis_4 );
}

// Define the unit test to verify ability to set and change the 
// averaging buffers
TEST( OdometryIntegratorTests, canChangeAndReadAverageBufferSizes )
{
  int average_2n_1;
  int average_2n_2;
  int average_2n_3;
  int average_2n_4;
  int average_2n_5;
  int average_2n_6;
  int average_2n_7;
  int average_2n_8;
  int average_2n_9;

  int average_num_1;
  int average_num_2;
  int average_num_3;
  int average_num_4;
  int average_num_5;
  int average_num_6;
  int average_num_7;
  int average_num_8;
  int average_num_9;

  // Establish Context
  OdometryIntegrator odometry_integrator;
  
  // Act
  // Check defaults
  average_2n_1 = odometry_integrator.getAverage2nReadings();
  average_num_1 = odometry_integrator.getAverageNumReadings();
 
  odometry_integrator.setAverage2nReadings( 4 );
  average_2n_2 = odometry_integrator.getAverage2nReadings();
  average_num_2 = odometry_integrator.getAverageNumReadings();
  
  odometry_integrator.setAverage2nReadings( 0 );
  average_2n_3 = odometry_integrator.getAverage2nReadings();
  average_num_3 = odometry_integrator.getAverageNumReadings();
  
  odometry_integrator.setAverageNumReadings( 4 );
  average_2n_4 = odometry_integrator.getAverage2nReadings();
  average_num_4 = odometry_integrator.getAverageNumReadings();
  
  odometry_integrator.setAverageNumReadings( 0 );
  average_2n_5 = odometry_integrator.getAverage2nReadings();
  average_num_5 = odometry_integrator.getAverageNumReadings();
  
  odometry_integrator.setAverage2nReadings( 16 );
  average_2n_6 = odometry_integrator.getAverage2nReadings();
  average_num_6 = odometry_integrator.getAverageNumReadings();
  
  odometry_integrator.setAverageNumReadings( 1000000 );
  average_2n_7 = odometry_integrator.getAverage2nReadings();
  average_num_7 = odometry_integrator.getAverageNumReadings();
  
  odometry_integrator.setAverageNumReadings( 10 );
  average_2n_8 = odometry_integrator.getAverage2nReadings();
  average_num_8 = odometry_integrator.getAverageNumReadings();
  
  odometry_integrator.setAverageNumReadings( 100 );
  average_2n_9 = odometry_integrator.getAverage2nReadings();
  average_num_9 = odometry_integrator.getAverageNumReadings();
  
  // Assert

  EXPECT_EQ( 3, average_2n_1 );
  EXPECT_EQ( 8, average_num_1 );

  EXPECT_EQ( 4, average_2n_2 );
  EXPECT_EQ( 16, average_num_2 );

  EXPECT_EQ( 0, average_2n_3 );
  EXPECT_EQ( 1, average_num_3 );

  EXPECT_EQ( 2, average_2n_4 );
  EXPECT_EQ( 4, average_num_4 );

  // Still last values
  EXPECT_EQ( 2, average_2n_5 );
  EXPECT_EQ( 4, average_num_5 );

  EXPECT_EQ( 16, average_2n_6 );
  EXPECT_EQ( 65536, average_num_6 );

  EXPECT_EQ( 16, average_2n_7 );
  EXPECT_EQ( 65536, average_num_7 );

  EXPECT_EQ( 3, average_2n_8 );
  EXPECT_EQ( 8, average_num_8 );

  EXPECT_EQ( 6, average_2n_9 );
  EXPECT_EQ( 64, average_num_9 );

}

// Define the unit test to verify ability to correctly calculate the average velocities
// specifically when less than the number of readings to averages have been received.
TEST( OdometryIntegratorTests, canCalculateAverageVelocities ) 
{
  unsigned int size_index = 0;
  unsigned int velocity_index = 0;

  double linear[ 18 ];
  double linear_average[ 18 ];
  double stasis[ 18 ];
  double stasis_average[ 18 ];

  double expected_linear[ 18 ];
  double expected_linear_average[ 18 ];
  double expected_stasis[ 18 ];
  double expected_stasis_average[ 18 ];

  int average_2n[ 3 ];
  int average_num[ 3 ];

  int expected_average_2n[ 3 ];
  int expected_average_num[ 3 ];

  // Establish Context
  OdometryIntegrator odometry_integrator;
  EncoderCountsGenerator count_generator;
  MovementStatusReceiver movement_status_receiver;
  differential_drive::EncoderCounts new_counts;
  BaseModel base_model( 0.5 / M_PI, 1, 0.5, 1.0, 0.5 / M_PI, 1);

  odometry_integrator.Attach(movement_status_receiver);

  count_generator.Attach(odometry_integrator);

  odometry_integrator.setBaseModel(base_model);

  // Act
  // 1) set average size
  //    Send 0 counts
  //    Verify 0 readings
  // 2) Send several matching counts
  //    Verify averages and velocities match
  // 3) set size to the same size
  //    Send 0 counts
  //    Verify 0 readings
  // 4) Send positive counts
  //    Send 0 counts
  //    Verify that the average is halved
  // 5) Send 2 different size counts to fill half the buffer
  //    Verify the average is the average of the 2 counts
  // 6) Send 2 different size counts to saturate the buffer
  //    Verify the average is the average of the 2 counts
  
  // Average size loop
  for ( int i = 5; i > 0; i -= 2 )
  {
    // 1)
    new_counts.left_count = 0;
    new_counts.right_count = 0;
    new_counts.stasis_count = 0;
    new_counts.dt_ms = 1000;

    odometry_integrator.setAverage2nReadings(i);
    count_generator.AddTicks(new_counts);

    usleep(2500);

    average_2n[ size_index ]                  = odometry_integrator.getAverage2nReadings();
    average_num[ size_index ]                 = odometry_integrator.getAverageNumReadings();

    expected_average_2n[ size_index ]         = i;
    expected_average_num[ size_index ]        = ldexp( 1.0, i );

    linear[ velocity_index ]                  = movement_status_receiver._linear;  
    linear_average[ velocity_index ]          = movement_status_receiver._linear_average;  
    stasis[ velocity_index ]                  = movement_status_receiver._stasis;  
    stasis_average[ velocity_index ]          = movement_status_receiver._stasis_average;  

    expected_linear[ velocity_index ]         = 0;
    expected_linear_average[ velocity_index ] = 0;
    expected_stasis[ velocity_index ]         = 0;
    expected_stasis_average[ velocity_index ] = 0;

    ++velocity_index;

    // 2)
    new_counts.left_count = 32;
    new_counts.right_count = 32;
    new_counts.stasis_count = 32;
    new_counts.dt_ms = 1000;

    // Reset the averaging 
    odometry_integrator.setAverage2nReadings(i);

    count_generator.AddTicks(new_counts);
    count_generator.AddTicks(new_counts);

    usleep(2500);

    linear[ velocity_index ]                  = movement_status_receiver._linear;  
    linear_average[ velocity_index ]          = movement_status_receiver._linear_average;  
    stasis[ velocity_index ]                  = movement_status_receiver._stasis;  
    stasis_average[ velocity_index ]          = movement_status_receiver._stasis_average;  

    expected_linear[ velocity_index ]         = 32;
    expected_linear_average[ velocity_index ] = 32;
    expected_stasis[ velocity_index ]         = 32;
    expected_stasis_average[ velocity_index ] = 32;

    ++velocity_index;

    // 3)
    new_counts.left_count = 0;
    new_counts.right_count = 0;
    new_counts.stasis_count = 0;
    new_counts.dt_ms = 1000;

    odometry_integrator.setAverage2nReadings(i);
    count_generator.AddTicks(new_counts);

    usleep(2500);

    linear[ velocity_index ]                  = movement_status_receiver._linear;  
    linear_average[ velocity_index ]          = movement_status_receiver._linear_average;  
    stasis[ velocity_index ]                  = movement_status_receiver._stasis;  
    stasis_average[ velocity_index ]          = movement_status_receiver._stasis_average;  

    expected_linear[ velocity_index ]         = 0;
    expected_linear_average[ velocity_index ] = 0;
    expected_stasis[ velocity_index ]         = 0;
    expected_stasis_average[ velocity_index ] = 0;

    ++velocity_index;

    // 4)
    new_counts.left_count = 32;
    new_counts.right_count = 32;
    new_counts.stasis_count = 32;
    new_counts.dt_ms = 1000;

    // Reset the averaging 
    odometry_integrator.setAverage2nReadings(i);

    count_generator.AddTicks(new_counts);

    usleep(2500);

    new_counts.left_count = 0;
    new_counts.right_count = 0;
    new_counts.stasis_count = 0;
    new_counts.dt_ms = 1000;

    count_generator.AddTicks(new_counts);

    usleep(2500);

    linear[ velocity_index ]                  = movement_status_receiver._linear;  
    linear_average[ velocity_index ]          = movement_status_receiver._linear_average;  
    stasis[ velocity_index ]                  = movement_status_receiver._stasis;  
    stasis_average[ velocity_index ]          = movement_status_receiver._stasis_average;  

    expected_linear[ velocity_index ]         = 0;
    expected_linear_average[ velocity_index ] = 16;
    expected_stasis[ velocity_index ]         = 0;
    expected_stasis_average[ velocity_index ] = 16;

    ++velocity_index;

    // 5)
    // Reset the averaging 
    odometry_integrator.setAverage2nReadings(i);

    // This is actually ( buffer / 2) + 1 to accommodate buffer size of 2 
    for ( int j = 0; j <= (average_num[ size_index ] / 4); ++j )
    {
      new_counts.left_count = 32;
      new_counts.right_count = 32;
      new_counts.stasis_count = 32;
      new_counts.dt_ms = 1000;
      count_generator.AddTicks(new_counts);

      usleep(2500);

      new_counts.left_count = 16;
      new_counts.right_count = 16;
      new_counts.stasis_count = 16;
      new_counts.dt_ms = 1000;
      count_generator.AddTicks(new_counts);

      usleep(2500);
    }

    linear[ velocity_index ]                  = movement_status_receiver._linear;  
    linear_average[ velocity_index ]          = movement_status_receiver._linear_average;  
    stasis[ velocity_index ]                  = movement_status_receiver._stasis;  
    stasis_average[ velocity_index ]          = movement_status_receiver._stasis_average;  

    expected_linear[ velocity_index ]         = 16;
    expected_linear_average[ velocity_index ] = 24;
    expected_stasis[ velocity_index ]         = 16;
    expected_stasis_average[ velocity_index ] = 24;

    ++velocity_index;

    // 6)
    // Reset the averaging 
    odometry_integrator.setAverage2nReadings(i);

    // This should fill the buffer twice
    for ( int j = 0; j < average_num[ size_index ]; ++j )
    {
      new_counts.left_count = 32;
      new_counts.right_count = 32;
      new_counts.stasis_count = 32;
      new_counts.dt_ms = 1000;
      count_generator.AddTicks(new_counts);

      usleep(2500);

      new_counts.left_count = 16;
      new_counts.right_count = 16;
      new_counts.stasis_count = 16;
      new_counts.dt_ms = 1000;
      count_generator.AddTicks(new_counts);

      usleep(2500);
    }

    linear[ velocity_index ]                  = movement_status_receiver._linear;  
    linear_average[ velocity_index ]          = movement_status_receiver._linear_average;  
    stasis[ velocity_index ]                  = movement_status_receiver._stasis;  
    stasis_average[ velocity_index ]          = movement_status_receiver._stasis_average;  

    expected_linear[ velocity_index ]         = 16;
    expected_linear_average[ velocity_index ] = 24;
    expected_stasis[ velocity_index ]         = 16;
    expected_stasis_average[ velocity_index ] = 24;

    ++velocity_index;

    ++size_index;
  }

  // Assert

  for ( int i = 0; i < 3; ++i )
  {
    EXPECT_EQ( expected_average_2n[ i ], average_2n[ i ] );
    EXPECT_EQ( expected_average_num[ i ], average_num[ i ] );
  }

  for ( int i = 0; i < 18; ++i )
  {
    // Debugging statement - where are my errors
    //std::cout << "i: " << i << std::endl;

    EXPECT_FLOAT_EQ( expected_linear[ i ], linear[ i ] );
    EXPECT_FLOAT_EQ( expected_linear_average[ i ], linear_average[ i ] );
    EXPECT_FLOAT_EQ( expected_stasis[ i ], stasis[ i ] );
    EXPECT_FLOAT_EQ( expected_stasis_average[ i ], stasis_average[ i ] );
  }
}

}
