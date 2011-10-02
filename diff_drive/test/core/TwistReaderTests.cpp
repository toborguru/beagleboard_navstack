/** @file
 * Unit test for TwistReader class.
 *
 * @author Sawyer Larkin (SJL toborguru)
 */

#include <gtest/gtest.h>

#include "geometry_msgs/Twist.h"

#include "TwistReader.hpp"
#include "ITickVelocityListener.hpp"
#include "ITwistEndpoint.hpp"
#include "BaseModel.hpp"

using namespace diff_drive_core;
 
namespace diff_drive_core_test_core
{
// Will be used by unit test to check tick_velocity
struct TickVelocityReceiver : public diff_drive_core::ITickVelocityListener
{
  TickVelocityReceiver()
    : _count_of_tick_velocities_received(0),
      _linear(0),
      _angular(0)
  { }

  int _count_of_tick_velocities_received;
  int _linear;
  int _angular;

  void OnTickVelocityAvailableEvent(const diff_drive::TickVelocity& tick_velocity)
  {
    _count_of_tick_velocities_received++;

    _linear = tick_velocity.linear_ticks_sec;
    _angular = tick_velocity.angular_ticks_sec;

    // Output the read values to the terminal; this isn't the
    // unit test, but merely a helpful means to show what's going on.

#if 0
    std::cout << "TickVelocity sent to TickVelocityReceiver with linear/sec: " 
              << _linear
              << ", angular/sec: "
              << _angular
              << std::endl;
#endif
  }
};

// Will be used by the unit test to produce encoder ticks
struct TwistGenerator : public diff_drive_core::ITwistEndpoint
{
  TwistGenerator() 
    : _subscribed(false)
  {}

  void NewTwistCommand( const geometry_msgs::Twist twist )
  {
    for (unsigned int i= 0; i < _twist_listeners.size(); i++) 
    {
      _twist_listeners[i]->OnTwistAvailableEvent(twist);
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
  
  void Attach( ITwistListener& twist_listener )
  {
    _twist_listeners.push_back(&twist_listener);
  }

  bool _subscribed;
  std::vector<ITwistListener*> _twist_listeners;
};

// Define the unit test to verify ability to listen for Ticks and generate TickVelocity
TEST( TwistReaderTests, canSendTwistAndReceiveTickVelocity ) 
{
  // Establish Context
  TwistReader twist_reader;
  TwistGenerator twist_generator;
  TickVelocityReceiver tick_velocity_receiver;
  geometry_msgs::Twist new_twist;

  twist_reader.Attach(tick_velocity_receiver);
  twist_generator.Attach(twist_reader);

  // Act
  new_twist.linear.x = 1.0;
  new_twist.angular.z = 0.0;
  twist_generator.NewTwistCommand( new_twist );
  twist_generator.NewTwistCommand( new_twist );
  twist_generator.NewTwistCommand( new_twist );
  twist_generator.NewTwistCommand( new_twist );

  // Assert

  // Check the results of send four twist commands
  EXPECT_TRUE(tick_velocity_receiver._count_of_tick_velocities_received == 4);
}
#if 0
// Define the unit test to verify ability to integrate and estimate position
TEST( TwistReaderTests, canCalculateEstimatedPosition) 
{
  // Establish Context
  double distance;
  double theta;

  TwistReader twist_reader;
  TwistGenerator twist_generator;
  TickVelocityReceiver tick_velocity_receiver;
  diff_drive::Twist new_twist;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );

  twist_reader.Attach(tick_velocity_receiver);
  twist_reader.SetBaseModel(base_model);

  twist_generator.Attach(twist_reader);

  // Act
  new_twist.left_twist = 100;
  new_twist.right_twist = 100;
  new_twist.dt_ms = 100;
  twist_generator.AddTicks(new_twist);
  twist_generator.AddTicks(new_twist);
  twist_generator.AddTicks(new_twist);
  twist_generator.AddTicks(new_twist);

  distance = tick_velocity_receiver._x;

  new_twist.left_twist = 0;
  new_twist.right_twist = 50;
  new_twist.dt_ms = 100;
  twist_generator.AddTicks(new_twist);
  twist_generator.AddTicks(new_twist);
  twist_generator.AddTicks(new_twist);

  theta = tick_velocity_receiver._theta;

  twist_generator.AddTicks(new_twist);

  // Assert

  EXPECT_FLOAT_EQ(distance, 4.0);
  EXPECT_FLOAT_EQ(theta, 3.0);
  EXPECT_LT(tick_velocity_receiver._theta, 0.0);
}
#endif
}
