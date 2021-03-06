/** @file
 * Unit test for TwistConverter class.
 *
 * @author Sawyer Larkin (SJL toborguru)
 */

#include <gtest/gtest.h>

#include "geometry_msgs/Twist.h"

#include "TwistConverter.hpp"
#include "ITickVelocityListener.hpp"
#include "ITwistSubscriberEndpoint.hpp"
#include "BaseModel.hpp"

using namespace differential_drive_core;
 
namespace differential_drive_core_test_core
{
// Will be used by unit test to check tick_velocity
struct TickVelocityReceiver : public differential_drive_core::ITickVelocityListener
{
  TickVelocityReceiver()
    : _count_of_tick_velocities_received(0),
      _linear(0),
      _angular(0)
  { }

  int _count_of_tick_velocities_received;
  int _linear;
  int _angular;

  void onTickVelocityAvailableEvent(const differential_drive::TickVelocity& tick_velocity)
  {
    ++_count_of_tick_velocities_received;

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
struct TwistGenerator : public differential_drive_core::ITwistSubscriberEndpoint
{
  TwistGenerator() 
    : _subscribed(false)
  {}

  void NewTwistCommand( const geometry_msgs::Twist twist )
  {
    for (unsigned int i= 0; i < _twist_listeners.size(); ++i) 
    {
      _twist_listeners[i]->onTwistAvailableEvent(twist);
    }
  }

  void subscribe()
  {
    _subscribed = true;
  }
  
  void unsubscribe()
  {
    _subscribed = false;
  }
  
  bool isSubscribed()
  {
    return _subscribed;
  }
  
  void attach( ITwistListener& twist_listener )
  {
    _twist_listeners.push_back(&twist_listener);
  }

  void detach( ITwistListener& twist_listener )
  { 
    // Using the remove-erase idiom
    std::vector<ITwistListener*>& vec = _twist_listeners; // use shorter name
    vec.erase( std::remove(vec.begin(), vec.end(), &twist_listener), vec.end() );

    if ( _twist_listeners.size() == 0 )
    { 
      unsubscribe();
    }
  }

  bool _subscribed;
  std::vector<ITwistListener*> _twist_listeners;
};

// Define the unit test to verify ability to listen for Ticks and generate TickVelocity
TEST( TwistConverterTests, canSendTwistAndReceiveTickVelocity ) 
{
  // Establish Context
  TwistConverter twist_conveter;
  TwistGenerator twist_generator;
  TickVelocityReceiver tick_velocity_receiver;
  geometry_msgs::Twist new_twist;

  twist_conveter.attach(tick_velocity_receiver);
  twist_generator.attach(twist_conveter);

  // Act
  new_twist.linear.x = 1.0;
  new_twist.angular.z = 0.0;
  twist_generator.NewTwistCommand( new_twist );
  twist_generator.NewTwistCommand( new_twist );
  twist_generator.NewTwistCommand( new_twist );
  twist_generator.NewTwistCommand( new_twist );

  // Assert

  // Check the results of send four twist commands
  ASSERT_TRUE(tick_velocity_receiver._count_of_tick_velocities_received == 4);
}

// Define the unit test to verify ability to convert twist messages into ticks
TEST( TwistConverterTests, canconvertTwistToTicks )
{
  int linear1, linear2, angular1, angular2;

  // Establish Context
  TwistConverter twist_conveter;
  TwistGenerator twist_generator;
  TickVelocityReceiver tick_velocity_receiver;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );
  geometry_msgs::Twist new_twist;

  twist_conveter.attach(tick_velocity_receiver);
  twist_conveter.setBaseModel(base_model);

  twist_generator.attach(twist_conveter);

  // Act
  new_twist.linear.x = 1.0;
  new_twist.angular.z = 0.0;
  twist_generator.NewTwistCommand( new_twist );
  linear1 = tick_velocity_receiver._linear;
  angular1 = tick_velocity_receiver._angular;

  new_twist.linear.x = 0.0;
  new_twist.angular.z = 1.0;
  twist_generator.NewTwistCommand( new_twist );
  linear2 = tick_velocity_receiver._linear;
  angular2 = tick_velocity_receiver._angular;

  // Assert

  ASSERT_EQ( 100, linear1 ); 
  ASSERT_EQ( 0 , angular1 );

  ASSERT_EQ( 0, linear2 ); 
  ASSERT_EQ( 50 , angular2 );
}

// Define the unit test to verify ability to convert twist messages into ticks
TEST( TwistConverterTests, canconvertTwistToTicksCalibrated )
{
  int linear1, linear2, angular1, angular2;

  // Establish Context
  TwistConverter twist_conveter;
  TwistGenerator twist_generator;
  TickVelocityReceiver tick_velocity_receiver;
  BaseModel base_model( 0.5 / M_PI, 100, 0.5, 0.95 );
  geometry_msgs::Twist new_twist;

  twist_conveter.attach(tick_velocity_receiver);
  twist_conveter.setBaseModel(base_model);

  twist_generator.attach(twist_conveter);

  // Act
  new_twist.linear.x = 1.0;
  new_twist.angular.z = 0.0;
  twist_generator.NewTwistCommand( new_twist );
  linear1 = tick_velocity_receiver._linear;
  angular1 = tick_velocity_receiver._angular;

  new_twist.linear.x = 0.0;
  new_twist.angular.z = 8.0;
  twist_generator.NewTwistCommand( new_twist );
  linear2 = tick_velocity_receiver._linear;
  angular2 = tick_velocity_receiver._angular;

  // Assert

  ASSERT_EQ( 100, linear1 ); 
  ASSERT_EQ( -5, angular1 );

  ASSERT_EQ( -5, linear2 ); 
  ASSERT_EQ( 400 , angular2 );
}
}
