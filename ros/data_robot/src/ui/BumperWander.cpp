#include "ros/ros.h"

#include "data_robot/Bumpers.h"
#include "geometry_msgs/Twist.h"

#define WANDER_SPEED  0.2 // m/s
#define WANDER_TURN   0.5 // rad/s
#define WANDER_TIME   3.0 // s
#define LOOP_RATE     10.0
#define LOOP_TIME     0.1

#define BUMP_SPEED    0.1
#define BUMP_TURN     1.0

#define BUMP_TIME1      0.5
#define BUMP_TIME2_MAX  1.0

#define BUMP_CENTER   20000
#define BUMP_SIDE     10000
#define BUMP_DRAIN    320
#define BUMP_FIRST_MULT 4

int8_t m_bump_direction = data_robot::Bumpers::NONE;

void BumpersCallback( data_robot::Bumpers current_bumps );
bool EscapeCollision( geometry_msgs::Twist* p_cmd_vel, uint8_t bump_direction );
geometry_msgs::Twist Wander();

/** This program is a simple bumpers test.
 * 
 */
int main(int argc, char **argv)
{
  geometry_msgs::Twist  new_cmd;

  ros::init(argc, argv, "bumper_wander");

  ros::NodeHandle nh;

  ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Subscriber bumpers_subscriber = nh.subscribe( "bumpers", 1, BumpersCallback );

  ros::Rate loop_rate(LOOP_RATE);

  while ( ros::ok() )
  {
    new_cmd = Wander();

    EscapeCollision( &new_cmd, m_bump_direction );
    
    //ROS_INFO( "Publishing cmd_vel: Speed: %.2f Turn: %.2f", new_cmd.linear.x, new_cmd.angular.z );

    cmd_vel_publisher.publish( new_cmd );

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

geometry_msgs::Twist Wander()
{
  static double time_left = 0.0;
  static bool left = true;

  geometry_msgs::Twist cmd;

  time_left -= LOOP_TIME;

  if ( time_left <= 0.0 )
  {
    time_left = WANDER_TIME;
    left = !left;
  }

  cmd.linear.x = WANDER_SPEED;

  if ( left )
  {
    cmd.angular.z = WANDER_TURN;
  }
  else
  {
    cmd.angular.z = -1.0 * WANDER_TURN;
  }

  return cmd;
}

void BumpersCallback( data_robot::Bumpers current_bumps )
{
  m_bump_direction = current_bumps.bump_direction;

  if ( m_bump_direction )
  {
    ROS_INFO( "BUMP: %d", m_bump_direction );
  }
}

bool EscapeCollision( geometry_msgs::Twist* p_cmd_vel, uint8_t bump_direction )
{
  static uint8_t last_direction = data_robot::Bumpers::NONE;
  
  static float time1 = 0.0;
  static float time2 = 0.0;
  static float turn_speed = 0.0;
  
  static int32_t integration_bucket = 0;
  static int32_t response_value = 0;

  static bool front = false;
  static bool left = false;

  bool new_bump = false;
  bool active_command = false;

  int rand_num;

  if ( NULL == p_cmd_vel )
  {
    return false;
  }

  if ( (bump_direction != data_robot::Bumpers::NONE) && (bump_direction != last_direction) )
  {
    new_bump = true;
  }

  last_direction = bump_direction;

  if ( new_bump )
  {
    time1 = BUMP_TIME1;

    if ( data_robot::Bumpers::FRONT == bump_direction )
    {
      front = true;

      // No ongoing response
      if ( integration_bucket == 0 )
      {
        rand_num = rand();
        left = rand_num % 2;

        integration_bucket += BUMP_FIRST_MULT * BUMP_CENTER;
        response_value = integration_bucket;
      }
      else // If we are already in a bump keep responding in the same direction
      {   
        response_value = integration_bucket;
        integration_bucket += BUMP_CENTER;
      } 
    }
    else if ( data_robot::Bumpers::FRONT_LEFT == bump_direction )
    {
      front = true;

      // No ongoing response
      if ( integration_bucket == 0 )
      {
        left = true;

        integration_bucket += BUMP_FIRST_MULT * BUMP_SIDE;
        response_value = integration_bucket;
      }
      else // If we are already in a bump keep responding in the same direction
      {   
        response_value = integration_bucket;
        integration_bucket += BUMP_SIDE;
      } 
    }
    else if ( data_robot::Bumpers::FRONT_RIGHT == bump_direction )
    {
      front = true;

      // No ongoing response
      if ( integration_bucket == 0 )
      {
        left = false;

        integration_bucket += BUMP_FIRST_MULT * BUMP_SIDE;
        response_value = integration_bucket;
      }
      else // If we are already in a bump keep responding in the same direction
      {   
        response_value = integration_bucket;
        integration_bucket += BUMP_SIDE;
      } 
    }
    else if ( data_robot::Bumpers::REAR == bump_direction )
    {
      front = false;

      // No ongoing response
      if ( integration_bucket == 0 )
      {
        rand_num = rand();
        left = rand_num % 2;

        integration_bucket += BUMP_FIRST_MULT * BUMP_CENTER;
        response_value = integration_bucket;
      }
      else // If we are already in a bump keep responding in the same direction
      {   
        response_value = integration_bucket;
        integration_bucket += BUMP_CENTER;
      } 
    }
    else if ( data_robot::Bumpers::REAR_LEFT == bump_direction )
    {
      front = false;

      // No ongoing response
      if ( integration_bucket == 0 )
      {
        left = true;

        integration_bucket += BUMP_FIRST_MULT * BUMP_SIDE;
        response_value = integration_bucket;
      }
      else // If we are already in a bump keep responding in the same direction
      {   
        response_value = integration_bucket;
        integration_bucket += BUMP_SIDE;
      } 
    }
    else if ( data_robot::Bumpers::REAR_RIGHT == bump_direction )
    {
      front = false;

      // No ongoing response
      if ( integration_bucket == 0 )
      {
        left = false;

        integration_bucket += BUMP_FIRST_MULT * BUMP_SIDE;
        response_value = integration_bucket;
      }
      else // If we are already in a bump keep responding in the same direction
      {   
        response_value = integration_bucket;
        integration_bucket += BUMP_SIDE;
      } 
    }
  }

  if ( time1 > 0.0 )
  {
    active_command = true;
    time1 -= LOOP_TIME;

    if ( front )
    {
      p_cmd_vel->linear.x = -1.0 * BUMP_SPEED;    
    }
    else
    {
      p_cmd_vel->linear.x = BUMP_SPEED;    
    }

    // Do not turn
    p_cmd_vel->angular.z = 0.0;

    if ( time1 <= 0.0 )
    {
      time2 = BUMP_TIME2_MAX;
      turn_speed = BUMP_TURN * (double)response_value / (double)BUMP_CENTER;
    }
  }
  else if ( time2 > 0.0 )
  {
    active_command = true;
    time2 -= LOOP_TIME;

    if ( front )
    {
      p_cmd_vel->linear.x = -1.0 * BUMP_SPEED;    
    }
    else
    {
      p_cmd_vel->linear.x = BUMP_SPEED;    
    }

    if ( left )
    {
      p_cmd_vel->angular.z = -1.0 * turn_speed;
    }
    else
    {
      p_cmd_vel->angular.z = turn_speed;
    }
  }

  // Drain a little from our integrater
  integration_bucket -= BUMP_DRAIN;

  if ( integration_bucket < 0 )
  {
    integration_bucket = 0;
  }

  return active_command;
}
