#include "ros/ros.h"

#include "data_robot/Bumpers.h"
#include "geometry_msgs/Twist.h"

#define WANDER_SPEED  0.2 // m/s
#define WANDER_TURN   0.5 // rad/s
#define WANDER_TIME   3.0 // s
#define LOOP_RATE     10.0
#define LOOP_TIME     0.1

geometry_msgs::Twist Wander();

/** This program is a simple bumpers test.
 * 
 */
int main(int argc, char **argv)
{
  geometry_msgs::Twist  new_cmd;
  data_robot::Bumpers   bumpers;

  ros::init(argc, argv, "bumper_wander");

  ros::NodeHandle nh;

  ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate loop_rate(LOOP_RATE);

  while ( ros::ok() )
  {
    new_cmd = Wander();
    
    ROS_INFO( "Publishing cmd_vel: Speed: %.2f Turn: %.2f", new_cmd.linear.x, new_cmd.angular.z );

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

