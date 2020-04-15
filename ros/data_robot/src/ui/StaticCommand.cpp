#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

#define WANDER_SPEED  0.3 // m/s
#define WANDER_TURN   0.35 // rad/s
#define WANDER_TIME   5.0 // s
#define LOOP_RATE     20.0
#define LOOP_TIME     0.05


/** This program is a simple bumpers test.
 * 
 */
int main(int argc, char **argv)
{
  double linear;
  double angular;

  geometry_msgs::Twist  new_cmd;

  ros::init(argc, argv, "static_vel_command");

  ros::NodeHandle nh("~");

  nh.getParam("linear", linear);
  nh.getParam("angular", angular);

  ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Rate loop_rate(LOOP_RATE);

  while ( ros::ok() )
  {
    new_cmd.linear.x = linear;
    new_cmd.angular.z = angular;

    ROS_DEBUG( "Publishing cmd_vel: Speed: %.2f Turn: %.2f", new_cmd.linear.x, new_cmd.angular.z );

    cmd_vel_publisher.publish( new_cmd );

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
