#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#define LOOP_RATE     50.0


/** This program is a simple bumpers test.
 * 
 */
int main(int argc, char **argv)
{
  double linear;
  double diameter;
  double radius;
  double angular;
  double roll, pitch, yaw;
  double last_yaw = 0.0;
  int flip_dir;
  int flip_locked;

  geometry_msgs::Twist  new_cmd;
  tf::StampedTransform transform;

  ros::init(argc, argv, "figure_eights");

  ros::NodeHandle nh("~");

  tf::TransformListener listener;

  ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Rate loop_rate(LOOP_RATE);

  while ( ros::ok() )
  {
    nh.getParam("speed", linear);
    nh.getParam("diameter", diameter);

    try
    {
      listener.lookupTransform("/odom", "/base_link",  
          ros::Time(0), transform);

      tf::Quaternion q = transform.getRotation();
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    radius = diameter / 2.0;

    angular = linear / radius;

    //printf("Yaw: %f Last Yaw: %f\n", last_yaw, yaw);

    if (( (last_yaw * yaw) < 0.0 ) && (abs(yaw) < 1.0) && !flip_locked) // Mixed signs
    {
      flip_dir = !flip_dir;
      flip_locked = 1;
    }
    last_yaw = yaw;

    if ( abs(yaw) > 2 )
    {
      flip_locked = 0;
    }

    if (flip_dir)
    {
      angular *= -1.0;
    }

    new_cmd.linear.x = linear;
    new_cmd.angular.z = angular;

    ROS_DEBUG( "Publishing cmd_vel: Speed: %.2f Turn: %.2f", new_cmd.linear.x, new_cmd.angular.z );

    cmd_vel_publisher.publish( new_cmd );

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
