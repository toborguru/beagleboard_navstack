#include <ros/ros.h>

#include "BumperIndexesRepository.hpp"
 
namespace data_robot_data_repositories
{
/** Returns bumper index from ROS parameters or assigns default.
 */
int8_t BumperIndexesRepository::QueryFrontBumperIndex() const
{
  int index;

  ros::param::param<int>( "~front_bumper_index", index, -1 );

  ROS_INFO( "Front bumper index: %d", index );
  
  return index;
}

/** Returns bumper index from ROS parameters or assigns default.
 */
int8_t BumperIndexesRepository::QueryFrontLeftBumperIndex() const
{
  int index;

  ros::param::param<int>( "~front_left_bumper_index", index, -1 );
  
  return index;
}

/** Returns bumper index from ROS parameters or assigns default.
 */
int8_t BumperIndexesRepository::QueryFrontRightBumperIndex() const
{
  int index;

  ros::param::param<int>( "~front_right_bumper_index", index, -1 );
  
  return index;
}

/** Returns bumper index from ROS parameters or assigns default.
 */
int8_t BumperIndexesRepository::QueryRearBumperIndex() const
{
  int index;

  ros::param::param<int>( "~rear_bumper_index", index, -1 );
  
  return index;
}

/** Returns bumper index from ROS parameters or assigns default.
 */
int8_t BumperIndexesRepository::QueryRearLeftBumperIndex() const
{
  int index;

  ros::param::param<int>( "~rear_left_bumper_index", index, -1 );
  
  return index;
}

/** Returns bumper index from ROS parameters or assigns default.
 */
int8_t BumperIndexesRepository::QueryRearRightBumperIndex() const
{
  int index;

  ros::param::param<int>( "~rear_right_bumper_index", index, -1 );
  
  return index;
}

}
