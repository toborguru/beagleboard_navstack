#include <gtest/gtest.h>
#include <ros/ros.h>
 
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
 
  // Initialize ROS and set node name
  ros::init(argc, argv, "diff_drive");

  return RUN_ALL_TESTS();
}
