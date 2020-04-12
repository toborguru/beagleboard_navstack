/** @file
 *  Uses ROS logging MACROs to register log messages.
 *  
 */
 
#include <ros/ros.h>

#include "LogPublisherEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
/** Default constructor
 */
LogPublisherEndpoint::LogPublisherEndpoint() 
{ 
}

/** Publishes the log message in ROS
 *  
 */
void LogPublisherEndpoint::Publish( data_robot_core::LogLevel_T level, data_robot_core::LogFlags_T flags, const std::string& log_message )
{
  if ( level == data_robot_core::DEBUG )
  {
    ROS_DEBUG( "%s", log_message );
  }
  else if ( level == data_robot_core::INFO )
  {
    ROS_INFO( "%s", log_message );
  }
  else if ( level == data_robot_core::WARNING )
  {
    ROS_WARN( "%s", log_message );
  }
  else if ( level == data_robot_core::ERROR )
  {
    ROS_ERROR( "%s", log_message );
  }
  else if ( level == data_robot_core::FATAL )
  {
    ROS_FATAL( "%s", log_message );
  }
}
}
