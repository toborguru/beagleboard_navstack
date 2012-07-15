#include <ros/ros.h>

#include "BaseModelRepository.hpp"
 
namespace diff_drive_data_repositories
{
/** Returns a @c BaseGeometry_T from ROS parameters or assigns defaults.
 */
diff_drive_core::BaseGeometry_T BaseModelRepository::QueryBaseGeometry() const
{
  diff_drive_core::BaseGeometry_T base_geometry;

  double wheel_radius;
  double wheel_base;
  double wheel_ratio;
  int wheel_ticks;

  double stasis_radius;
  int stasis_ticks;

  ros::param::param<double>( "~drive_wheel_radius", wheel_radius, 1.0 );
  ros::param::param<double>( "~drive_wheel_base", wheel_base, 1.0 );
  ros::param::param<double>( "~drive_wheel_ratio", wheel_ratio, 1.0 );
  ros::param::param<int>( "~drive_wheel_encoder_ticks", wheel_ticks, 128 );

  if ( wheel_radius <= 0.0 )
  {
    ROS_ERROR_NAMED(  "BaseModelRepository", "drive_wheel_radius <= 0! : %f changed to %f.",
                      wheel_radius, wheel_radius * -1.0 );

    wheel_radius *= -1.0;
  }

  if ( wheel_base <= 0.0 )
  {
    ROS_ERROR_NAMED(  "BaseModelRepository", "drive_wheel_base <= 0! : %f changed to %f.",
                      wheel_base, wheel_base * -1.0 );
    wheel_base *= -1.0;
  }

  if ( wheel_ratio <= 0.0 )
  {
    ROS_ERROR_NAMED(  "BaseModelRepository", "drive_wheel_ratio <= 0! : %f changed to %f.",
                      wheel_ratio, wheel_ratio * -1.0 );
    wheel_ratio *= -1.0;
  }

  if ( wheel_ticks <= 0.0 )
  {
    ROS_ERROR_NAMED(  "BaseModelRepository", "drive_wheel_encoder_ticks <= 0! : %d changed to %d.",
                      wheel_ticks, wheel_ticks * -1 );
    wheel_ticks *= -1;
  }

  if ( ros::param::get( "~stasis_wheel_radius", stasis_radius) ) 
  {
    if ( stasis_radius > 0.0 )
    {
      ros::param::param<int>( "~stasis_wheel_encoder_ticks", stasis_ticks, 100 );

      if ( stasis_ticks <= 0.0 )
      {
        ROS_ERROR_NAMED(  "BaseModelRepository", "stasis_wheel_encoder_ticks <= 0! Disabling stasis wheel." );

        stasis_radius = 1.0;
        stasis_ticks = -1;
      }
    }
    else
    {
      ROS_ERROR_NAMED(  "BaseModelRepository", "stasis_wheel_radius <= 0! Disabling stasis wheel." );

      stasis_radius = 1.0;
      stasis_ticks = -1;
    }
  }
  else
  {
    stasis_radius = 1.0;
    stasis_ticks = -1;
  }


  // Assign return values
  base_geometry.wheel_radius = wheel_radius;
  base_geometry.wheel_base = wheel_base;
  base_geometry.wheel_ratio = wheel_ratio;
  base_geometry.wheel_ticks = wheel_ticks;
  base_geometry.stasis_radius = stasis_radius;
  base_geometry.stasis_ticks = stasis_ticks;

  return base_geometry;
}
}
