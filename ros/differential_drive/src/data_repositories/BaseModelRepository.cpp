#include <ros/ros.h>

#include "BaseModelRepository.hpp"
 
namespace differential_drive_data_repositories
{
BaseModelRepository::BaseModelRepository()
{
}

BaseModelRepository::BaseModelRepository( differential_drive_core::BaseModel* p_new_model )
{
  SetBaseModel( p_new_model );
}

BaseModelRepository::~BaseModelRepository()
{
}

void BaseModelRepository::QueryBaseGeometry()
{
  _p_base_model->SetBaseGeometry( RosQueryBaseGeometry() );
}

void BaseModelRepository::PersistBaseGeometry()
{
}

void BaseModelRepository::StartListeningForUpdates()
{
}

void BaseModelRepository::StopListeningForUpdates()
{
}

void BaseModelRepository::SetBaseModel( differential_drive_core::BaseModel* p_new_model )
{
  _p_base_model = p_new_model;
}

/** Returns a @c BaseGeometry_T from ROS parameters or assigns defaults.
 */
differential_drive_core::BaseGeometry_T BaseModelRepository::RosQueryBaseGeometry() const
{
  differential_drive_core::BaseGeometry_T base_geometry;

  double wheel_diameter;
  double wheel_base;
  double wheel_ratio;
  int wheel_ticks;

  double stasis_diameter;
  int stasis_ticks;

  ros::param::param<double>( "~drive_wheel_diameter", wheel_diameter, 1.0 );
  ros::param::param<double>( "~drive_wheel_base", wheel_base, 1.0 );
  ros::param::param<double>( "~drive_wheel_ratio", wheel_ratio, 1.0 );
  ros::param::param<int>( "~drive_wheel_encoder_ticks", wheel_ticks, 128 );

  if ( wheel_diameter <= 0.0 )
  {
    ROS_ERROR_NAMED(  "BaseModelRepository", "drive_wheel_diameter <= 0! : %f changed to %f.",
                      wheel_diameter, wheel_diameter * -1.0 );

    wheel_diameter *= -1.0;
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

  if ( wheel_ticks <= 0 )
  {
    ROS_ERROR_NAMED(  "BaseModelRepository", "drive_wheel_encoder_ticks <= 0! : %d changed to %d.",
                      wheel_ticks, wheel_ticks * -1 );
    wheel_ticks *= -1;
  }

  if ( ros::param::get( "~stasis_wheel_diameter", stasis_diameter) ) 
  {
    if ( stasis_diameter > 0.0 )
    {
      ros::param::param<int>( "~stasis_wheel_encoder_ticks", stasis_ticks, 100 );

      if ( stasis_ticks <= 0.0 )
      {
        ROS_ERROR_NAMED(  "BaseModelRepository", "stasis_wheel_encoder_ticks <= 0! Disabling stasis wheel." );

        stasis_diameter = 2.0;
        stasis_ticks = -1;
      }
    }
    else
    {
      ROS_ERROR_NAMED(  "BaseModelRepository", "stasis_wheel_diameter <= 0! Disabling stasis wheel." );

      stasis_diameter = 2.0;
      stasis_ticks = -1;
    }
  }
  else
  {
    stasis_diameter = 2.0;
    stasis_ticks = -1;
  }

  // Assign return values
  base_geometry.wheel_radius = wheel_diameter / 2.0;
  base_geometry.wheel_base = wheel_base;
  base_geometry.wheel_ratio = wheel_ratio;
  base_geometry.wheel_ticks = wheel_ticks;
  base_geometry.stasis_radius = stasis_diameter / 2.0;
  base_geometry.stasis_ticks = stasis_ticks;

  return base_geometry;
}
}
