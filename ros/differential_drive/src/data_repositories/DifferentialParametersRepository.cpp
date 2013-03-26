#include <ros/ros.h>

#include "DifferentialParametersRepository.hpp"
 
namespace differential_drive_data_repositories
{
DifferentialParametersRepository::DifferentialParametersRepository()
                    : _p_base_model(NULL),
                      _p_odometry_integrator(NULL),
                      _p_geometry_reconfigure_server(NULL)
{
}

DifferentialParametersRepository::DifferentialParametersRepository( differential_drive_core::BaseModel* p_new_model,
                                                                    differential_drive_core::OdometryIntegrator* p_new_integrator )
                    : _p_geometry_reconfigure_server(NULL)
{
  SetBaseModel( p_new_model );
  SetOdometryIntegrator( p_new_integrator);
}

DifferentialParametersRepository::~DifferentialParametersRepository()
{
  StopListeningForUpdates();
}

void DifferentialParametersRepository::QueryBaseGeometry()
{
  if ( _p_base_model != NULL )
  {
    _p_base_model->SetBaseGeometry( RosQueryBaseGeometry() );
  }
}

void DifferentialParametersRepository::PersistBaseGeometry()
{
  if ( _p_base_model != NULL )
  {
    RosPersistBaseGeometry( _p_base_model->GetBaseGeometry() );
  }
}

void DifferentialParametersRepository::StartListeningForUpdates()
{
  if ( _p_geometry_reconfigure_server == NULL )
  {
    _p_geometry_reconfigure_server = new dynamic_reconfigure::Server<differential_drive::DifferentialParametersConfig>;
    dynamic_reconfigure::Server<differential_drive::DifferentialParametersConfig>::CallbackType call_back_type;

    call_back_type = boost::bind( &DifferentialParametersRepository::UpdateParametersCallBack, this, _1, _2 );

    _p_geometry_reconfigure_server->setCallback( call_back_type );
  }
}

void DifferentialParametersRepository::StopListeningForUpdates()
{
  if ( _p_geometry_reconfigure_server != NULL )
  {
    delete _p_geometry_reconfigure_server;
    _p_geometry_reconfigure_server = NULL;
  }
}

void DifferentialParametersRepository::UpdateParametersCallBack( differential_drive::DifferentialParametersConfig &config, uint32_t level)
{
  differential_drive_core::BaseGeometry_T base_geometry;

  base_geometry.wheel_radius  = config.drive_wheel_diameter / 2.0;
  base_geometry.wheel_base    = config.drive_wheel_base;
  base_geometry.wheel_ratio   = config.drive_wheel_ratio;
  base_geometry.wheel_ticks   = config.drive_wheel_encoder_ticks;

  base_geometry.stasis_radius = config.stasis_wheel_diameter / 2.0;
  base_geometry.stasis_ticks =  config.stasis_wheel_encoder_ticks;

  if ( _p_base_model != NULL )
  {  
    _p_base_model->SetBaseGeometry( base_geometry );
  }

  PersistBaseGeometry();
}

void DifferentialParametersRepository::SetBaseModel( differential_drive_core::BaseModel* p_new_model )
{
  _p_base_model = p_new_model;
}

void DifferentialParametersRepository::SetOdometryIntegrator( differential_drive_core::OdometryIntegrator* p_new_integrator )
{
  _p_odometry_integrator = p_new_integrator;
}

/** Returns a @c BaseGeometry_T from ROS parameters or assigns defaults.
 */
differential_drive_core::BaseGeometry_T DifferentialParametersRepository::RosQueryBaseGeometry() const
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
    ROS_ERROR_NAMED(  "DifferentialParametersRepository", "drive_wheel_diameter <= 0! : %f changed to %f.",
                      wheel_diameter, wheel_diameter * -1.0 );

    wheel_diameter *= -1.0;
  }

  if ( wheel_base <= 0.0 )
  {
    ROS_ERROR_NAMED(  "DifferentialParametersRepository", "drive_wheel_base <= 0! : %f changed to %f.",
                      wheel_base, wheel_base * -1.0 );
    wheel_base *= -1.0;
  }

  if ( wheel_ratio <= 0.0 )
  {
    ROS_ERROR_NAMED(  "DifferentialParametersRepository", "drive_wheel_ratio <= 0! : %f changed to %f.",
                      wheel_ratio, wheel_ratio * -1.0 );
    wheel_ratio *= -1.0;
  }

  if ( wheel_ticks <= 0 )
  {
    ROS_ERROR_NAMED(  "DifferentialParametersRepository", "drive_wheel_encoder_ticks <= 0! : %d changed to %d.",
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
        ROS_ERROR_NAMED(  "DifferentialParametersRepository", "stasis_wheel_encoder_ticks <= 0! Disabling stasis wheel." );
        
        ros::param::set("~stasis_wheel_enabled", false );
        stasis_diameter = 2.0;
        stasis_ticks = -1;
      }
    }
    else
    {
      ROS_ERROR_NAMED(  "DifferentialParametersRepository", "stasis_wheel_diameter <= 0! Disabling stasis wheel." );

      ros::param::set("~stasis_wheel_enabled", false );
      stasis_diameter = 2.0;
      stasis_ticks = -1;
    }
  }
  else
  {
    ros::param::set("~stasis_wheel_enabled", false );
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

void DifferentialParametersRepository::RosPersistBaseGeometry( differential_drive_core::BaseGeometry_T geometry ) const
{
  double wheel_diameter;
  double stasis_diameter;

  wheel_diameter = geometry.wheel_radius * 2.0;
  stasis_diameter = geometry.stasis_radius * 2.0;

  ros::param::set( "~drive_wheel_diameter", wheel_diameter );
  ros::param::set( "~drive_wheel_base", geometry.wheel_base );
  ros::param::set( "~drive_wheel_encoder_ticks", geometry.wheel_ticks);

  if ( (geometry.wheel_ratio != 1.0) || (ros::param::has( "~drive_wheel_ratio" )) )
  {
    ros::param::set( "~drive_wheel_ratio", geometry.wheel_ratio );
  }

  ros::param::set( "~stasis_wheel_diameter", stasis_diameter );
  ros::param::set( "~stasis_wheel_encoder_ticks", geometry.stasis_ticks );

  if ( (geometry.stasis_ticks > 0) && (stasis_diameter > 0.0) )
  {
    ros::param::set( "~stasis_wheel_enabled", true );
  }
  else
  {
    ros::param::set( "~stasis_wheel_enabled", false );
  }
}
}