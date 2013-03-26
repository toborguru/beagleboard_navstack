#include <ros/ros.h>

#include "DifferentialParametersRepository.hpp"

using namespace differential_drive_core;

namespace differential_drive_data_repositories
{
DifferentialParametersRepository::DifferentialParametersRepository()
                    : _p_base_model(NULL),
                      _p_odometry_integrator(NULL),
                      _p_dynamic_reconfigure_server(NULL)
{
}

DifferentialParametersRepository::DifferentialParametersRepository( BaseModel* p_new_model,
                                                                    OdometryIntegrator* p_new_integrator )
                    : _p_dynamic_reconfigure_server(NULL)
{
  SetBaseModel( p_new_model );
  SetOdometryIntegrator( p_new_integrator);
}

DifferentialParametersRepository::~DifferentialParametersRepository()
{
  StopListeningForUpdates();
}

void DifferentialParametersRepository::StartListeningForUpdates()
{
  if ( _p_dynamic_reconfigure_server == NULL )
  {
    _p_dynamic_reconfigure_server = new dynamic_reconfigure::Server<differential_drive::DifferentialParametersConfig>;
    dynamic_reconfigure::Server<differential_drive::DifferentialParametersConfig>::CallbackType call_back_type;

    call_back_type = boost::bind( &DifferentialParametersRepository::UpdateParametersCallBack, this, _1, _2 );

    _p_dynamic_reconfigure_server->setCallback( call_back_type );
  }
}

void DifferentialParametersRepository::StopListeningForUpdates()
{
  if ( _p_dynamic_reconfigure_server != NULL )
  {
    delete _p_dynamic_reconfigure_server;
    _p_dynamic_reconfigure_server = NULL;
  }
}

void DifferentialParametersRepository::UpdateParametersCallBack( differential_drive::DifferentialParametersConfig &config, uint32_t level)
{
  /* Let's try this another way
  BaseGeometry_T base_geometry;
  int_fast8_t average_2n_readings;
  double velocity_percentage;
  double velocity_limit;

  base_geometry.wheel_radius  = config.drive_wheel_diameter / 2.0;
  base_geometry.wheel_base    = config.drive_wheel_base;
  base_geometry.wheel_ratio   = config.drive_wheel_ratio;
  base_geometry.wheel_ticks   = config.drive_wheel_encoder_ticks;

  base_geometry.stasis_radius = config.stasis_wheel_diameter / 2.0;
  base_geometry.stasis_ticks  = config.stasis_wheel_encoder_ticks;

  average_2n_readings = config.average_2n_readings;  
  velocity_percentage = config.velocity_difference_percentage;
  velocity_limit      = config.velocity_lower_limit;

  if ( _p_base_model != NULL )
  {  
    _p_base_model->SetBaseGeometry( base_geometry );
  }

  if ( _p_odometry_integrator != NULL )
  {
    _p_odometry_integrator->SetAverage2nReadings( average_2n_readings );
    _p_odometry_integrator->SetVelocityMatchPercentage( velocity_percentage );
    _p_odometry_integrator->SetVelocityLowerLimit( velocity_limit );
  }
  */

  QueryBaseParameters();
  QueryOdometryParameters();

  PersistBaseParameters();
  PersistOdometryParameters();
}

void DifferentialParametersRepository::SetBaseModel( BaseModel* p_new_model )
{
  _p_base_model = p_new_model;
}

void DifferentialParametersRepository::QueryBaseParameters()
{
  if ( _p_base_model != NULL )
  {
    _p_base_model->SetBaseGeometry( RosQueryBaseParameters() );
  }
}

void DifferentialParametersRepository::PersistBaseParameters()
{
  if ( _p_base_model != NULL )
  {
    RosPersistBaseParameters( _p_base_model->GetBaseGeometry() );
  }
}

void DifferentialParametersRepository::SetOdometryIntegrator( OdometryIntegrator* p_new_integrator )
{
  _p_odometry_integrator = p_new_integrator;
}

void DifferentialParametersRepository::QueryOdometryParameters()
{
  if ( _p_odometry_integrator != NULL )
  {
    RosQueryOdometryParameters( _p_odometry_integrator );
  }
}

void DifferentialParametersRepository::PersistOdometryParameters()
{
  if ( _p_odometry_integrator != NULL )
  {
    RosPersistOdometryParameters( _p_odometry_integrator );
  }
}

/** Returns a @c BaseGeometry_T from ROS parameters or assigns defaults.
 */
BaseGeometry_T DifferentialParametersRepository::RosQueryBaseParameters() const
{
  BaseGeometry_T base_geometry;

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

void DifferentialParametersRepository::RosPersistBaseParameters( BaseGeometry_T geometry ) const
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

void DifferentialParametersRepository::RosQueryOdometryParameters( OdometryIntegrator* p_odometry_integrator )
{
  int average_2n_readings;
  double velocity_percentage;
  double velocity_limit;

  ros::param::param<int>( "~average_2n_readings", average_2n_readings, 3 );
  ros::param::param<double>( "~velocity_difference_percentage", velocity_percentage, 10.0 );
  ros::param::param<double>( "~velocity_lower_limit", velocity_limit, 0.10 );

  p_odometry_integrator->SetAverage2nReadings( average_2n_readings );
  p_odometry_integrator->SetVelocityMatchPercentage( velocity_percentage );
  p_odometry_integrator->SetVelocityLowerLimit( velocity_limit );
}

void DifferentialParametersRepository::RosPersistOdometryParameters( const OdometryIntegrator* p_odometry_integrator ) const
{
  ros::param::set( "~average_2n_readings", (int)p_odometry_integrator->GetAverage2nReadings() );
  ros::param::set( "~average_number_of_readings", (int)p_odometry_integrator->GetAverageNumReadings() );
  ros::param::set( "~velocity_difference_percentage", p_odometry_integrator->GetVelocityMatchPercentage() );
  ros::param::set( "~velocity_lower_limit", p_odometry_integrator->GetVelocityLowerLimit() );
}
}
