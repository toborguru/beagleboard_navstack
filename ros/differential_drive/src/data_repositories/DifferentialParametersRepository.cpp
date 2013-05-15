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

void DifferentialParametersRepository::UpdateParametersCallBack( const differential_drive::DifferentialParametersConfig &config, uint32_t level)
{
  BaseGeometry_T base_geometry;
  OdometryParameters_T odometry_parameters;

  base_geometry.wheel_radius  = config.drive_wheel_diameter * 0.5;
  base_geometry.wheel_base    = config.drive_wheel_base;
  base_geometry.wheel_ratio   = config.drive_wheel_ratio;
  base_geometry.wheel_ticks   = config.drive_wheel_encoder_ticks;
  base_geometry.stasis_radius = config.stasis_wheel_diameter * 0.5;
  base_geometry.stasis_ticks  = config.stasis_wheel_encoder_ticks;

  odometry_parameters.average_2n_readings = config.average_2n_readings;
  odometry_parameters.velocity_percentage = config.velocity_difference_percentage;
  odometry_parameters.velocity_limit      = config.velocity_lower_limit;

  RosAssignBaseParameters( _p_base_model, base_geometry );
  RosAssignOdometryParameters( _p_odometry_integrator, odometry_parameters );

  PersistBaseParameters();
  PersistOdometryParameters();
}

void DifferentialParametersRepository::SetBaseModel( BaseModel* p_new_model )
{
  _p_base_model = p_new_model;
}

void DifferentialParametersRepository::QueryBaseParameters()
{
  BaseGeometry_T base_geometry;

  base_geometry = RosQueryBaseParameters();

  RosAssignBaseParameters( _p_base_model, base_geometry );
}

void DifferentialParametersRepository::PersistBaseParameters()
{
  if ( _p_base_model != NULL )
  {
    RosPersistBaseParameters( *_p_base_model );
  }
}

void DifferentialParametersRepository::SetOdometryIntegrator( OdometryIntegrator* p_new_integrator )
{
  _p_odometry_integrator = p_new_integrator;
}

void DifferentialParametersRepository::QueryOdometryParameters()
{
  OdometryParameters_T parameters;

  parameters = RosQueryOdometryParameters();

  RosAssignOdometryParameters( _p_odometry_integrator, parameters );
}

void DifferentialParametersRepository::PersistOdometryParameters()
{
  if ( _p_odometry_integrator != NULL )
  {
    RosPersistOdometryParameters( *_p_odometry_integrator );
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
  ros::param::param<int>( "~drive_wheel_encoder_ticks", wheel_ticks, 1 );

  ros::param::param<double>( "~stasis_wheel_diameter", stasis_diameter, 0.0 );
  ros::param::param<int>( "~stasis_wheel_encoder_ticks", stasis_ticks, -1 );

  // Assign return values
  base_geometry.wheel_radius = wheel_diameter * 0.5;
  base_geometry.wheel_base = wheel_base;
  base_geometry.wheel_ratio = wheel_ratio;
  base_geometry.wheel_ticks = wheel_ticks;
  base_geometry.stasis_radius = stasis_diameter * 0.5;
  base_geometry.stasis_ticks = stasis_ticks;

  return base_geometry;
}

void DifferentialParametersRepository::RosAssignBaseParameters( BaseModel* p_base_model, BaseGeometry_T base_geometry )
{
  double wheel_diameter = base_geometry.wheel_radius * 2.0;
  double stasis_diameter = base_geometry.stasis_radius * 2.0;

  if ( p_base_model != NULL )
  {
    if ( wheel_diameter <= 0.0 )
    {
      ROS_ERROR_NAMED(  "DifferentialParametersRepository", "drive_wheel_diameter <= 0! : %f",
          wheel_diameter );
    }

    if ( base_geometry.wheel_base <= 0.0 )
    {
      ROS_ERROR_NAMED(  "DifferentialParametersRepository", "drive_wheel_base <= 0! : %f",
          base_geometry.wheel_base );
    }

    if ( base_geometry.wheel_ratio <= 0.0 )
    {
      ROS_ERROR_NAMED(  "DifferentialParametersRepository", "drive_wheel_ratio <= 0! : %f",
          base_geometry.wheel_ratio );
    }

    if ( base_geometry.wheel_ticks <= 0 )
    {
      ROS_ERROR_NAMED(  "DifferentialParametersRepository", "drive_wheel_encoder_ticks <= 0! : %d",
          base_geometry.wheel_ticks );
    }

    if ( stasis_diameter > 0.0 )
    {
      if ( base_geometry.stasis_ticks <= 0 )
      {
        ROS_WARN_NAMED(  "DifferentialParametersRepository", "stasis_wheel_encoder_ticks <= 0! Disabling stasis wheel." );

        ros::param::set("~stasis_wheel_enabled", false );
        base_geometry.stasis_ticks = -1;
      }
    }
    else
    {
      ROS_WARN_NAMED(  "DifferentialParametersRepository", "stasis_wheel_diameter <= 0! Disabling stasis wheel." );

      ros::param::set("~stasis_wheel_enabled", false );
      base_geometry.stasis_radius = 0.0;
    }

    p_base_model->SetBaseGeometry( base_geometry );
  }
}

void DifferentialParametersRepository::RosPersistBaseParameters( const BaseModel& base_model ) const
{
  double wheel_diameter;
  double stasis_diameter;

  BaseGeometry_T base_geometry;

  base_geometry = base_model.GetBaseGeometry();

  wheel_diameter = base_geometry.wheel_radius * 2.0;
  stasis_diameter = base_geometry.stasis_radius * 2.0;

  ros::param::set( "~drive_wheel_diameter", wheel_diameter );
  ros::param::set( "~drive_wheel_base", base_geometry.wheel_base );
  ros::param::set( "~drive_wheel_encoder_ticks", (int)base_geometry.wheel_ticks);

  if ( (base_geometry.wheel_ratio != 1.0) || (ros::param::has( "~drive_wheel_ratio" )) )
  {
    ros::param::set( "~drive_wheel_ratio", base_geometry.wheel_ratio );
  }

  ros::param::set( "~stasis_wheel_diameter", stasis_diameter );
  ros::param::set( "~stasis_wheel_encoder_ticks", base_geometry.stasis_ticks );

  if ( (base_geometry.stasis_ticks > 0) && (stasis_diameter > 0.0) )
  {
    ros::param::set( "~stasis_wheel_enabled", true );
  }
  else
  {
    ros::param::set( "~stasis_wheel_enabled", false );
  }
}

DifferentialParametersRepository::OdometryParameters_T DifferentialParametersRepository::RosQueryOdometryParameters() const
{
  OdometryParameters_T parameters;

  ros::param::param<int>( "~average_2n_readings", parameters.average_2n_readings, 3 );
  ros::param::param<double>( "~velocity_difference_percentage", parameters.velocity_percentage, 10.0 );
  ros::param::param<double>( "~velocity_lower_limit", parameters.velocity_limit, 0.10 );

  return parameters;
}

void DifferentialParametersRepository::RosAssignOdometryParameters( OdometryIntegrator* p_odometry_integrator, const OdometryParameters_T& parameters )
{
  if ( p_odometry_integrator != NULL )
  {
    if ( !p_odometry_integrator->SetAverage2nReadings(parameters.average_2n_readings) )
    {
      ROS_ERROR_NAMED(  "DifferentialParametersRepository", "Error setting ~average_2n_readings, value out of range." );
    }

    if ( !p_odometry_integrator->SetVelocityMatchPercentage(parameters.velocity_percentage) )
    {
      ROS_ERROR_NAMED(  "DifferentialParametersRepository", "Error setting ~velocity_difference_percentage, value out of range." );
    }

    if ( !p_odometry_integrator->SetVelocityLowerLimit(parameters.velocity_limit) )
    {
      ROS_ERROR_NAMED(  "DifferentialParametersRepository", "Error setting ~velocity_lower_limit, value out of range." );
    }
  }
}

void DifferentialParametersRepository::RosPersistOdometryParameters( const OdometryIntegrator& odometry_integrator ) const
{
  ros::param::set( "~average_2n_readings", (int)odometry_integrator.GetAverage2nReadings() );
  ros::param::set( "~average_num_readings", (int)odometry_integrator.GetAverageNumReadings() );
  ros::param::set( "~velocity_difference_percentage", odometry_integrator.GetVelocityMatchPercentage() );
  ros::param::set( "~velocity_lower_limit", odometry_integrator.GetVelocityLowerLimit() );
}
}
