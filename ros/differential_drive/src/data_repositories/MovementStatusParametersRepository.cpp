#include <ros/ros.h>

#include "MovementStatusParametersRepository.hpp"
 
namespace differential_drive_data_repositories
{
MovementStatusParametersRepository::MovementStatusParametersRepository()
                    : _p_odometry_integrator(NULL),
                      _p_parameter_reconfigure_server(NULL)
{
}

MovementStatusParametersRepository::MovementStatusParametersRepository( differential_drive_core::OdometryIntegrator* p_new_integrator )
                    : _p_parameter_reconfigure_server(NULL)
{
  SetOdometryIntegrator( p_new_integrator );
}

MovementStatusParametersRepository::~MovementStatusParametersRepository()
{
  StopListeningForUpdates();
}

void MovementStatusParametersRepository::QueryParameters()
{
  double percentage;
  double velocity_limit;
  int num_readings;

  ros::param::param<double>( "~percent_velocity_mismatch_allowed", percentage, 10.0 );   
  ros::param::param<double>( "~stasis_lower_velocity_limit", velocity_limit, 0.001 );
  ros::param::param<int>( "~number_of_readings_to_average", num_readings, 8 );

  _p_odometry_integrator->SetStasisPercentage( percentage );
  _p_odometry_integrator->SetVelocityLowerLimit( velocity_limit );
  _p_odometry_integrator->SetAverageNumReadings( num_readings );
}

void MovementStatusParametersRepository::PersistParameters()
{
  double percentage;
  double velocity_limit;
  int num_readings;

  percentage = _p_odometry_integrator->GetStasisPercentage();
  velocity_limit = _p_odometry_integrator->GetVelocityLowerLimit();
  num_readings = _p_odometry_integrator->GetAverageNumReadings();

  ros::param::set( "~percent_velocity_mismatch_allowed", percentage );   
  ros::param::set( "~stasis_lower_velocity_limit", velocity_limit );
  ros::param::set( "~number_of_readings_to_average", num_readings );
}

void MovementStatusParametersRepository::StartListeningForUpdates()
{
  if ( _p_parameter_reconfigure_server == NULL )
  {
    _p_parameter_reconfigure_server = new dynamic_reconfigure::Server<differential_drive::MovementStatusParametersConfig>;
    dynamic_reconfigure::Server<differential_drive::MovementStatusParametersConfig>::CallbackType call_back_type;

    call_back_type = boost::bind( &MovementStatusParametersRepository::UpdateParametersCallBack, this, _1, _2 );

    _p_parameter_reconfigure_server->setCallback( call_back_type );
  }
}

void MovementStatusParametersRepository::StopListeningForUpdates()
{
  if ( _p_parameter_reconfigure_server != NULL )
  {
    delete _p_parameter_reconfigure_server;
    _p_parameter_reconfigure_server = NULL;
  }
}

void MovementStatusParametersRepository::UpdateParametersCallBack( differential_drive::MovementStatusParametersConfig &config, uint32_t level)
{
  _p_odometry_integrator->SetAverageNumReadings( config.number_of_readings_to_average );
  _p_odometry_integrator->SetStasisPercentage( config.percent_velocity_mismatch_allowed );
  _p_odometry_integrator->SetVelocityLowerLimit( config.stasis_lower_velocity_limit );

  PersistParameters();
}

void MovementStatusParametersRepository::SetOdometryIntegrator( differential_drive_core::OdometryIntegrator* p_new_integrator )
{
  _p_odometry_integrator = p_new_integrator;
}
}
