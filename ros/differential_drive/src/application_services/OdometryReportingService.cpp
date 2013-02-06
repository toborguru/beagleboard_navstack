// OdometryReportingService.cpp
 
#include "nav_msgs/Odometry.h"
#include "IOdometryListener.hpp"
#include "OdometryReportingService.hpp"
 
using namespace differential_drive_core;
 
namespace differential_drive_application_services
{
/** Default constructor.
 *
 *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
 *  of objects. The object pointed to will be destroyed when all pointers to the object have been
 *  destroyed.
 */
OdometryReportingService::OdometryReportingService( boost::shared_ptr<IOdometryPublisherEndpoint> odometry_endpoint,
                                                    boost::shared_ptr<differential_drive_core::IMovementStatusPublisherEndpoint> movement_status_endpoint,
                                                    boost::shared_ptr<IEncoderCountsSubscriberEndpoint> encoder_counts_endpoint, 
                                                    boost::shared_ptr<const differential_drive_core::BaseModel> base_model )
  : _p_odometry_endpoint( odometry_endpoint ),
    _p_movement_status_endpoint( movement_status_endpoint ),
    _p_encoder_counts_endpoint( encoder_counts_endpoint ),
    _p_base_model( base_model )
{
  // Always listen so even if not reporting the integration occurs
  _p_encoder_counts_endpoint->Attach( _odometry_integrator );

  _odometry_integrator.SetBaseModel(*_p_base_model );
}

/** Do everything required to start all listening and reporting.
 */
void OdometryReportingService::BeginReporting() 
{
  BeginReportingOdometry();
  BeginReportingMovementStatus();
}

/** Do everything required to stop all listening and reporting.
 */
void OdometryReportingService::StopReporting()
{
  StopReportingOdometry();
  StopReportingMovementStatus();
}

/** Do everything required to start odometry reporting.
 */
void OdometryReportingService::BeginReportingOdometry() 
{
  _odometry_integrator.Attach( *_p_odometry_endpoint );
}

/** Do everything required to stop odometry reporting.
 */
void OdometryReportingService::StopReportingOdometry()
{
  _odometry_integrator.Detach( *_p_odometry_endpoint );
}

/** Do everything required to start movement status reporting.
 */
void OdometryReportingService::BeginReportingMovementStatus() 
{
  _odometry_integrator.Attach( *_p_movement_status_endpoint );
}

/** Do everything required to stop movement status reporting.
 */
void OdometryReportingService::StopReportingMovementStatus()
{
  _odometry_integrator.Detach( *_p_movement_status_endpoint );
}

/** Access function.
 */
unsigned int OdometryReportingService::GetAverageNumReadings() const
{
  return _odometry_integrator.GetAverageNumReadings();
}

/** Access function, value will be truncated to a power of 2.
 */
void OdometryReportingService::SetAverageNumReadings( const unsigned int new_average_num_readings )
{
  _odometry_integrator.SetAverageNumReadings( new_average_num_readings );
}

/** Access function.
 */
float OdometryReportingService::GetStasisPercentage() const
{
  return _odometry_integrator.GetStasisPercentage();
}

/** Access function.
 */
void OdometryReportingService::SetStasisPercentage( float percentage )
{
  _odometry_integrator.SetStasisPercentage( percentage );
}

/** Access function.
 */
float OdometryReportingService::GetVelocityLowerLimit() const
{
  return _odometry_integrator.GetVelocityLowerLimit();
}

/** Access function.
 */
void OdometryReportingService::SetVelocityLowerLimit( float velocity_limit )
{
  _odometry_integrator.SetVelocityLowerLimit( velocity_limit );
}
}
