// OdometryReportingService.cpp
 
#include "nav_msgs/Odometry.h"
#include "IOdometryListener.hpp"
#include "OdometryReportingService.hpp"
 
using namespace diff_drive_core;
 
namespace diff_drive_application_services
{
/** Default constructor.
 *
 *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
 *  of objects. The object pointed to will be destroyed when all pointers to the object have been
 *  destroyed.
 */
OdometryReportingService::OdometryReportingService( boost::shared_ptr<IOdometryPublisherEndpoint> odometry_endpoint,
                                                    boost::shared_ptr<diff_drive_core::IMovementStatusPublisherEndpoint> movement_status_endpoint,
                                                    boost::shared_ptr<IEncoderCountsSubscriberEndpoint> encoder_counts_endpoint, 
                                                    boost::shared_ptr<const diff_drive_core::BaseModel> base_model )
  : _p_odometry_endpoint( odometry_endpoint ),
    _p_movement_status_endpoint( movement_status_endpoint ),
    _p_encoder_counts_endpoint( encoder_counts_endpoint ),
    _p_base_model( base_model ),
    _is_reporting_odometry( false ),
    _is_reporting_movement_status( false )
{
  _p_encoder_counts_endpoint->Attach( _odometry_integrator );

  _odometry_integrator.Attach( * dynamic_cast<IOdometryListener*>(this));

  _odometry_integrator.Attach( * dynamic_cast<IMovementStatusListener*>(this));

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
  _is_reporting_odometry = true;

  _p_encoder_counts_endpoint->Subscribe();
}

/** Do everything required to stop odometry reporting.
 */
void OdometryReportingService::StopReportingOdometry()
{
  // TODO Replace this with a counting semaphore
  // Check if everybody is done
  if ( ! _is_reporting_movement_status )
  {
    _p_encoder_counts_endpoint->Unsubscribe();
  }

  _is_reporting_odometry = false;
}

/** Do everything required to start movement status reporting.
 */
void OdometryReportingService::BeginReportingMovementStatus() 
{
  _is_reporting_movement_status = true;

  _p_encoder_counts_endpoint->Subscribe();
}

/** Do everything required to stop movement status reporting.
 */
void OdometryReportingService::StopReportingMovementStatus()
{
  // TODO Replace this with a counting semaphore
  // Check if everybody is done
  if ( ! _is_reporting_odometry )
  {
    _p_encoder_counts_endpoint->Unsubscribe();
  }

  _is_reporting_movement_status = false;
}

/** This class is an odometry listener, act on new odometry available.
 */
void OdometryReportingService::OnOdometryAvailableEvent(const nav_msgs::Odometry& odometry)
{
  if ( _is_reporting_odometry )
  {
    // Send odometry to the message end point
    _p_odometry_endpoint->Publish( odometry );
  }
}

/** This class is an movement status listener, and publishes any new messages available.
 */
void OdometryReportingService::OnMovementStatusAvailableEvent(const diff_drive::MovementStatus& movement_status)
{
  if ( _is_reporting_movement_status )
  {
    // Send movement_status to the message end point
    _p_movement_status_endpoint->Publish( movement_status );
  }
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
