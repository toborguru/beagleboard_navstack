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
    _p_base_model( base_model ),
    _is_reporting_odometry(false),
    _is_reporting_movement_status(false),
    _is_processing_encoder_counts(false)
{
  _odometry_integrator.SetBaseModel( *_p_base_model );
}

/** Do everything required to start all listening and reporting.
 */
void OdometryReportingService::StartReporting() 
{
  StartReportingOdometry();
  StartReportingMovementStatus();
  StartProcessingEncoderCounts();
}

/** Do everything required to stop all listening and reporting.
 */
void OdometryReportingService::StopReporting()
{
  StopProcessingEncoderCounts(); 
  StopReportingOdometry();
  StopReportingMovementStatus();
}

/** Start reading and processing incoming encoder counts.
 */
void OdometryReportingService::StartProcessingEncoderCounts()
{
  if ( !_is_processing_encoder_counts )
  {
    _p_encoder_counts_endpoint->Attach( _odometry_integrator );
  }

  _is_processing_encoder_counts = true;
}

/** Stop reading and processing incoming encoder counts.
 */
void OdometryReportingService::StopProcessingEncoderCounts()
{
  _p_encoder_counts_endpoint->Detach( _odometry_integrator );
  _is_processing_encoder_counts = false;
}

/** Do everything required to start odometry reporting.
 */
void OdometryReportingService::StartReportingOdometry() 
{
  if ( !_is_reporting_odometry )
  {
    _odometry_integrator.Attach( *_p_odometry_endpoint );
  }

  _is_reporting_odometry = true;
}

/** Do everything required to stop odometry reporting.
 */
void OdometryReportingService::StopReportingOdometry()
{
  _odometry_integrator.Detach( *_p_odometry_endpoint );
  _is_reporting_odometry = false;
}

/** Do everything required to start movement status reporting.
 */
void OdometryReportingService::StartReportingMovementStatus() 
{
  if ( !_is_reporting_movement_status )
  {
    _odometry_integrator.Attach( *_p_movement_status_endpoint );
  }

  _is_reporting_movement_status = true;
}

/** Do everything required to stop movement status reporting.
 */
void OdometryReportingService::StopReportingMovementStatus()
{
  _odometry_integrator.Detach( *_p_movement_status_endpoint );
  _is_reporting_movement_status = false;
}

/** Returns the number of velocity readings that are being averaged before 
 *  comparing the drive and status speeds.
 */
unsigned int OdometryReportingService::GetAverage2nReadings() const
{
  return _odometry_integrator.GetAverage2nReadings();
}

/** Set the number of velocity readings to average before comparing the drive 
 *  and status speeds.
 */
void OdometryReportingService::SetAverage2nReadings( unsigned int average_num_readings )
{
  _odometry_integrator.SetAverage2nReadings( average_num_readings );
}

/** Returns the percentage of difference in drive and status speeds before a 
 *  mismatch will be flagged.
 */
float OdometryReportingService::GetVelocityMatchPercentage() const
{
  return _odometry_integrator.GetVelocityMatchPercentage();
}

/** Sets the percentage of difference in drive and status speeds before a 
 *  mismatch will be flagged.
 */
void OdometryReportingService::SetVelocityMatchPercentage( float percentage )
{
  _odometry_integrator.SetVelocityMatchPercentage( percentage );
}

/** Returns the velocity (m/s) below which speed mismatches will be ignored.
 */
float OdometryReportingService::GetVelocityLowerLimit() const
{
  return _odometry_integrator.GetVelocityMatchPercentage();
}

/** Sets the velocity (m/s) below which speed mismatches will be ignored.
 */
void OdometryReportingService::SetVelocityLowerLimit( float velocity_limit )
{
  _odometry_integrator.SetVelocityLowerLimit( velocity_limit );
}
}
