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
    _is_reporting_odometry(false),
    _is_reporting_movement_status(false),
    _is_processing_encoder_counts(false)
{
  _odometry_integrator.setBaseModel( *_p_base_model );
}

/** Do everything required to start all listening and reporting.
 */
void OdometryReportingService::startReporting() 
{
  startReportingOdometry();
  startReportingMovementStatus();
  startProcessingEncoderCounts();
}

/** Do everything required to stop all listening and reporting.
 */
void OdometryReportingService::stopReporting()
{
  stopProcessingEncoderCounts(); 
  stopReportingOdometry();
  stopReportingMovementStatus();
}

/** Start reading and processing incoming encoder counts.
 */
void OdometryReportingService::startProcessingEncoderCounts()
{
  if ( !_is_processing_encoder_counts )
  {
    _p_encoder_counts_endpoint->attach( _odometry_integrator );
  }

  _is_processing_encoder_counts = true;
}

/** Stop reading and processing incoming encoder counts.
 */
void OdometryReportingService::stopProcessingEncoderCounts()
{
  _p_encoder_counts_endpoint->detach( _odometry_integrator );
  _is_processing_encoder_counts = false;
}

/** Do everything required to start odometry reporting.
 */
void OdometryReportingService::startReportingOdometry() 
{
  if ( !_is_reporting_odometry )
  {
    _odometry_integrator.attach( *_p_odometry_endpoint );
  }

  _is_reporting_odometry = true;
}

/** Do everything required to stop odometry reporting.
 */
void OdometryReportingService::stopReportingOdometry()
{
  _odometry_integrator.detach( *_p_odometry_endpoint );
  _is_reporting_odometry = false;
}

/** Do everything required to start movement status reporting.
 */
void OdometryReportingService::startReportingMovementStatus() 
{
  if ( !_is_reporting_movement_status )
  {
    _odometry_integrator.attach( *_p_movement_status_endpoint );
  }

  _is_reporting_movement_status = true;
}

/** Do everything required to stop movement status reporting.
 */
void OdometryReportingService::stopReportingMovementStatus()
{
  _odometry_integrator.detach( *_p_movement_status_endpoint );
  _is_reporting_movement_status = false;
}

/** Return a pointer to the internal OdometryIntegrator object.
 */
OdometryIntegrator* OdometryReportingService::getOdometryIntegrator()
{
  return &_odometry_integrator;
}
}
