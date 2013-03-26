// OdometryReportingService.hpp
 
#ifndef GUARD_OdometryReportingService
#define GUARD_OdometryReportingService
 
#include <boost/shared_ptr.hpp>

#include "BaseModel.hpp"
#include "IOdometryPublisherEndpoint.hpp"
#include "IMovementStatusPublisherEndpoint.hpp"
#include "IEncoderCountsSubscriberEndpoint.hpp"
#include "OdometryIntegrator.hpp"
 
namespace differential_drive_application_services
{
class OdometryReportingService
{
public:
  explicit OdometryReportingService(  boost::shared_ptr<differential_drive_core::IOdometryPublisherEndpoint> odometry_endpoint,
                                      boost::shared_ptr<differential_drive_core::IMovementStatusPublisherEndpoint> movement_status_endpoint,
                                      boost::shared_ptr<differential_drive_core::IEncoderCountsSubscriberEndpoint> encoder_count_endpoint,
                                      boost::shared_ptr<const differential_drive_core::BaseModel> base_model );

  void StartProcessingEncoderCounts();
  void StopProcessingEncoderCounts();

  void StartReporting();
  void StopReporting();

  void StartReportingOdometry();
  void StopReportingOdometry();

  void StartReportingMovementStatus();
  void StopReportingMovementStatus();

  unsigned int GetAverage2nReadings() const;
  void SetAverage2nReadings( unsigned int average_num_readings );

  float GetVelocityMatchPercentage() const;
  void SetVelocityMatchPercentage( float percentage );

  float GetVelocityLowerLimit() const;
  void SetVelocityLowerLimit( float velocity_limit );

private:

  differential_drive_core::OdometryIntegrator  _odometry_integrator;

  boost::shared_ptr<differential_drive_core::IOdometryPublisherEndpoint>       _p_odometry_endpoint;
  boost::shared_ptr<differential_drive_core::IMovementStatusPublisherEndpoint> _p_movement_status_endpoint;
  boost::shared_ptr<differential_drive_core::IEncoderCountsSubscriberEndpoint>  _p_encoder_counts_endpoint;
  boost::shared_ptr<const differential_drive_core::BaseModel>         _p_base_model;

  bool _is_reporting_odometry;
  bool _is_reporting_movement_status;
  bool _is_processing_encoder_counts;
};
}
 
#endif /* GUARD_OdometryReportingService */
