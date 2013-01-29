// OdometryReportingService.hpp
 
#ifndef GUARD_OdometryReportingService
#define GUARD_OdometryReportingService
 
#include <boost/shared_ptr.hpp>

#include "BaseModel.hpp"
#include "IOdometryPublisherEndpoint.hpp"
#include "IMovementStatusPublisherEndpoint.hpp"
#include "IEncoderCountsSubscriberEndpoint.hpp"
#include "OdometryIntegrator.hpp"
 
namespace diff_drive_application_services
{
class OdometryReportingService :  public diff_drive_core::IOdometryListener,
                                  public diff_drive_core::IMovementStatusListener
{
public:
  explicit OdometryReportingService(  boost::shared_ptr<diff_drive_core::IOdometryPublisherEndpoint> odometry_endpoint,
                                      boost::shared_ptr<diff_drive_core::IMovementStatusPublisherEndpoint> movement_status_endpoint,
                                      boost::shared_ptr<diff_drive_core::IEncoderCountsSubscriberEndpoint> encoder_count_endpoint,
                                      boost::shared_ptr<const diff_drive_core::BaseModel> base_model );

  void BeginReporting();
  void StopReporting();

  void BeginReportingOdometry();
  void StopReportingOdometry();

  void BeginReportingMovementStatus();
  void StopReportingMovementStatus();

  void OnOdometryAvailableEvent(  const nav_msgs::Odometry& odometry );
  void OnMovementStatusAvailableEvent(  const diff_drive::MovementStatus& movement_status );

  unsigned int GetAverageNumReadings() const;
  void SetAverageNumReadings( const unsigned int new_average_num_readings );

  float GetStasisPercentage() const;
  void SetStasisPercentage( float percentage );

  float GetVelocityLowerLimit() const;
  void SetVelocityLowerLimit( float velocity_limit );

private:

  diff_drive_core::OdometryIntegrator  _odometry_integrator;

  boost::shared_ptr<diff_drive_core::IOdometryPublisherEndpoint>       _p_odometry_endpoint;
  boost::shared_ptr<diff_drive_core::IMovementStatusPublisherEndpoint> _p_movement_status_endpoint;
  boost::shared_ptr<diff_drive_core::IEncoderCountsSubscriberEndpoint>  _p_encoder_counts_endpoint;
  boost::shared_ptr<const diff_drive_core::BaseModel>         _p_base_model;

  bool  _is_reporting_odometry; 
  bool  _is_reporting_movement_status; 
};
}
 
#endif /* GUARD_OdometryReportingService */
