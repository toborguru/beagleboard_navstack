// OdometryReportingService.hpp
 
#ifndef GUARD_OdometryReportingService
#define GUARD_OdometryReportingService
 
#include <boost/shared_ptr.hpp>

#include "BaseModel.hpp"
#include "IOdometryEndpoint.hpp"
#include "IEncoderCountsEndpoint.hpp"
#include "OdometryIntegrator.hpp"
 
namespace diff_drive_application_services
{
class OdometryReportingService : public diff_drive_core::IOdometryListener
{
public:
  explicit OdometryReportingService(  boost::shared_ptr<diff_drive_core::IOdometryEndpoint> odometry_endpoint,
                                      boost::shared_ptr<diff_drive_core::IEncoderCountsEndpoint> encoder_count_endpoint,
                                      boost::shared_ptr<const diff_drive_core::BaseModel> base_model );

  void BeginReporting();
  void StopReporting();

  void OnOdometryAvailableEvent(  const nav_msgs::Odometry& odometry );

private:

  diff_drive_core::OdometryIntegrator  _odometry_integrator;

  boost::shared_ptr<diff_drive_core::IOdometryEndpoint>                _p_odometry_endpoint;
  boost::shared_ptr<diff_drive_core::IEncoderCountsEndpoint>           _p_encoder_counts_endpoint;
  boost::shared_ptr<const diff_drive_core::BaseModel> _p_base_model;

  bool  _is_reporting; 
};
}
 
#endif /* GUARD_OdometryReportingService */
