// OdometryReportingService.hpp
 
#ifndef GUARD_OdometryReportingService
#define GUARD_OdometryReportingService
 
#include <boost/shared_ptr.hpp>
#include "IOdometryEndpoint.hpp"
#include "IEncoderCountsEndpoint.hpp"
 
namespace diff_drive_application_services
{
  class OdometryReportingService
  {
    public:
      explicit OdometryReportingService(  boost::shared_ptr<diff_drive_core::IOdometryEndpoint> odometry_endpoint,
                                          boost::shared_ptr<diff_drive_core::IEncoderCountsEndpoint> encoder_count_endpoint,
                                          boost::shared_ptr<const diff_drive_core::BaseModel> base_model );

      void BeginReporting();
      void StopReporting();
 
    private:
      // Forward declare the implementation class
      class OdometryReportingServiceImpl;
      boost::shared_ptr<OdometryReportingServiceImpl> _p_impl;
  };
}
 
#endif /* GUARD_OdometryReportingService */
