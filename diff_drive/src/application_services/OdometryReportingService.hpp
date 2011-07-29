// OdometryReportingService.hpp
 
#ifndef GUARD_OdometryReportingService
#define GUARD_OdometryReportingService
 
#include <boost/shared_ptr.hpp>
#include "IOdometryEndpoint.hpp"
#include "IEncoderCountEndpoint.hpp"
 
namespace diff_drive_application_services
{
  class OdometryReportingService
  {
    public:
      explicit OdometryReportingService(boost::shared_ptr<diff_drive_core::IOdometryEndpoint> odometryEndpoint,
                                        boost::shared_ptr<diff_drive_core::IEncoderCountEndpoint> encoderCountEndpoint);

      BeginReporting();
      StopReporting();
 
    private:
      // Forward declare the implementation class
      class OdometryReportingServiceImpl;
      boost::shared_ptr<OdometryReportingServiceImpl> _pImpl;
  };
}
 
#endif /* GUARD_OdometryReportingService */
