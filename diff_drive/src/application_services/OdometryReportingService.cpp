// OdometryReportingService.cpp
 
#include "nav_msgs/Odometry.h"
#include "IOdometryListener.hpp"
#include "OdometryReader.hpp"
#include "OdometryReportingService.hpp"
 
using namespace diff_drive_core;
 
namespace diff_drive_application_services
{
  // Private implementation of OdometryReportingService
  class OdometryReportingService::OdometryReportingServiceImpl : public diff_drive_core::IOdometryListener
  {
    public:
      explicit OdometryReportingServiceImpl(boost::shared_ptr<IOdometryEndpoint> odometryEndpoint,
                                            IEncoderCountEndpoint& encoderCountEndpoint); 
      void onOdometryAvailableEvent(const nav_msgs::Odometry& odometry) const;
 
    private:
      boost::shared_ptr<IOdometryEndpoint> _odometryEndpoint;
  };
 
  OdometryReportingService::OdometryReportingService(boost::shared_ptr<IOdometryEndpoint> odometryEndpoint,
                                                     boost::shared_ptr<IEncoderCountEndpoint> encoderCountEndpoint) 
    : _pImpl(new OdometryReportingServiceImpl( odometryEndpoint, encoderCountEndpoint )) 
  {
  }
 
  OdometryReportingService::OdometryReportingServiceImpl::OdometryReportingServiceImpl(
                                                                boost::shared_ptr<IOdometryEndpoint> odometryEndpoint,
                                                                boost::shared_ptr<IEncoderCountEndpoint> encoderCountEndpoint) 
    : _odometryEndpoint(odometryEndpoint),
      _encoderCountEndpoint(encoderCountEndpoint)
  { 
  }
 
  void OdometryReportingService::beginReporting() const {
    _encoderCountEndpoint->beginReading();
  }
 
  void OdometryReportingService::stopReporting() const {
    _encoderCountEndpoint->stopReading();
  }
 
  void OdometryReportingService::OdometryReportingServiceImpl::onOdometryAvailableEvent(const nav_msgs::Odometry& odometry) const {
    // Send odometry to the message end point
    _odometryEndpoint->publish(odometry);
  };
}
