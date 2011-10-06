// OdometryReportingService.cpp
 
#include "nav_msgs/Odometry.h"
#include "IOdometryListener.hpp"
#include "OdometryIntegrater.hpp"
#include "OdometryReportingService.hpp"
 
using namespace diff_drive_core;
 
namespace diff_drive_application_services
{
  // Private implementation of OdometryReportingService
  class OdometryReportingService::OdometryReportingServiceImpl : public diff_drive_core::IOdometryListener
  {
    public:
      explicit OdometryReportingServiceImpl(  boost::shared_ptr<IOdometryEndpoint> odometry_endpoint,
                                              boost::shared_ptr<IEncoderCountsEndpoint> encoder_counts_endpoint,
                                              boost::shared_ptr<const diff_drive_core::BaseModel> base_model );

      void OnOdometryAvailableEvent(  const nav_msgs::Odometry& odometry );

      OdometryIntegrater odometry_integrater;
 
      boost::shared_ptr<IOdometryEndpoint>                p_odometry_endpoint;
      boost::shared_ptr<IEncoderCountsEndpoint>           p_encoder_counts_endpoint; 
      boost::shared_ptr<const diff_drive_core::BaseModel> p_base_model;
    private:
  };
 
  OdometryReportingService::OdometryReportingService( boost::shared_ptr<IOdometryEndpoint> odometry_endpoint,
                                                      boost::shared_ptr<IEncoderCountsEndpoint> encoder_counts_endpoint, 
                                                      boost::shared_ptr<const diff_drive_core::BaseModel> base_model )
    : _p_impl(new OdometryReportingServiceImpl( odometry_endpoint, encoder_counts_endpoint, base_model )) 
  {
  }
 
  OdometryReportingService::OdometryReportingServiceImpl::OdometryReportingServiceImpl(
                                                                boost::shared_ptr<IOdometryEndpoint> odometry_endpoint,
                                                                boost::shared_ptr<IEncoderCountsEndpoint> encoder_counts_endpoint,
                                                                boost::shared_ptr<const diff_drive_core::BaseModel> base_model )
    : p_odometry_endpoint( odometry_endpoint ),
      p_encoder_counts_endpoint( encoder_counts_endpoint ),
      p_base_model( base_model )
  { 
    p_encoder_counts_endpoint->Attach( odometry_integrater );

    odometry_integrater.Attach( *this);
    odometry_integrater.SetBaseModel(*p_base_model );
  }
 
  void OdometryReportingService::BeginReporting() 
  {
    _p_impl->p_encoder_counts_endpoint->Subscribe();
  }
 
  void OdometryReportingService::StopReporting()
  {
    _p_impl->p_encoder_counts_endpoint->Unsubscribe();
  }
 
  void OdometryReportingService::OdometryReportingServiceImpl::OnOdometryAvailableEvent(const nav_msgs::Odometry& odometry)
  {
    // Send odometry to the message end point
    p_odometry_endpoint->Publish( odometry );
  };
}
