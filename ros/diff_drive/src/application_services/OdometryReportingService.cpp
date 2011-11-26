// OdometryReportingService.cpp
 
#include "nav_msgs/Odometry.h"
#include "IOdometryListener.hpp"
#include "OdometryIntegrator.hpp"
#include "OdometryReportingService.hpp"
 
using namespace diff_drive_core;
 
namespace diff_drive_application_services
{
  /** Private implementation of OdometryReportingService
   */
  class OdometryReportingService::OdometryReportingServiceImpl : public diff_drive_core::IOdometryListener
  {
    public:
      explicit OdometryReportingServiceImpl(  boost::shared_ptr<IOdometryEndpoint> odometry_endpoint,
                                              boost::shared_ptr<IEncoderCountsEndpoint> encoder_counts_endpoint,
                                              boost::shared_ptr<const diff_drive_core::BaseModel> base_model );

      void OnOdometryAvailableEvent(  const nav_msgs::Odometry& odometry );

      OdometryIntegrator odometry_integrator;
 
      boost::shared_ptr<IOdometryEndpoint>                p_odometry_endpoint;
      boost::shared_ptr<IEncoderCountsEndpoint>           p_encoder_counts_endpoint; 
      boost::shared_ptr<const diff_drive_core::BaseModel> p_base_model;

      bool  is_reporting;
    private:
  };

  /** Default constructor.
   *
   *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
   *  of objects. The object pointed to will be destroyed when all pointers to the object have been
   *  destroyed.
   */
  OdometryReportingService::OdometryReportingService( boost::shared_ptr<IOdometryEndpoint> odometry_endpoint,
                                                      boost::shared_ptr<IEncoderCountsEndpoint> encoder_counts_endpoint, 
                                                      boost::shared_ptr<const diff_drive_core::BaseModel> base_model )
    : _p_impl(new OdometryReportingServiceImpl( odometry_endpoint, encoder_counts_endpoint, base_model )) 
  {
  }
 
  /** Default Impl constructor.
   *
   *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
   *  of objects. The object pointed to will be destroyed when all pointers to the object have been
   *  destroyed.
   */
  OdometryReportingService::OdometryReportingServiceImpl::OdometryReportingServiceImpl(
                                                                boost::shared_ptr<IOdometryEndpoint> odometry_endpoint,
                                                                boost::shared_ptr<IEncoderCountsEndpoint> encoder_counts_endpoint,
                                                                boost::shared_ptr<const diff_drive_core::BaseModel> base_model )
    : p_odometry_endpoint( odometry_endpoint ),
      p_encoder_counts_endpoint( encoder_counts_endpoint ),
      p_base_model( base_model ),
      is_reporting( false )
  { 
    p_encoder_counts_endpoint->Attach( odometry_integrator );

    odometry_integrator.Attach( *this);
    odometry_integrator.SetBaseModel(*p_base_model );
  }

  /** Do everything required to start count listening and odometry reporting.
   */
  void OdometryReportingService::BeginReporting() 
  {
    _p_impl->is_reporting = true;

    _p_impl->p_encoder_counts_endpoint->Subscribe();
  }
 
  /** Do everything required to stop count listening and odometry reporting.
   */
  void OdometryReportingService::StopReporting()
  {
    _p_impl->p_encoder_counts_endpoint->Unsubscribe();
  
    _p_impl->is_reporting = false;
  }

  /** This class is an odometry listener, act on new odometry available.
   */
  void OdometryReportingService::OdometryReportingServiceImpl::OnOdometryAvailableEvent(const nav_msgs::Odometry& odometry)
  {
    if ( is_reporting )
    {
      // Send odometry to the message end point
      p_odometry_endpoint->Publish( odometry );
    }
  };
}
