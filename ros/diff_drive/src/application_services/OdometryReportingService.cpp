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
OdometryReportingService::OdometryReportingService( boost::shared_ptr<IOdometryEndpoint> odometry_endpoint,
                                                    boost::shared_ptr<IEncoderCountsEndpoint> encoder_counts_endpoint, 
                                                    boost::shared_ptr<const diff_drive_core::BaseModel> base_model )
  : _p_odometry_endpoint( odometry_endpoint ),
    _p_encoder_counts_endpoint( encoder_counts_endpoint ),
    _p_base_model( base_model ),
    _is_reporting( false )
{
}

/** Do everything required to start count listening and odometry reporting.
 */
void OdometryReportingService::BeginReporting() 
{
  _is_reporting = true;

  _p_encoder_counts_endpoint->Subscribe();
}

/** Do everything required to stop count listening and odometry reporting.
 */
void OdometryReportingService::StopReporting()
{
  _p_encoder_counts_endpoint->Unsubscribe();

  _is_reporting = false;
}

/** This class is an odometry listener, act on new odometry available.
 */
void OdometryReportingService::OnOdometryAvailableEvent(const nav_msgs::Odometry& odometry)
{
  if ( _is_reporting )
  {
    // Send odometry to the message end point
    _p_odometry_endpoint->Publish( odometry );
  }
};
}
