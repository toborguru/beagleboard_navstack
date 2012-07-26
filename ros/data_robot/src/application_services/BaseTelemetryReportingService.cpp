// BaseTelemetryReportingService.cpp
 
#include "diff_drive/EncoderCounts.h"

#include "Telemetry.hpp"
#include "IBaseTelemetryListener.hpp"
#include "BaseTelemetryReportingService.hpp"
 
using namespace data_robot_core;
 
namespace data_robot_application_services
{
/** Default constructor.
 *
 *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
 *  of objects. The object pointed to will be destroyed when all pointers to the object have been
 *  destroyed.
 */
BaseTelemetryReportingService::BaseTelemetryReportingService( boost::shared_ptr<IEncoderCountsEndpoint> encoder_counts_endpoint,
                                                              boost::shared_ptr<IExternalBusEndpoint> external_bus_endpoint )
  : _p_encoder_counts_endpoint( encoder_counts_endpoint ),
    _p_external_bus_endpoint( external_bus_endpoint ),
    _is_reporting( false )
{ 
  _telemetry_reader.Attach( *this );
  _telemetry_reader.SetExternalBus( _p_external_bus_endpoint.get() );
}

/** Do everything required to start count listening and telemetry reporting.
 */
void BaseTelemetryReportingService::BeginReporting() 
{
  _telemetry_reader.BeginReading();
  _is_reporting = true;
}

/** Do everything required to stop count listening and telemetry reporting.
 */
void BaseTelemetryReportingService::StopReporting()
{
  _telemetry_reader.StopReading();
  _is_reporting = false;
}

/** This class is an telemetry listener, act on new telemetry available.
 */
void BaseTelemetryReportingService::OnBaseTelemetryAvailableEvent(const data_robot_core::BaseTelemetry_T& telemetry)
{
  diff_drive::EncoderCounts encoder_counts;

  if ( _is_reporting )
  {
    encoder_counts.left_count = telemetry.left_encoder;
    encoder_counts.right_count = telemetry.right_encoder;
    encoder_counts.stasis_count = telemetry.stasis_encoder;
    encoder_counts.dt_ms = telemetry.encoder_time;
    encoder_counts.reading_time.sec = telemetry.seconds;
    encoder_counts.reading_time.nsec = telemetry.nano_seconds;

    // Send telemetry to the message end point
    _p_encoder_counts_endpoint->Publish( encoder_counts );
  }
};
}
