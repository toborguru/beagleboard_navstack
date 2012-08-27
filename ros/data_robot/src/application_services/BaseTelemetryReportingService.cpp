// BaseTelemetryReportingService.cpp

#include "ros/ros.h"
 
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
                                                              boost::shared_ptr<IPowerStateEndpoint> power_state_endpoint,
                                                              boost::shared_ptr<IExternalBusEndpoint> external_bus_endpoint,
                                                              boost::shared_ptr<EncoderCountsProcessor> encoder_counts_processor,
                                                              boost::shared_ptr<PowerStateProcessor> power_state_processor )
  : _p_encoder_counts_endpoint( encoder_counts_endpoint ),
    _p_power_state_endpoint( power_state_endpoint ),
    _p_external_bus_endpoint( external_bus_endpoint ),
    _p_encoder_counts_processor( encoder_counts_processor ),
    _p_power_state_processor( power_state_processor ),
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
  data_robot::PowerState    power_state;

  if ( _is_reporting )
  { 
    // Encoder Counts
    _p_encoder_counts_processor->AddNewData(  telemetry.left_encoder, 
                                              telemetry.right_encoder, 
                                              telemetry.stasis_encoder,
                                              telemetry.encoder_time);

    encoder_counts = _p_encoder_counts_processor->GetEncoderCounts();

    //encoder_counts.stasis_count = telemetry.stasis_encoder;
    encoder_counts.reading_time.sec = telemetry.seconds;
    encoder_counts.reading_time.nsec = telemetry.nano_seconds;

    _p_encoder_counts_endpoint->Publish( encoder_counts );

    // Power State
    _p_power_state_processor->AddNewData( telemetry.current, telemetry.voltage, 0 );
    power_state = _p_power_state_processor->GetPowerState();

    _p_power_state_endpoint->Publish( power_state );
  }
};
}
