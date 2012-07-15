// FrontTelemetryReportingService.cpp
 
#include "IFrontTelemetryListener.hpp"
#include "FrontTelemetryReportingService.hpp"
 
using namespace data_robot_core;
 
namespace data_robot_application_services
{
/** Default constructor.
 *
 *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
 *  of objects. The object pointed to will be destroyed when all pointers to the object have been
 *  destroyed.
 */
FrontTelemetryReportingService::FrontTelemetryReportingService( boost::shared_ptr<IBumpersEndpoint> bumpers_endpoint,
                                                                boost::shared_ptr<IExternalBusEndpoint> external_bus_endpoint,
                                                                boost::shared_ptr<BumpersProcessor> bumpers_processor )
  : _p_bumpers_endpoint( bumpers_endpoint ),
    _p_external_bus_endpoint( external_bus_endpoint ),
    _p_bumpers_processor( bumpers_processor ),
    _is_reporting( false )
{ 
  _telemetry_reader.Attach( *this );
  _telemetry_reader.SetExternalBus( _p_external_bus_endpoint.get() );
}

/** Do everything required to start count listening and telemetry reporting.
 */
void FrontTelemetryReportingService::BeginReporting() 
{
  _telemetry_reader.BeginReading();
  _is_reporting = true;
}

/** Do everything required to stop count listening and telemetry reporting.
 */
void FrontTelemetryReportingService::StopReporting()
{
  _telemetry_reader.StopReading();
  _is_reporting = false;
}

/** This class is an telemetry listener, act on new telemetry available.
 */
void FrontTelemetryReportingService::OnFrontTelemetryAvailableEvent(const data_robot_core::FrontShellTelemetry_T& telemetry)
{
  data_robot::Bumpers bumpers;

  if ( _is_reporting )
  {
    // Report bumpers status
    _p_bumpers_processor->AddNewData( 0x0F, ~telemetry.bumpers );
    bumpers = _p_bumpers_processor->GetBumpDirection();
    _p_bumpers_endpoint->Publish( bumpers );
  }
}
};
