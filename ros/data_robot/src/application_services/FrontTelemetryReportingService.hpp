// FrontTelemetryReportingService.hpp
 
#ifndef GUARD_FrontTelemetryReportingService
#define GUARD_FrontTelemetryReportingService
 
#include <boost/shared_ptr.hpp>

#include "FrontTelemetryReader.hpp"
#include "IBumpersEndpoint.hpp"
#include "IFrontTelemetryEndpoint.hpp"
#include "IExternalBusEndpoint.hpp"
#include "Telemetry.hpp"
 
namespace data_robot_application_services
{
class FrontTelemetryReportingService : public data_robot_core::IFrontTelemetryListener
{
public:
  explicit FrontTelemetryReportingService(  boost::shared_ptr<data_robot_core::IBumpersEndpoint> bumpers_endpoint,
                                            boost::shared_ptr<data_robot_core::IExternalBusEndpoint> external_bus_endpoint );

  void BeginReporting();
  void StopReporting();

  void OnFrontTelemetryAvailableEvent(  const data_robot_core::FrontShellTelemetry_T& telemetry );

private:

  data_robot_core::FrontTelemetryReader _telemetry_reader;

  boost::shared_ptr<data_robot_core::IBumpersEndpoint>  _p_bumpers_endpoint; 
  boost::shared_ptr<data_robot_core::IExternalBusEndpoint>    _p_external_bus_endpoint;

  bool  _is_reporting;
};
}
 
#endif /* GUARD_FrontTelemetryReportingService */
