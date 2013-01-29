// FrontTelemetryReportingService.hpp
 
#ifndef GUARD_FrontTelemetryReportingService
#define GUARD_FrontTelemetryReportingService
 
#include <boost/shared_ptr.hpp>

#include "BumpersProcessor.hpp"
#include "FrontTelemetryReader.hpp"
#include "IBumpersPublisherEndpoint.hpp"
#include "IFrontTelemetryListener.hpp"
#include "IBusRequestProcessorEndpoint.hpp"
#include "Telemetry.hpp"
 
namespace data_robot_application_services
{
class FrontTelemetryReportingService : public data_robot_core::IFrontTelemetryListener
{
public:
  explicit FrontTelemetryReportingService(  boost::shared_ptr<data_robot_core::IBumpersPublisherEndpoint> bumpers_endpoint,
                                            boost::shared_ptr<data_robot_core::IBusRequestProcessorEndpoint> external_bus_endpoint,
                                            boost::shared_ptr<data_robot_core::BumpersProcessor> bumpers_processor );

  void BeginReporting();
  void StopReporting();

  void OnFrontTelemetryAvailableEvent(  const data_robot_core::FrontShellTelemetry_T& telemetry );

private:

  data_robot_core::FrontTelemetryReader _telemetry_reader;

  boost::shared_ptr<data_robot_core::IBumpersPublisherEndpoint>      _p_bumpers_endpoint; 
  boost::shared_ptr<data_robot_core::IBusRequestProcessorEndpoint>  _p_external_bus_endpoint;
  boost::shared_ptr<data_robot_core::BumpersProcessor>      _p_bumpers_processor;

  bool  _is_reporting;
};
}
 
#endif /* GUARD_FrontTelemetryReportingService */
