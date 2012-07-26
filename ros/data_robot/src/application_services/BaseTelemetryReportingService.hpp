// BaseTelemetryReportingService.hpp
 
#ifndef GUARD_BaseTelemetryReportingService
#define GUARD_BaseTelemetryReportingService
 
#include <boost/shared_ptr.hpp>

#include "BaseTelemetryReader.hpp"
#include "IEncoderCountsEndpoint.hpp"
#include "IExternalBusEndpoint.hpp"
 
namespace data_robot_application_services
{
class BaseTelemetryReportingService : public data_robot_core::IBaseTelemetryListener
{
public:
  explicit BaseTelemetryReportingService( boost::shared_ptr<data_robot_core::IEncoderCountsEndpoint> encoder_counts_endpoint,
                                          boost::shared_ptr<data_robot_core::IExternalBusEndpoint> external_bus_endpoint );

  void BeginReporting();
  void StopReporting();

  void OnBaseTelemetryAvailableEvent(  const data_robot_core::BaseTelemetry_T& telemetry );

private:

  data_robot_core::BaseTelemetryReader _telemetry_reader;

  boost::shared_ptr<data_robot_core::IEncoderCountsEndpoint>  _p_encoder_counts_endpoint; 
  boost::shared_ptr<data_robot_core::IExternalBusEndpoint>    _p_external_bus_endpoint;

  bool  _is_reporting;
};
}
 
#endif /* GUARD_BaseTelemetryReportingService */
