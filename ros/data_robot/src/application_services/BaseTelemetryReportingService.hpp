// BaseTelemetryReportingService.hpp
 
#ifndef GUARD_BaseTelemetryReportingService
#define GUARD_BaseTelemetryReportingService
 
#include <boost/shared_ptr.hpp>

#include "BaseTelemetryReader.hpp"
#include "EncoderCountsProcessor.hpp"
#include "IEncoderCountsPublisherEndpoint.hpp"
#include "IBusRequestProcessorEndpoint.hpp"
#include "IPowerStatePublisherEndpoint.hpp"
#include "PowerStateProcessor.hpp"
 
namespace data_robot_application_services
{
class BaseTelemetryReportingService : public data_robot_core::IBaseTelemetryListener
{
public:
  explicit BaseTelemetryReportingService( boost::shared_ptr<data_robot_core::IEncoderCountsPublisherEndpoint> encoder_counts_endpoint,
                                          boost::shared_ptr<data_robot_core::IPowerStatePublisherEndpoint> power_state_endpoint,
                                          boost::shared_ptr<data_robot_core::IBusRequestProcessorEndpoint> external_bus_endpoint,
                                          boost::shared_ptr<data_robot_core::EncoderCountsProcessor> encoder_counts_processor,
                                          boost::shared_ptr<data_robot_core::PowerStateProcessor> power_state_processor );

  void BeginReporting();
  void StopReporting();

  void OnBaseTelemetryAvailableEvent(  const data_robot_core::BaseTelemetry_T& telemetry );

private:

  data_robot_core::BaseTelemetryReader _telemetry_reader;

  boost::shared_ptr<data_robot_core::IEncoderCountsPublisherEndpoint>  _p_encoder_counts_endpoint; 
  boost::shared_ptr<data_robot_core::IBusRequestProcessorEndpoint>    _p_external_bus_endpoint;
  boost::shared_ptr<data_robot_core::IPowerStatePublisherEndpoint>     _p_power_state_endpoint; 
  boost::shared_ptr<data_robot_core::EncoderCountsProcessor>  _p_encoder_counts_processor;
  boost::shared_ptr<data_robot_core::PowerStateProcessor>     _p_power_state_processor;

  bool  _is_reporting;
};
}
 
#endif /* GUARD_BaseTelemetryReportingService */
