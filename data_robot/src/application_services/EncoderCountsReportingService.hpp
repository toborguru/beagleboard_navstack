// EncoderCountsReportingService.hpp
 
#ifndef GUARD_EncoderCountsReportingService
#define GUARD_EncoderCountsReportingService
 
#include <boost/shared_ptr.hpp>

#include "EncoderCountsReader.hpp"
#include "IEncoderCountsEndpoint.hpp"
#include "IExternalBusEndpoint.hpp"
 
namespace data_robot_application_services
{
class EncoderCountsReportingService : public data_robot_core::IEncoderCountsListener
{
public:
  explicit EncoderCountsReportingService( boost::shared_ptr<data_robot_core::IEncoderCountsEndpoint> encoder_counts_endpoint,
                                          boost::shared_ptr<data_robot_core::IExternalBusEndpoint> external_bus_endpoint );

  void BeginReporting();
  void StopReporting();

  void OnEncoderCountsAvailableEvent(  const diff_drive::EncoderCounts& encoder_counts );

private:

  data_robot_core::EncoderCountsReader _encoder_counts_reader;

  boost::shared_ptr<data_robot_core::IEncoderCountsEndpoint>  _p_encoder_counts_endpoint; 
  boost::shared_ptr<data_robot_core::IExternalBusEndpoint>    _p_external_bus_endpoint;

  bool  _is_reporting;
};
}
 
#endif /* GUARD_EncoderCountsReportingService */
