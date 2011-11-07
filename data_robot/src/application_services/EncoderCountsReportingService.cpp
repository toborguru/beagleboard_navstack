// EncoderCountsReportingService.cpp
 
#include "diff_drive/EncoderCounts.h"
#include "IEncoderCountsListener.hpp"
#include "EncoderCountsReportingService.hpp"
 
using namespace data_robot_core;
 
namespace data_robot_application_services
{
/** Default constructor.
 *
 *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
 *  of objects. The object pointed to will be destroyed when all pointers to the object have been
 *  destroyed.
 */
EncoderCountsReportingService::EncoderCountsReportingService( boost::shared_ptr<IEncoderCountsEndpoint> encoder_counts_endpoint,
                                                              boost::shared_ptr<IExternalBusEndpoint> external_bus_endpoint )
  : _p_encoder_counts_endpoint( encoder_counts_endpoint ),
    _p_external_bus_endpoint( external_bus_endpoint ),
    _is_reporting( false )
{ 
  _encoder_counts_reader.Attach( *this );
  _encoder_counts_reader.SetExternalBus( _p_external_bus_endpoint.get() );
}

/** Do everything required to start count listening and encoder_counts reporting.
 */
void EncoderCountsReportingService::BeginReporting() 
{
  _encoder_counts_reader.BeginReading();
  _is_reporting = true;
}

/** Do everything required to stop count listening and encoder_counts reporting.
 */
void EncoderCountsReportingService::StopReporting()
{
  _encoder_counts_reader.StopReading();
  _is_reporting = false;
}

/** This class is an encoder_counts listener, act on new encoder_counts available.
 */
void EncoderCountsReportingService::OnEncoderCountsAvailableEvent(const diff_drive::EncoderCounts& encoder_counts)
{
  if ( _is_reporting )
  {
    // Send encoder_counts to the message end point
    _p_encoder_counts_endpoint->Publish( encoder_counts );
  }
};
}
