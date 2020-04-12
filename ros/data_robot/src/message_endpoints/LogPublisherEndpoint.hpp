// LogPublisherEndpoint.hpp
 
#ifndef GUARD_LogPublisherEndpoint
#define GUARD_LogPublisherEndpoint
 
#include "ILogPublisherEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
class LogPublisherEndpoint : public data_robot_core::ILogPublisherEndpoint
{ 
  public:
    LogPublisherEndpoint();

    virtual ~LogPublisherEndpoint() {};

    virtual void Publish( data_robot_core::LogLevel_T level, data_robot_core::LogFlags_T flags, const std::string& log_message );

  private:
};
}
 
#endif /* GUARD_LogPublisherEndpoint */
