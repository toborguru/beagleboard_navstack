// ILogPublisherEndpoint.hpp
 
#ifndef GUARD_ILogPublisherEndpoint
#define GUARD_ILogPublisherEndpoint

#include <string>

#include "Log.hpp"
 
namespace data_robot_core
{
class ILogPublisherEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ILogPublisherEndpoint() {}
  virtual void Publish( LogLevel_T level, LogFlags_T flags, const std::string& log_message ) = 0;
};
}
 
#endif /* GUARD_ILogPublisherEndpoint */
