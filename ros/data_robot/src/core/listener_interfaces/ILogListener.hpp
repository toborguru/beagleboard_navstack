// ILogListener.hpp
 
#ifndef GUARD_ILogListener
#define GUARD_ILogListener

#include <string>

#include "Log.hpp"

namespace data_robot_core
{
  class ILogListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~ILogListener() {}
      virtual void OnLogAvailableEvent( LogLevel_T level, LogFlags_T flags, const std::string& log_message ) = 0;
  };
}
 
#endif /* GUARD_ILogListener */
