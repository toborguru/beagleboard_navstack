// IBaseModelRepository.hpp
 
#ifndef GUARD_IBaseModelRepository
#define GUARD_IBaseModelRepository
 
#include "BaseModel.hpp"
 
namespace differential_drive_core
{
  class IBaseModelRepository
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IBaseModelRepository() {}
      virtual BaseGeometry_T QueryBaseGeometry() const = 0;
  };
}
 
#endif /* GUARD_IBaseModelRepository */
