// IBaseModelRepository.hpp
 
#ifndef GUARD_IBaseModelRepository
#define GUARD_IBaseModelRepository
 
#include "BaseModel.hpp"
 
namespace differential_drive_core
{
class IBaseModelRepository
{
public:
  IBaseModelRepository() {}
  IBaseModelRepository( BaseModel *p_new_model ) {}
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IBaseModelRepository() {}

  virtual void QueryBaseGeometry() = 0;
  virtual void PersistBaseGeometry() = 0;
  virtual void StartListeningForUpdates() = 0;
  virtual void StopListeningForUpdates() = 0;
  virtual void SetBaseModel( BaseModel *p_new_model ) = 0;
};
}
 
#endif /* GUARD_IBaseModelRepository */
