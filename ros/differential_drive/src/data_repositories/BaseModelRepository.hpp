// BaseModelRepository.hpp
 
#ifndef GUARD_BaseModelRepository
#define GUARD_BaseModelRepository
 
#include "IBaseModelRepository.hpp"
 
namespace differential_drive_data_repositories
{
class BaseModelRepository : public differential_drive_core::IBaseModelRepository
{ 
public:
  BaseModelRepository();
  BaseModelRepository( differential_drive_core::BaseModel* p_new_model );
  ~BaseModelRepository();

  virtual void QueryBaseGeometry();
  virtual void PersistBaseGeometry();

  virtual void StartListeningForUpdates();
  virtual void StopListeningForUpdates();

  virtual void SetBaseModel( differential_drive_core::BaseModel* p_new_model );

private:
  differential_drive_core::BaseGeometry_T RosQueryBaseGeometry() const;
  void RosPersistBaseGeometry( const differential_drive_core::BaseGeometry_T geometry ) const;

  differential_drive_core::BaseModel* _p_base_model;
};
}
 
#endif /* GUARD_BaseModelRepository */
