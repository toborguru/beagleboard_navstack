// BaseModelRepositoryStub.hpp
 
#ifndef GUARD_BaseModelRepositoryStub
#define GUARD_BaseModelRepositoryStub
 
#include "IBaseModelRepository.hpp"
 
namespace differential_drive_test_data_repositories_test_doubles
{
  class BaseModelRepositoryStub : public differential_drive_core::IBaseModelRepository
  { 
    public:
      BaseModelRepositoryStub()
      { }

      differential_drive_core::BaseGeometry_T QueryBaseGeometry() const
      {
        return base_geometry;
      }

      differential_drive_core::BaseGeometry_T base_geometry;
  };
}
 
#endif /* GUARD_BaseModelRepositoryStub */
