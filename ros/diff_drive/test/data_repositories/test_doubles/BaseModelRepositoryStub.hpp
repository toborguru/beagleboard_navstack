// BaseModelRepositoryStub.hpp
 
#ifndef GUARD_BaseModelRepositoryStub
#define GUARD_BaseModelRepositoryStub
 
#include "IBaseModelRepository.hpp"
 
namespace diff_drive_test_data_repositories_test_doubles
{
  class BaseModelRepositoryStub : public diff_drive_core::IBaseModelRepository
  { 
    public:
      BaseModelRepositoryStub()
      { }

      diff_drive_core::BaseGeometry_T QueryBaseGeometry() const
      {
        return base_geometry;
      }

      diff_drive_core::BaseGeometry_T base_geometry;
  };
}
 
#endif /* GUARD_BaseModelRepositoryStub */
