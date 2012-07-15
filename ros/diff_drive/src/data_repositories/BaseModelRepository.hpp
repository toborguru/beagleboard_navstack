// BaseModelRepository.hpp
 
#ifndef GUARD_BaseModelRepository
#define GUARD_BaseModelRepository
 
#include "IBaseModelRepository.hpp"
 
namespace diff_drive_data_repositories
{
class BaseModelRepository : public diff_drive_core::IBaseModelRepository
{ 
public:
  diff_drive_core::BaseGeometry_T QueryBaseGeometry() const;

private:
};
}
 
#endif /* GUARD_BaseModelRepository */
