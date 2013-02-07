// BaseModelSetupService.hpp
 
#ifndef GUARD_BaseModelSetupService
#define GUARD_BaseModelSetupService
 
#include <boost/shared_ptr.hpp>
#include "IBaseModelRepository.hpp"
 
namespace differential_drive_application_services
{
class BaseModelSetupService
{
public:
  explicit BaseModelSetupService( boost::shared_ptr<differential_drive_core::IBaseModelRepository> base_model_repository,
                                  boost::shared_ptr<differential_drive_core::BaseModel> base_model );

  void StartUpdating();
  void StopUpdating();

  void Update();

private:
  boost::shared_ptr<differential_drive_core::IBaseModelRepository> _p_base_model_repository;
};
}
 
#endif /* GUARD_BaseModelSetupService */
