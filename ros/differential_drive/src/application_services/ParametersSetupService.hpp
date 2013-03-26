// ParametersSetupService.hpp
 
#ifndef GUARD_ParametersSetupService
#define GUARD_ParametersSetupService
 
#include <boost/shared_ptr.hpp>
#include "IDifferentialParametersRepository.hpp"
 
namespace differential_drive_application_services
{
class ParametersSetupService
{
public:
  explicit ParametersSetupService(  boost::shared_ptr<differential_drive_core::IDifferentialParametersRepository> base_model_repository,
                                                boost::shared_ptr<differential_drive_core::BaseModel> base_model );
                                                

  void StartUpdating();
  void StopUpdating();

  void Update();

private:
  boost::shared_ptr<differential_drive_core::IDifferentialParametersRepository> _p_base_model_repository;
  boost::shared_ptr<differential_drive_core::BaseModel> _p_base_model;
};
}
 
#endif /* GUARD_ParametersSetupService */
