// ParameterSetupService.cpp
 
#include "ParameterSetupService.hpp"
 
using namespace differential_drive_core;
 
namespace differential_drive_application_services
{
/** Default constructor.
 *
 *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
 *  of objects. The object pointed to will be destroyed when all pointers to the object have been
 *  destroyed.
 */
BaseModelSetupService::BaseModelSetupService( boost::shared_ptr<IBaseModelRepository> base_model_repository,
                                              boost::shared_ptr<differential_drive_core::BaseModel> base_model )
  : _p_base_model_repository( base_model_repository )
{ 
  _p_base_model_repository->SetBaseModel( base_model.get() );
}

/** Registers with ROS dynamic reconfigure for base parameter updates.
 */
void BaseModelSetupService::StartUpdating()
{
  _p_base_model_repository->StartListeningForUpdates();
}

/** Registers with ROS dynamic reconfigure for base parameter updates.
 */
void BaseModelSetupService::StopUpdating()
{
  _p_base_model_repository->StopListeningForUpdates();
}

/** Request updated parameters for the internal BaseModel from the data repository.
 */
void BaseModelSetupService::Update()
{
  _p_base_model_repository->QueryBaseGeometry();
}
}
