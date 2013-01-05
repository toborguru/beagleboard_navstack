// BaseModelSetupService.cpp
 
#include "BaseModelSetupService.hpp"
 
using namespace diff_drive_core;
 
namespace diff_drive_application_services
{
  /** Default constructor.
   *
   *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
   *  of objects. The object pointed to will be destroyed when all pointers to the object have been
   *  destroyed.
   */
  BaseModelSetupService::BaseModelSetupService( boost::shared_ptr<IBaseModelRepository> base_model_repository )
    : _p_base_model_repository( base_model_repository )
  { 
    _p_base_model = boost::shared_ptr<BaseModel>( new BaseModel() );
  }

  void BaseModelSetupService::Update()
  {
    _p_base_model->SetBaseGeometry( _p_base_model_repository->QueryBaseGeometry() );
  }

  boost::shared_ptr<BaseModel> BaseModelSetupService::GetBaseModel()
  {
    return _p_base_model;
  }
}
