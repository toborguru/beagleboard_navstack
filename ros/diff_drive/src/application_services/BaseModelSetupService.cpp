// BaseModelSetupService.cpp
 
#include "BaseModelSetupService.hpp"
 
using namespace diff_drive_core;
 
namespace diff_drive_application_services
{
  /** Private implementation of BaseModelSetupService
   */
  class BaseModelSetupService::BaseModelSetupServiceImpl
  {
    public:
      explicit BaseModelSetupServiceImpl(  boost::shared_ptr<IBaseModelRepository> base_model_repository );

      boost::shared_ptr<BaseModel>  p_base_model;
 
      boost::shared_ptr<IBaseModelRepository> p_base_model_repository;
    private:
  };

  /** Default constructor.
   *
   *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
   *  of objects. The object pointed to will be destroyed when all pointers to the object have been
   *  destroyed.
   */
  BaseModelSetupService::BaseModelSetupService( boost::shared_ptr<IBaseModelRepository> base_model_repository )
    : _p_impl(new BaseModelSetupServiceImpl( base_model_repository ))
  {
  }
 
  /** Default Impl constructor.
   *
   *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
   *  of objects. The object pointed to will be destroyed when all pointers to the object have been
   *  destroyed.
   */
  BaseModelSetupService::BaseModelSetupServiceImpl::BaseModelSetupServiceImpl(
                                                                boost::shared_ptr<IBaseModelRepository> base_model_repository )
    : p_base_model_repository( base_model_repository )
  { 
    p_base_model = boost::shared_ptr<BaseModel>( new BaseModel() );
  }

  void BaseModelSetupService::Update()
  {
    _p_impl->p_base_model->SetBaseGeometry( _p_impl->p_base_model_repository->QueryBaseGeometry() );
  }

  boost::shared_ptr<BaseModel> BaseModelSetupService::GetBaseModel()
  {
    return _p_impl->p_base_model;
  }
}
