// BaseModelSetupService.hpp
 
#ifndef GUARD_BaseModelSetupService
#define GUARD_BaseModelSetupService
 
#include <boost/shared_ptr.hpp>
#include "IBaseModelRepository.hpp"
 
namespace diff_drive_application_services
{
  class BaseModelSetupService
  {
    public:
      explicit BaseModelSetupService( boost::shared_ptr<diff_drive_core::IBaseModelRepository> base_model_repository );

      void Update();
  
      boost::shared_ptr<diff_drive_core::BaseModel> GetBaseModel();
 
    private:
      boost::shared_ptr<diff_drive_core::BaseModel> _p_base_model;
      boost::shared_ptr<diff_drive_core::IBaseModelRepository> _p_base_model_repository;
  };
}
 
#endif /* GUARD_BaseModelSetupService */
