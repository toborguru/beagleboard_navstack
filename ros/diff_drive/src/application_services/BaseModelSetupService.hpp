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
      // Forward declare the implementation class
      class BaseModelSetupServiceImpl;
      boost::shared_ptr<BaseModelSetupServiceImpl> _p_impl;
  };
}
 
#endif /* GUARD_BaseModelSetupService */
