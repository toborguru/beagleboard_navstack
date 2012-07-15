// BumpersProcessorSetupService.hpp
 
#ifndef GUARD_BumpersProcessorSetupService
#define GUARD_BumpersProcessorSetupService
 
#include <boost/shared_ptr.hpp>

#include "BumpersProcessor.hpp"
#include "IBumperIndexesRepository.hpp"
 
namespace data_robot_application_services
{
class BumpersProcessorSetupService
{
public:
  explicit BumpersProcessorSetupService(  boost::shared_ptr<data_robot_core::IBumperIndexesRepository> indexes_repository,
                                          boost::shared_ptr<data_robot_core::BumpersProcessor> processor );

  void Update();

private:

  boost::shared_ptr<data_robot_core::IBumperIndexesRepository>  _p_indexes_repository; 
  boost::shared_ptr<data_robot_core::BumpersProcessor>  _p_processor; 
};
}
 
#endif /* GUARD_BumpersProcessorSetupService */
