// ParametersSetupService.hpp
 
#ifndef GUARD_ParametersSetupService
#define GUARD_ParametersSetupService
 
#include <boost/shared_ptr.hpp>
#include "IDifferentialParametersRepository.hpp"
#include "OdometryReportingService.hpp"
 
namespace differential_drive_application_services
{
class ParametersSetupService
{
public:
  explicit ParametersSetupService(  boost::shared_ptr<differential_drive_core::IDifferentialParametersRepository> p_parameters_repository,
                                    boost::shared_ptr<differential_drive_core::BaseModel> p_base_model,
                                    boost::shared_ptr<OdometryReportingService> p_odometry_service );

  void StartUpdating();
  void StopUpdating();

  void Update();

private:
  boost::shared_ptr<differential_drive_core::IDifferentialParametersRepository> _p_parameters_repository;
  boost::shared_ptr<differential_drive_core::BaseModel> _p_base_model;
  boost::shared_ptr<OdometryReportingService> _p_odometry_service;
};
}
 
#endif /* GUARD_ParametersSetupService */
