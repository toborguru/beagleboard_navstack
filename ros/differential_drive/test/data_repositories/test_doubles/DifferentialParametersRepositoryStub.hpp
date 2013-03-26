// DifferentialParametersRepositoryStub.hpp
 
#ifndef GUARD_DifferentialParametersRepositoryStub
#define GUARD_DifferentialParametersRepositoryStub
 
#include "IDifferentialParametersRepository.hpp"
 
namespace differential_drive_test_data_repositories_test_doubles
{
class DifferentialParametersRepositoryStub : public differential_drive_core::IDifferentialParametersRepository
{ 
public:
  DifferentialParametersRepositoryStub()
  { }

  void StartListeningForUpdates() {}
  void StopListeningForUpdates() {}  

  void SetBaseModel( differential_drive_core::BaseModel *p_new_model ) 
  {
    _p_base_model = p_new_model;
  }

  void QueryBaseParameters() 
  {
  }

  void PersistBaseParameters()
  {
  }

  void SetOdometryIntegrator( differential_drive_core::OdometryIntegrator* p_odometry_integrator ) 
  {
    _p_odometry_integrator = p_odometry_integrator;
  }

  void QueryOdometryParameters() 
  {
  }

  void PersistOdometryParameters()
  {
  }

  differential_drive_core::BaseModel* _p_base_model;
  differential_drive_core::OdometryIntegrator* _p_odometry_integrator;
};
}

#endif /* GUARD_DifferentialParametersRepositoryStub */
