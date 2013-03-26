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
    _p_base_model->SetBaseGeometry( _db_base_model.GetBaseGeometry() );
  }

  void PersistBaseParameters()
  {
    _db_base_model.SetBaseGeometry( _p_base_model->GetBaseGeometry() );
  }

  void SetOdometryIntegrator( differential_drive_core::OdometryIntegrator* p_odometry_integrator ) 
  {
    _p_odometry_integrator = p_odometry_integrator;
  }

  void QueryOdometryParameters() 
  {
    _p_odometry_integrator->SetAverage2nReadings( _db_odometry_integrator.GetAverage2nReadings() );
    _p_odometry_integrator->SetVelocityMatchPercentage( _db_odometry_integrator.GetVelocityMatchPercentage() );
    _p_odometry_integrator->SetVelocityLowerLimit( _db_odometry_integrator.GetVelocityLowerLimit() );
  }

  void PersistOdometryParameters()
  {
    _db_odometry_integrator.SetAverage2nReadings( _p_odometry_integrator->GetAverage2nReadings() );
    _db_odometry_integrator.SetVelocityMatchPercentage( _p_odometry_integrator->GetVelocityMatchPercentage() );
    _db_odometry_integrator.SetVelocityLowerLimit( _p_odometry_integrator->GetVelocityLowerLimit() );
  }

  differential_drive_core::BaseModel* _p_base_model;
  differential_drive_core::BaseModel _db_base_model;

  differential_drive_core::OdometryIntegrator* _p_odometry_integrator;
  differential_drive_core::OdometryIntegrator _db_odometry_integrator;
};
}

#endif /* GUARD_DifferentialParametersRepositoryStub */
