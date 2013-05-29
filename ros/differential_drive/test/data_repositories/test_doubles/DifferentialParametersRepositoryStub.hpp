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

  void setBaseModel( differential_drive_core::BaseModel *p_new_model ) 
  {
    _p_base_model = p_new_model;
  }

  void QueryBaseParameters() 
  {
    _p_base_model->setBaseGeometry( _db_base_model.getBaseGeometry() );
  }

  void PersistBaseParameters()
  {
    _db_base_model.setBaseGeometry( _p_base_model->getBaseGeometry() );
  }

  void setOdometryIntegrator( differential_drive_core::OdometryIntegrator* p_odometry_integrator ) 
  {
    _p_odometry_integrator = p_odometry_integrator;
  }

  void QueryOdometryParameters() 
  {
    _p_odometry_integrator->setAverage2nReadings( _db_odometry_integrator.getAverage2nReadings() );
    _p_odometry_integrator->setVelocityMatchPercentage( _db_odometry_integrator.getVelocityMatchPercentage() );
    _p_odometry_integrator->setVelocityLowerLimit( _db_odometry_integrator.getVelocityLowerLimit() );
  }

  void PersistOdometryParameters()
  {
    _db_odometry_integrator.setAverage2nReadings( _p_odometry_integrator->getAverage2nReadings() );
    _db_odometry_integrator.setVelocityMatchPercentage( _p_odometry_integrator->getVelocityMatchPercentage() );
    _db_odometry_integrator.setVelocityLowerLimit( _p_odometry_integrator->getVelocityLowerLimit() );
  }

  differential_drive_core::BaseModel* _p_base_model;
  differential_drive_core::BaseModel _db_base_model;

  differential_drive_core::OdometryIntegrator* _p_odometry_integrator;
  differential_drive_core::OdometryIntegrator _db_odometry_integrator;
};
}

#endif /* GUARD_DifferentialParametersRepositoryStub */
