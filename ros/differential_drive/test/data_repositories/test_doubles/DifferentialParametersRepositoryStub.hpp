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

  //differential_drive_core::BaseGeometry_T QueryBaseGeometry() const
  void QueryBaseGeometry() 
  {
    //return base_geometry;
  }

  //void PersistBaseGeometry( differential_drive_core::BaseGeometry_T new_geometry )
  void PersistBaseGeometry()
  {
    //base_geometry = new_geometry;
    //
  }

  void StartListeningForUpdates() {}
  void StopListeningForUpdates() {}  

  void SetBaseModel( differential_drive_core::BaseModel *p_new_model ) {}
  void SetOdometryIntegrator( differential_drive_core::OdometryIntegrator* odometry_integrator ) {}

  differential_drive_core::BaseGeometry_T base_geometry;
};
}

#endif /* GUARD_DifferentialParametersRepositoryStub */
