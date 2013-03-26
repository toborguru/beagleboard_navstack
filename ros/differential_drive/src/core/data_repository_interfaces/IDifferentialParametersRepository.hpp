// IDifferentialParametersRepository.hpp
 
#ifndef GUARD_IDifferentialParametersRepository
#define GUARD_IDifferentialParametersRepository
 
#include "BaseModel.hpp"
#include "OdometryIntegrator.hpp"
 
namespace differential_drive_core
{
class IDifferentialParametersRepository
{
public:
  IDifferentialParametersRepository() {}
  IDifferentialParametersRepository( BaseModel *p_new_model, OdometryIntegrator* odometry_integrator ) {}

  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IDifferentialParametersRepository() {}

  virtual void QueryBaseGeometry() = 0;
  virtual void PersistBaseGeometry() = 0;
  virtual void StartListeningForUpdates() = 0;
  virtual void StopListeningForUpdates() = 0;
  virtual void SetBaseModel( BaseModel *p_new_model ) = 0;
  virtual void SetOdometryIntegrator( OdometryIntegrator* odometry_integrator ) = 0;
};
}
 
#endif /* GUARD_IDifferentialParametersRepository */
