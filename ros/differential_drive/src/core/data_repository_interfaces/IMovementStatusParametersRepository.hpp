// IMovementStatusParametersRepository.hpp
 
#ifndef GUARD_IMovementStatusParametersRepository
#define GUARD_IMovementStatusParametersRepository
 
#include "OdometryIntegrator.hpp"
 
namespace differential_drive_core
{
class IMovementStatusParametersRepository
{
public:
  IMovementStatusParametersRepository() {}
  IMovementStatusParametersRepository( OdometryIntegrator* odometry_integrator ) {}
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IMovementStatusParametersRepository() {}

  virtual void QueryParameters() = 0;
  virtual void PersistParameters() = 0;
  virtual void StartListeningForUpdates() = 0;
  virtual void StopListeningForUpdates() = 0;
  virtual void SetOdometryIntegrator( OdometryIntegrator* odometry_integrator ) = 0;
};
}
 
#endif /* GUARD_IMovementStatusParametersRepository */
