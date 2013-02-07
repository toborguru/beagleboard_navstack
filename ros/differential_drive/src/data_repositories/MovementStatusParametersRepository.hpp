// MovementStatusParametersRepository.hpp
 
#ifndef GUARD_MovementStatusParametersRepository
#define GUARD_MovementStatusParametersRepository

#include <dynamic_reconfigure/server.h>

#include <differential_drive/MovementStatusParametersConfig.h>

#include "OdometryIntegrator.hpp"
#include "IMovementStatusParametersRepository.hpp"
 
namespace differential_drive_data_repositories
{
class MovementStatusParametersRepository : public differential_drive_core::IMovementStatusParametersRepository
{ 
public:
  MovementStatusParametersRepository();
  MovementStatusParametersRepository( differential_drive_core::OdometryIntegrator* p_new_integrator );
  ~MovementStatusParametersRepository();

  virtual void QueryParameters();
  virtual void PersistParameters();

  virtual void StartListeningForUpdates();
  virtual void StopListeningForUpdates();

  virtual void SetOdometryIntegrator( differential_drive_core::OdometryIntegrator* p_new_integrator );

  void UpdateParametersCallBack( differential_drive::MovementStatusParametersConfig &config, uint32_t level);

private:

  differential_drive_core::OdometryIntegrator* _p_odometry_integrator;
  dynamic_reconfigure::Server<differential_drive::MovementStatusParametersConfig>* _p_parameter_reconfigure_server;
};
}
 
#endif /* GUARD_MovementStatusParametersRepository */
