// DifferentialParametersRepository.hpp
 
#ifndef GUARD_DifferentialParametersRepository
#define GUARD_DifferentialParametersRepository

#include <dynamic_reconfigure/server.h>

#include <differential_drive/DifferentialParametersConfig.h>

#include "IDifferentialParametersRepository.hpp"
 
namespace differential_drive_data_repositories
{
class DifferentialParametersRepository : public differential_drive_core::IDifferentialParametersRepository
{ 
public:
  DifferentialParametersRepository();
  DifferentialParametersRepository( differential_drive_core::BaseModel* p_new_model,
                                    differential_drive_core::OdometryIntegrator* p_new_integrator );
  ~DifferentialParametersRepository();

  virtual void StartListeningForUpdates();
  virtual void StopListeningForUpdates();

  virtual void SetBaseModel( differential_drive_core::BaseModel* p_new_model );
  virtual void QueryBaseParameters();
  virtual void PersistBaseParameters();

  virtual void SetOdometryIntegrator( differential_drive_core::OdometryIntegrator* p_new_integrator );
  virtual void QueryOdometryParameters();
  virtual void PersistOdometryParameters();

  void UpdateParametersCallBack( differential_drive::DifferentialParametersConfig &config, uint32_t level);

private:
  differential_drive_core::BaseGeometry_T RosQueryBaseParameters() const;
  void RosPersistBaseParameters( const differential_drive_core::BaseGeometry_T geometry ) const;

  void RosQueryOdometryParameters( differential_drive_core::OdometryIntegrator* p_odometry_integrator );
  void RosPersistOdometryParameters( const differential_drive_core::OdometryIntegrator* p_odometry_integrator ) const;

  differential_drive_core::BaseModel* _p_base_model;
  differential_drive_core::OdometryIntegrator* _p_odometry_integrator;

  dynamic_reconfigure::Server<differential_drive::DifferentialParametersConfig>* _p_dynamic_reconfigure_server;
};
}
 
#endif /* GUARD_DifferentialParametersRepository */
