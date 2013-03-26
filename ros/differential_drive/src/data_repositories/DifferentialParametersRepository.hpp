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

  virtual void QueryBaseGeometry();
  virtual void PersistBaseGeometry();

  virtual void StartListeningForUpdates();
  virtual void StopListeningForUpdates();

  virtual void SetBaseModel( differential_drive_core::BaseModel* p_new_model );
  virtual void SetOdometryIntegrator( differential_drive_core::OdometryIntegrator* p_new_integrator );

  void UpdateParametersCallBack( differential_drive::DifferentialParametersConfig &config, uint32_t level);

private:
  differential_drive_core::BaseGeometry_T RosQueryBaseGeometry() const;
  void RosPersistBaseGeometry( const differential_drive_core::BaseGeometry_T geometry ) const;

  differential_drive_core::BaseModel* _p_base_model;
  differential_drive_core::OdometryIntegrator* _p_odometry_integrator;

  dynamic_reconfigure::Server<differential_drive::DifferentialParametersConfig>* _p_geometry_reconfigure_server;
};
}
 
#endif /* GUARD_DifferentialParametersRepository */