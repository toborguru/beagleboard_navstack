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

  void startListeningForUpdates();
  void stopListeningForUpdates();

  void setBaseModel( differential_drive_core::BaseModel* p_new_model );
  void queryBaseParameters();
  void persistBaseParameters();

  void setOdometryIntegrator( differential_drive_core::OdometryIntegrator* p_new_integrator );
  void queryOdometryParameters();
  void persistOdometryParameters();

  void updateParametersCallBack( const differential_drive::DifferentialParametersConfig &config, uint32_t level);

private:
  struct OdometryParameters_T
  {
    int     average_2n_readings;
    double  velocity_percentage;
    double  velocity_limit;
  };

  differential_drive_core::BaseGeometry_T rosQueryBaseParameters() const;
  void rosAssignBaseParameters( differential_drive_core::BaseModel* base_model, differential_drive_core::BaseGeometry_T geometry );
  void rosPersistBaseParameters( const differential_drive_core::BaseModel& base_model ) const;

  OdometryParameters_T rosQueryOdometryParameters() const;
  void rosAssignOdometryParameters( differential_drive_core::OdometryIntegrator* odometry_integrator, const OdometryParameters_T& parameters );
  void rosPersistOdometryParameters( const differential_drive_core::OdometryIntegrator& odometry_integrator ) const;

  differential_drive_core::BaseModel* _p_base_model;
  differential_drive_core::OdometryIntegrator* _p_odometry_integrator;

  dynamic_reconfigure::Server<differential_drive::DifferentialParametersConfig>* _p_dynamic_reconfigure_server;
};
}
 
#endif /* GUARD_DifferentialParametersRepository */
