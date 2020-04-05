// DifferentialParametersRepository.hpp
 
#ifndef GUARD_DifferentialParametersRepository
#define GUARD_DifferentialParametersRepository

#include <dynamic_reconfigure/server.h>

#include <diff_drive_calibrated/DifferentialParametersConfig.h>

#include "IDifferentialParametersRepository.hpp"
 
namespace diff_drive_data_repositories
{
class DifferentialParametersRepository : public diff_drive_core::IDifferentialParametersRepository
{ 
public:
  DifferentialParametersRepository();
  DifferentialParametersRepository( diff_drive_core::BaseModel* p_new_model,
                                    diff_drive_core::OdometryIntegrator* p_new_integrator );
  ~DifferentialParametersRepository();

  void startListeningForUpdates();
  void stopListeningForUpdates();

  void setBaseModel( diff_drive_core::BaseModel* p_new_model );
  void queryBaseParameters();
  void persistBaseParameters();

  void setOdometryIntegrator( diff_drive_core::OdometryIntegrator* p_new_integrator );
  void queryOdometryParameters();
  void persistOdometryParameters();

  void updateParametersCallBack( const diff_drive_calibrated::DifferentialParametersConfig &config, uint32_t level);

private:
  struct OdometryParameters_T
  {
    int     average_2n_readings;
    double  velocity_percentage;
    double  velocity_limit;
  };

  diff_drive_core::BaseGeometry_T rosQueryBaseParameters() const;
  void rosAssignBaseParameters( diff_drive_core::BaseModel* base_model, diff_drive_core::BaseGeometry_T geometry );
  void rosPersistBaseParameters( const diff_drive_core::BaseModel& base_model ) const;

  OdometryParameters_T rosQueryOdometryParameters() const;
  void rosAssignOdometryParameters( diff_drive_core::OdometryIntegrator* odometry_integrator, const OdometryParameters_T& parameters );
  void rosPersistOdometryParameters( const diff_drive_core::OdometryIntegrator& odometry_integrator ) const;

  diff_drive_core::BaseModel* _p_base_model;
  diff_drive_core::OdometryIntegrator* _p_odometry_integrator;

  dynamic_reconfigure::Server<diff_drive_calibrated::DifferentialParametersConfig>* _p_dynamic_reconfigure_server;
};
}
 
#endif /* GUARD_DifferentialParametersRepository */
