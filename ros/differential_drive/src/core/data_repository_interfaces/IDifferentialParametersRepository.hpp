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

  virtual void startListeningForUpdates() = 0;
  virtual void stopListeningForUpdates() = 0;

  virtual void setBaseModel( BaseModel *p_new_model ) = 0;
  virtual void queryBaseParameters() = 0;
  virtual void persistBaseParameters() = 0;

  virtual void setOdometryIntegrator( OdometryIntegrator* odometry_integrator ) = 0;
  virtual void queryOdometryParameters() = 0;
  virtual void persistOdometryParameters() = 0;
};
}
 
#endif /* GUARD_IDifferentialParametersRepository */
