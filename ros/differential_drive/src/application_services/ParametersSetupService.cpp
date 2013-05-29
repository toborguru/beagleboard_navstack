// ParametersSetupService.cpp
 
#include "ParametersSetupService.hpp"
 
using namespace differential_drive_core;
 
namespace differential_drive_application_services
{
/** Default constructor.
 *
 *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
 *  of objects. The object pointed to will be destroyed when all pointers to the object have been
 *  destroyed.
 */
ParametersSetupService::ParametersSetupService( boost::shared_ptr<IDifferentialParametersRepository> p_parameters_repository,
                                                boost::shared_ptr<differential_drive_core::BaseModel> p_base_model,
                                                boost::shared_ptr<OdometryReportingService> p_odometry_service )
  : _p_parameters_repository( p_parameters_repository ),
    _p_base_model( p_base_model ),
    _p_odometry_service( p_odometry_service )
{ 
  _p_parameters_repository->setBaseModel( _p_base_model.get() );
  _p_parameters_repository->setOdometryIntegrator( _p_odometry_service->getOdometryIntegrator() );
}

/** Registers with ROS dynamic reconfigure for base parameter updates.
 */
void ParametersSetupService::StartUpdating()
{
  _p_parameters_repository->startListeningForUpdates();
}

/** Registers with ROS dynamic reconfigure for base parameter updates.
 */
void ParametersSetupService::StopUpdating()
{
  _p_parameters_repository->stopListeningForUpdates();
}

/** Request updated parameters for the internal BaseModel from the data repository.
 */
void ParametersSetupService::Update()
{
  _p_parameters_repository->queryBaseParameters();
}
}
