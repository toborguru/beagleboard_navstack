// BumpersProcessorSetupService.cpp
 
#include "IFrontTelemetryListener.hpp"
#include "BumpersProcessorSetupService.hpp"
 
using namespace data_robot_core;
 
namespace data_robot_application_services
{
/** Default constructor.
 *
 *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
 *  of objects. The object pointed to will be destroyed when all pointers to the object have been
 *  destroyed.
 */
BumpersProcessorSetupService::BumpersProcessorSetupService( boost::shared_ptr<IBumperIndexesRepository> indexes_repository,
                                                            boost::shared_ptr<BumpersProcessor> processor )
  : _p_indexes_repository( indexes_repository ),
    _p_processor( processor )
{ 
}

/** Reads the repository and updates the configuration parameters.
 */
void  BumpersProcessorSetupService::Update()
{
/*
  _p_processor->SetFrontBumperMask(0); 
  _p_processor->SetFrontLeftBumperMask(0); 
  _p_processor->SetFrontRightBumperMask(0); 
  _p_processor->SetRearBumperMask(0); 
  _p_processor->SetRearLeftBumperMask(0); 
  _p_processor->SetRearRightBumperMask(0); 
*/
  _p_processor->AddFrontBumperIndex( _p_indexes_repository->QueryFrontBumperIndex() );
  _p_processor->AddFrontLeftBumperIndex( _p_indexes_repository->QueryFrontLeftBumperIndex() );
  _p_processor->AddFrontRightBumperIndex( _p_indexes_repository->QueryFrontRightBumperIndex() );

  _p_processor->AddRearBumperIndex( _p_indexes_repository->QueryRearBumperIndex() );
  _p_processor->AddRearLeftBumperIndex( _p_indexes_repository->QueryRearLeftBumperIndex() );
  _p_processor->AddRearRightBumperIndex( _p_indexes_repository->QueryRearRightBumperIndex() );
}
};
