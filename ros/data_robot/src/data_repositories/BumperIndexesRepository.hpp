// BumperIndexesRepository.hpp
 
#ifndef GUARD_BumperIndexesRepository
#define GUARD_BumperIndexesRepository

#include "IBumperIndexesRepository.hpp"
 
namespace data_robot_data_repositories
{
class BumperIndexesRepository : public data_robot_core::IBumperIndexesRepository
{ 
public:
  int8_t  QueryFrontBumperIndex() const;
  int8_t  QueryFrontLeftBumperIndex() const;
  int8_t  QueryFrontRightBumperIndex() const;
  int8_t  QueryRearBumperIndex() const;
  int8_t  QueryRearLeftBumperIndex() const;
  int8_t  QueryRearRightBumperIndex() const;

private:
};
}
 
#endif /* GUARD_BumperIndexesRepository */
