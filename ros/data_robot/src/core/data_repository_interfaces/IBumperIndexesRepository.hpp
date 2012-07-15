// IBumperIndexesRepository.hpp
 
#ifndef GUARD_IBumperIndexesRepository
#define GUARD_IBumperIndexesRepository
 
namespace data_robot_core
{
class IBumperIndexesRepository
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IBumperIndexesRepository() {}
  virtual int8_t  QueryFrontBumperIndex() const = 0;
  virtual int8_t  QueryFrontLeftBumperIndex() const = 0;
  virtual int8_t  QueryFrontRightBumperIndex() const = 0;
  virtual int8_t  QueryRearBumperIndex() const = 0;
  virtual int8_t  QueryRearLeftBumperIndex() const = 0;
  virtual int8_t  QueryRearRightBumperIndex() const = 0;
};
}
 
#endif /* GUARD_IBumperIndexesRepository */
