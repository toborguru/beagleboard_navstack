// OdometryIntegrator.hpp
 
#ifndef GUARD_OdometryIntegrator
#define GUARD_OdometryIntegrator
 
#include <vector>
#include <queue>
#include <pthread.h>
#include <semaphore.h>

#include "nav_msgs/Odometry.h"
#include "differential_drive/EncoderCounts.h"
#include "differential_drive/MovementStatus.h"

#include "BaseModel.hpp"
#include "IMovementStatusListener.hpp"
#include "IOdometryListener.hpp"
#include "IEncoderCountsListener.hpp"

namespace differential_drive_core
{

#define MAX_2N_AVERAGES 16

class OdometryIntegrator : public IEncoderCountsListener
{
public:
  OdometryIntegrator();
  ~OdometryIntegrator();

  void Attach( IOdometryListener& odometry_listener );
  void Detach( IOdometryListener& odometry_listener );

  void Attach( IMovementStatusListener& movement_status_listener );
  void Detach( IMovementStatusListener& movement_status_listener );

  void SetBaseModel( BaseModel const & base_model );

  void OnEncoderCountsAvailableEvent( differential_drive::EncoderCounts const & encoder_counts );

  unsigned int GetAverage2nReadings() const;
  bool SetAverage2nReadings( int average_2n_readings );

  unsigned int GetAverageNumReadings() const;
  bool SetAverageNumReadings( int average_num_readings );

  double GetVelocityMatchPercentage() const;
  bool SetVelocityMatchPercentage( double percentage );

  double GetVelocityLowerLimit() const;
  bool SetVelocityLowerLimit( double velocity_limit );

private:
  void AddNewCounts( const differential_drive::EncoderCounts& encoder_counts );

  nav_msgs::Odometry CalculatePosition( BaseVelocities_T*  p_velocities, 
                                        differential_drive::EncoderCounts const & counts,
                                        nav_msgs::Odometry const & last_position,
                                        BaseModel const & base_model ) const;

  void CalculateCovariance( nav_msgs::Odometry *p_current_position,
                            differential_drive::MovementStatus const & movement_status ) const;

  differential_drive::MovementStatus CalculateMovementStatus( BaseVelocities_T const & velocities,
                                                              BaseModel const & base_model );

  void NotifyOdometryListeners( nav_msgs::Odometry const & odometry) const;
  std::vector<IOdometryListener*> _odometry_listeners;

  void NotifyMovementStatusListeners( differential_drive::MovementStatus const & movement_status) const;
  std::vector<IMovementStatusListener*> _movement_status_listeners;

  std::queue<const differential_drive::EncoderCounts*> _encoder_counts_messages;

  nav_msgs::Odometry _current_position;

  differential_drive::MovementStatus _movement_status;

  BaseVelocities_T  _velocities;

  double  _velocity_allowance;
  double  _velocity_lower_limit;

  uint_fast16_t _average_index;
  uint_fast16_t _num_readings_read;
  uint_fast16_t _average_2n_readings;
  uint_fast16_t _average_num_readings;
  uint_fast16_t _average_index_mask;

  double  _linear_average_total;
  double  _stasis_average_total;
  double* _p_linear_velocities; 
  double* _p_stasis_velocities; 

  const BaseModel* _p_base_model;

  // Basic threading support as suggested by Jeremy Friesner at
  // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
  void ProcessOdometry();
  void StartProcessingOdometry();
  void StopProcessingOdometry();

  volatile bool _stop_requested;
  volatile bool _is_running;
  pthread_t     _thread;

  pthread_mutex_t*  _p_data_mutex;
  sem_t*            _p_message_sem;            

  static void * ProcessOdometryFunction(void * This) {
    ( (OdometryIntegrator*)This )->ProcessOdometry();
    return 0;
  }
};
}

#endif /* GUARD_OdometryIntegrator */
