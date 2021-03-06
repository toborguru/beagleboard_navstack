// OdometryIntegrator.hpp
 
#ifndef GUARD_OdometryIntegrator
#define GUARD_OdometryIntegrator
 
#include <vector>
#include <queue>
#include <pthread.h>
#include <semaphore.h>

#include "nav_msgs/Odometry.h"
#include "diff_drive_calibrated/EncoderCounts.h"
#include "diff_drive_calibrated/MovementStatus.h"

#include "BaseModel.hpp"
#include "IMovementStatusListener.hpp"
#include "IOdometryListener.hpp"
#include "IEncoderCountsListener.hpp"

namespace diff_drive_core
{

#define MAX_2N_AVERAGES 16

class OdometryIntegrator : public IEncoderCountsListener
{
public:
  OdometryIntegrator();
  ~OdometryIntegrator();

  void attach( IOdometryListener& odometry_listener );
  void detach( IOdometryListener& odometry_listener );

  void attach( IMovementStatusListener& movement_status_listener );
  void detach( IMovementStatusListener& movement_status_listener );

  void setBaseModel( BaseModel const & base_model );

  void onEncoderCountsAvailableEvent( diff_drive_calibrated::EncoderCounts const & encoder_counts );

  unsigned int getAverage2nReadings() const;
  bool setAverage2nReadings( int average_2n_readings );

  unsigned int getAverageNumReadings() const;
  bool setAverageNumReadings( int average_num_readings );

  double getVelocityMatchPercentage() const;
  bool setVelocityMatchPercentage( double percentage );

  double getVelocityLowerLimit() const;
  bool setVelocityLowerLimit( double velocity_limit );

private:
  void addNewCounts( const diff_drive_calibrated::EncoderCounts& encoder_counts );

  nav_msgs::Odometry calculatePosition( BaseVelocities_T*  p_velocities, 
                                        diff_drive_calibrated::EncoderCounts const & counts,
                                        nav_msgs::Odometry const & last_position,
                                        BaseModel const & base_model ) const;

  void calculateCovariance( nav_msgs::Odometry *p_current_position,
                            diff_drive_calibrated::MovementStatus const & movement_status ) const;

  diff_drive_calibrated::MovementStatus calculateMovementStatus( BaseVelocities_T const & velocities,
                                                              BaseModel const & base_model );

  void notifyOdometryListeners( nav_msgs::Odometry const & odometry) const;
  std::vector<IOdometryListener*> _odometry_listeners;

  void notifyMovementStatusListeners( diff_drive_calibrated::MovementStatus const & movement_status) const;
  std::vector<IMovementStatusListener*> _movement_status_listeners;

  std::queue<const diff_drive_calibrated::EncoderCounts*> _encoder_counts_messages;

  nav_msgs::Odometry _current_position;

  diff_drive_calibrated::MovementStatus _movement_status;

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
  void processOdometry();
  void startProcessingOdometry();
  void stopProcessingOdometry();

  volatile bool _stop_requested;
  volatile bool _is_running;
  pthread_t     _thread;

  pthread_mutex_t*  _p_data_mutex;
  sem_t*            _p_message_sem;            

  static void * processOdometryFunction(void * This) {
    ( (OdometryIntegrator*)This )->processOdometry();
    return 0;
  }
};
}

#endif /* GUARD_OdometryIntegrator */
