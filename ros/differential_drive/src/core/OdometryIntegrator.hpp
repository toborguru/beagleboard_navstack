// OdometryIntegrator.hpp
 
#ifndef GUARD_OdometryIntegrator
#define GUARD_OdometryIntegrator
 
#include <vector>
#include <queue>

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

    void SetBaseModel( const BaseModel& base_model );

    void OnEncoderCountsAvailableEvent( const differential_drive::EncoderCounts& encoder_counts );

    unsigned int GetAverage2nReadings() const;
    void SetAverage2nReadings( unsigned int average_2n_readings );

    unsigned int GetAverageNumReadings() const;
    void SetAverageNumReadings( unsigned int average_num_readings );

    float GetStasisPercentage() const;
    void SetStasisPercentage( float percentage );

    float GetVelocityLowerLimit() const;
    void SetVelocityLowerLimit( float velocity_limit );

  private:
    void AddNewCounts( const differential_drive::EncoderCounts& encoder_counts );

    nav_msgs::Odometry CalculatePosition( BaseVelocities_T*  p_velocities, 
                                            const differential_drive::EncoderCounts counts,
                                            const nav_msgs::Odometry last_position );

    void CalculateCovariance( nav_msgs::Odometry *p_current_position,
                                const differential_drive::MovementStatus movement_status );

    differential_drive::MovementStatus CalculateMovementStatus( const BaseVelocities_T  velocities );

    void NotifyOdometryListeners(const nav_msgs::Odometry& odometry);
    std::vector<IOdometryListener*> _odometry_listeners;

    void NotifyMovementStatusListeners(const differential_drive::MovementStatus& movement_status);
    std::vector<IMovementStatusListener*> _movement_status_listeners;

    std::queue<const differential_drive::EncoderCounts*> _encoder_counts_messages;

    nav_msgs::Odometry _current_position;

    differential_drive::MovementStatus _movement_status;

    BaseVelocities_T  _velocities;
  
    float _stasis_percentage;
    float _velocity_lower_limit;
  
    uint_fast16_t _average_index;
    uint_fast16_t _num_readings_read;
    uint_fast16_t _average_2n_readings;
    uint_fast16_t _average_num_readings;
    uint_fast16_t _average_index_mask;

    float   _linear_average_total;
    float   _stasis_average_total;
    float  *_p_linear_velocities; 
    float  *_p_stasis_velocities; 

    const BaseModel* _p_base_model;

    // Basic threading support as suggested by Jeremy Friesner at
    // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
    void ProcessOdometry();
    void StartProcessingOdometry();
    void StopProcessingOdometry();

    volatile bool _stop_requested;
    volatile bool _is_running;
    pthread_t _thread;

    pthread_mutex_t  *_p_processing_mutex;
    pthread_mutex_t  *_p_message_mutex;
    pthread_cond_t   *_p_message_cond;

    static void * ProcessOdometryFunction(void * This) {
      ( (OdometryIntegrator*)This )->ProcessOdometry();
      return 0;
    }
};
}

#endif /* GUARD_OdometryIntegrator */
