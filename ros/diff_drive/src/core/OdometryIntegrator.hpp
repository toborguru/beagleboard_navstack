// OdometryIntegrator.hpp
 
#ifndef GUARD_OdometryIntegrator
#define GUARD_OdometryIntegrator
 
#include <vector>

#include "nav_msgs/Odometry.h"
#include "diff_drive/EncoderCounts.h"
#include "diff_drive/MovementStatus.h"

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

    void Attach( IOdometryListener& odometry_listener );

    void Attach( IMovementStatusListener& movement_status_listener );

    void SetBaseModel( const BaseModel& base_model );

    void OnEncoderCountsAvailableEvent( const diff_drive::EncoderCounts& encoder_counts );

    unsigned int GetAverage2nReadings() const;
    void SetAverage2nReadings( unsigned int average_2n_readings );

    unsigned int GetAverageNumReadings() const;
    void SetAverageNumReadings( unsigned int average_num_readings );

    float GetStasisPercentage() const;
    void SetStasisPercentage( float percentage );

    float GetVelocityLowerLimit() const;
    void SetVelocityLowerLimit( float velocity_limit );

  private:
    void AddNewCounts( const diff_drive::EncoderCounts& encoder_counts );

    nav_msgs::Odometry CalculatePosition( BaseVelocities_T*  p_velocities, 
                                            const diff_drive::EncoderCounts counts,
                                            const nav_msgs::Odometry last_position );

    void CalculateCovariance( nav_msgs::Odometry *p_current_position,
                                const diff_drive::MovementStatus movement_status );

    diff_drive::MovementStatus CalculateMovementStatus( const BaseVelocities_T  velocities );

    void NotifyOdometryListeners(const nav_msgs::Odometry& odometry);
    std::vector<IOdometryListener*> _odometry_listeners;

    void NotifyMovementStatusListeners(const diff_drive::MovementStatus& movement_status);
    std::vector<IMovementStatusListener*> _movement_status_listeners;

    nav_msgs::Odometry _current_position;

    diff_drive::MovementStatus _movement_status;

    BaseVelocities_T  _velocities;
  
    float _stasis_percentage;
    float _velocity_lower_limit;
  
    unsigned int  _average_index;
    unsigned int  _num_readings_read;
    unsigned int  _average_2n_readings;
    unsigned int  _average_num_readings;

    float   _linear_average_total;
    float   _stasis_average_total;
    float  *_p_linear_velocities; 
    float  *_p_stasis_velocities; 

    const BaseModel* _p_base_model;
};
}

#endif /* GUARD_OdometryIntegrator */
