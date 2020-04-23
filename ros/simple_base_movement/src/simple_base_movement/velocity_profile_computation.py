#! /usr/bin/env python

# velocity_profile_computation.py 
# 
# Software License Agreement (BSD License)                            
#                                                                     
# Copyright (c) 2014, Sawyer Larkin
# All rights reserved.                                                
#                                                                     
# Redistribution and use in source and binary forms, with or without  
# modification, are permitted provided that the following conditions  
# are met:                                                            
#                                                                     
#  * Redistributions of source code must retain the above copyright   
#    notice, this list of conditions and the following disclaimer.    
#  * Redistributions in binary form must reproduce the above          
#    copyright notice, this list of conditions and the following      
#    disclaimer in the documentation and/or other materials provided  
#    with the distribution.                                           
#  * Neither the name of Sawyer Larkin nor the names of its     
#    contributors may be used to endorse or promote products derived  
#    from this software without specific prior written permission.    
#                                                                     
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT   
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE      
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER    
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT  
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN   
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE     
# POSSIBILITY OF SUCH DAMAGE.

"""Computes a trapezoidal velocity profile to traverse a specified distance 
using acceleration constraints.

"""

# Changelog
# 2014-03-19 SJL: Created.
#

# TODO
# allow lower velocity to be set

import math

class VelocityProfileComputation(object):
    def __init__(self, aim_short_dist, max_vel, min_vel, max_accel, dt):
        """Constructor

        aim_short_dist
            Original goal distance is shortened by this amount to allow a 
            deceleration below @p accel to exactly reach the target.
        
        """

        if (max_vel == 0):
            raise ValueError('Max velocity cannot be 0.')

        if (max_accel == 0):
            raise ValueError('Max Acceleration cannot be 0.')

        if (dt == 0):
            raise ValueError('Delta time cannot be 0.')

        self._aim_short_dist = abs(aim_short_dist)
        self._max_vel = abs(max_vel)
        self._min_vel = abs(min_vel)
        self._max_accel = abs(max_accel)
        self._dt = abs(dt)


    def next_profile_step_ballistic(self, distance_left, loop_num):
        """ Needs this

        """
        finished = False
        goal_reached = False

        if (loop_num == 0):
            # compute required profile on first run
            profile_dist = abs(distance_left) - abs(self._aim_short_dist)
            # compute required profile on first run
            (self._accel_cnt, self._accel, self._coast_cnt, self._coast_vel) = \
                    self.compute_trapezoidal_profile(profile_dist,
                                                     self._max_vel, 
                                                     self._max_accel,
                                                     self._dt)

            # CYA
            self._decel_rate = self._accel
            self._decel_cnt = self._accel_cnt

            # first acceleration step
            # assumes we are stopped
            self._vel_command = self._min_vel; 

            '''
            print('Accel: %f Accel cnt: %g Coast Vel: %g Coast cnt: %g' %
                    (self._accel, self._accel_cnt, self._coast_vel, 
                    self._coast_cnt))
            # '''

        if (loop_num < self._accel_cnt):
            # acceleration leg
            self._vel_command = self.apply_acceleration(self._vel_command, 
                                                       self._coast_vel,
                                                       self._accel,
                                                       self._dt)

        elif (loop_num < (self._accel_cnt + self._coast_cnt)):
            # coast leg
            self._vel_command = self._coast_vel

        elif (loop_num < (self._accel_cnt + self._coast_cnt + self._accel_cnt)):
            # deceleration leg
            self._vel_command = self.apply_acceleration(self._vel_command, 
                                                       0,
                                                       self._accel,
                                                       self._dt)
            if (self._vel_command < self._min_vel):
                self._vel_command = self._min_vel

        else:
            self._vel_command = 0.0

            # profile finshed
            finished = True

        # assumes we are stopping
        if (self._vel_command == 0):
            goal_reached = True

        return (self._vel_command, finished, goal_reached)

    def next_profile_step_with_feedback(self, distance_left, current_vel, 
                                        measurement_secs, loop_num):
        """ Needs this

        measurement_secs
            The time elapsed since the distance measurement was taken in seconds.

        """
        finished = False
        goal_reached = False

        if (loop_num == 0):
            # compute required profile on first run

            if ( abs(self._min_vel * self._dt) > abs(self._aim_short_dist) ):
                self._aim_short_dist = self._min_vel * self._dt

            profile_dist = abs(distance_left) - abs(self._aim_short_dist)

            (self._accel_cnt, self._accel, self._coast_cnt, self._coast_vel) = \
                    self.compute_trapezoidal_profile(profile_dist,
                                                     self._max_vel, 
                                                     self._max_accel,
                                                     self._dt)

            # CYA
            self._decel_rate = self._accel
            self._decel_cnt = self._accel_cnt

            # first acceleration step
            # assumes we are stopped
            self._vel_command = self._min_vel

            '''
            print('Accel: %f Accel cnt: %g Coast Vel: %g Coast cnt: %g' %
                    (self._accel, self._accel_cnt, self._coast_vel, 
                    self._coast_cnt))
            # '''

        if (loop_num < self._accel_cnt):
            # acceleration leg
            self._vel_command = self.apply_acceleration(self._vel_command, 
                                                       self._coast_vel,
                                                       self._accel,
                                                       self._dt)

        elif (loop_num < (self._accel_cnt + self._coast_cnt)):
            # coast leg
            self._vel_command = self._coast_vel

        elif (self._decel_cnt > 0) and (self._vel_command != 0.0):
            # deceleration leg
            decel_loop_num = int(loop_num - (self._accel_cnt + self._coast_cnt))
            decel_check_cnt = int(self._accel_cnt / 2)
            
            if (decel_check_cnt != 0) :
                remainder = decel_loop_num % decel_check_cnt
            else :
                remainder = 1

            # If we are almost half-way through the calculated deceleration 
            # phase or within aim_short_dist of the goal, recalculate the 
            # required deceleration rate
            #if (remainder == (decel_check_cnt - 1)) or \
            #                    (abs(distance_left) < abs(self._aim_short_dist)):
            if (True) :
                # compute deceleration required
                # assumes we are stopping
                if (measurement_secs > 0):
                    distance_estimate = abs(distance_left) - \
                            ( (current_vel * self._dt) +
                            (self._decel_rate * measurement_secs * measurement_secs) )

                else:
                    distance_estimate = distance_left

                self._decel_rate = self.acceleration_to_reach_distance(distance_estimate,
                                                                       current_vel,
                                                                       self._dt)

                if (self._decel_rate != 0.0):
                    # Calculate number of steps to decel by counting how long it would take to get here
                    (self._decel_cnt, self._decel_rate) = \
                            self.acceleration_steps_to_reach_velocity(current_vel,
                                                                      self._decel_rate,
                                                                      self._dt)
                else:
                    self._decel_cnt = 0
                
                '''
                print('Distance left: %g Measurement secs: %g Current Velocity: %g Accel: %g Accel cnt: %g Decel: %g Decel Cnt: %g Decel loop: %g' 
                        %   (distance_left, measurement_secs, current_vel, 
                            self._accel, self._accel_cnt, self._decel_rate, 
                            self._decel_cnt, decel_loop_num))
                # '''


            # deceleration leg
            # assumes we are stopping
            self._vel_command = self.apply_acceleration(self._vel_command, 
                                                       0,
                                                       self._decel_rate,
                                                       self._dt)

            if ( (self._vel_command < self._min_vel) and (self._vel_command > 0.0) ) :
                self._vel_command = self._min_vel

        else:
            self._vel_command = 0.0

            # profile finshed
            finished = True

        # assumes we are stopping
        if (distance_left <= 0.0) :
            self._vel_command = 0.0
            goal_reached = True

        return (self._vel_command, finished, goal_reached)

    def compute_trapezoidal_profile(self, dist, vel, accel, dt):
        """Returns a trapezoidal velocity profile with constant acceleration 
        that will traverse a specified distance.

        Returns:
        accel_count
            Number of acceleration ticks at 1/*dt* Hz to apply to reach peak
            velocity.  
            
            Note: Peak velocity may be less than *vel* if there is not enough 
            distance to reach *vel* at the specified acceleration (*accel*).

        accel_rate
            

        coast_count
            Number of coasting ticks at specified velocity (*vel*) before 
            decelerating.

            Note: This will be 0 if peak velocity is less than *vel*.

        coast_vel

        Usage:
        for i in accel_count
            apply acceleration *accel* 
            wait *dt* 
        for i in coast_count
            keep going *vel* m/s
            wait *dt* 
        for i in accel_count
            apply deceleration *accel* 
            wait *dt* 

        """
        dist = abs(dist)
        vel = abs(vel)
        accel = abs(accel)
        dt = abs(dt)

        accel_dist = self.distance_to_accelerate_to_velocity(vel, accel, dt)
        half_way = dist / 2.0
        if (accel_dist < half_way):
            # Enough time to reach full speed
            coast_dist = dist - (2.0 * accel_dist) #  accel and decel
            (coast_count, coast_vel) = self.velocity_steps_to_reach_distance(
                                            coast_dist, vel, dt)
        else:
            # Need to switch straight from accel to decel
            coast_vel = self.peak_velocity_in_distance(half_way, accel, dt)
            coast_count = 0

        (accel_count, accel) = self.acceleration_steps_to_reach_velocity(
                                            coast_vel, accel, dt)

        return (accel_count, accel, coast_count, coast_vel)

    def distance_to_accelerate_to_velocity(self, vel, accel, dt):
        """Returns the distance required to reach the specified velocity with 
        constant acceleration.

        """
        dist = ((vel * dt) ** 2) / (2 * accel * dt * dt)
        return dist

    def acceleration_to_reach_distance(self, dist, vel, dt):
        """Returns the acceleration required to reach the specified distance 
        with specified starting velocity.

        """
        accel = ((vel * dt) ** 2) / (2 * dist * dt * dt)
        return accel

    def peak_velocity_in_distance(self, dist, accel, dt):
        """Returns the peak velocity that can be reached using constant 
        acceleration in the given distance.

        """
        vel = ( (2.0 * dist * accel * dt * dt) ** 0.5 ) / dt
        return vel

    def apply_acceleration(self, old_vel, target_vel, accel, dt):
        """Returns the next velocity step after applying acceleration in the 
        direction of the target.

        """
        if (old_vel == target_vel):
            new_vel = old_vel

        elif (old_vel < target_vel):
            new_vel = old_vel + math.fabs(accel * dt)
            if (new_vel > target_vel):
                new_vel = target_vel

        else:
            new_vel = old_vel - math.fabs(accel * dt)
            if (new_vel < target_vel):
                new_vel = target_vel

        return new_vel

    def acceleration_steps_to_reach_velocity(self, vel, max_accel, dt):
        """Returns the number of discrete steps required to reach the specified
        velocity at constant acceleration.

        """
        steps = math.ceil(vel / (max_accel * dt))

        # acceleration required to correctly reach vel
        if (steps != 0):
            accel = vel / (steps * dt)
        else:
            accel = max_accel

        return steps, accel

    def velocity_steps_to_reach_distance(self, dist, max_vel, dt):
        """Returns the number of discrete steps required to reach the specified
        distance at constant velocity.

        """
        steps = math.ceil(dist / (max_vel * dt))

        # make sure to correctly reach coasting distance
        if (steps != 0):
            vel = dist / (steps * dt)
        else:
            vel = max_vel

        return steps, vel


if __name__ == '__main__':
    pass
