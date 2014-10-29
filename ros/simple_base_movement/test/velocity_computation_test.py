#!/usr/bin/env python

# velocity_computation_test.py 
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

"""Tests the velocity_profile_computation module.

"""

# Changelog
# 2014-05-30 SJL: Created
#

PKG='simple_base_movement'

import sys
import unittest
import roslib

from simple_base_movement.velocity_profile_computation \
        import VelocityProfileComputation as VelocityProfiler 

## python unit test
class TestVelocityProfiles(unittest.TestCase):

    def test_plot_ballistic_velocity_profile(self):
        aim_short_dist = 0.0
        max_vel = 1.0
        max_accel = 0.5
        dt = 0.1

        profile = VelocityProfiler( aim_short_dist, max_vel, max_accel, dt)

        loop_num = 0
        distance_to_travel = 10.0
        distance_left = distance_to_travel
        distance_traveled = 0.0

        finished = False

        loop_arr = [0]
        dist_left_arr = [distance_left]
        dist_traveled_arr = [0]
        vel_arr = [0]

        while not finished:
            (velocity, finished, goal_reached) = \
                profile.next_profile_step_ballistic(distance_left, loop_num) 

            dist = (velocity * dt)
            distance_traveled += dist
            distance_left -= dist
            loop_num += 1

            loop_arr.append(loop_num)
            vel_arr.append(velocity)
            dist_left_arr.append(distance_left)
            dist_traveled_arr.append(distance_traveled)

        abs(distance_to_travel)
        abs(distance_traveled)
        max_dist = distance_to_travel + (max_accel * dt)
        min_dist = distance_to_travel - (max_accel * dt)

        print 'Ballistic Actual Distance %g, Expected %g' % 
                (distance_traveled, distance_to_travel)

        self.assertLess(distance_traveled, max_dist,
                        "Distance Traveled: %g > Max Distance: %g" % 
                        (distance_traveled, max_dist))
 
        self.assertGreater(distance_traveled, min_dist,
                        "Distance Traveled: %g < Min Distance: %g!" % 
                        (distance_traveled, min_dist))
 
    def test_plot_feedback_velocity_profile(self):
        aim_short_dist = 0.1
        max_vel = 1.0
        max_accel = 0.5
        dt = 0.1

        profile = VelocityProfiler( aim_short_dist, max_vel, max_accel, dt)

        loop_num = 0
        distance_to_travel = 10.0
        distance_left = distance_to_travel
        distance_traveled = 0.0
        velocity = 0.0

        finished = False

        loop_arr = [0]
        dist_left_arr = [distance_left]
        dist_traveled_arr = [0]
        vel_arr = [0]

        while not finished:
            (velocity, finished, goal_reached) = \
                    profile.next_profile_step_with_feedback(distance_left, 
                    velocity, 0, loop_num) 

            dist = (velocity * dt)
            distance_traveled += dist
            distance_left -= dist
            loop_num += 1

            loop_arr.append(loop_num)
            vel_arr.append(velocity)
            dist_left_arr.append(distance_left)
            dist_traveled_arr.append(distance_traveled)

        abs(distance_to_travel)
        abs(distance_traveled)
        max_dist = distance_to_travel + (max_accel * dt)
        min_dist = distance_to_travel - (max_accel * dt)

        print 'Feedback Actual Distance %g, Expected %g' % 
                (distance_traveled, distance_to_travel)

        self.assertLess(distance_traveled, max_dist,
                        "Distance Traveled: %g > Max Distance: %g" % 
                        (distance_traveled, max_dist))
 
        self.assertGreater(distance_traveled, min_dist,
                        "Distance Traveled: %g < Min Distance: %g!" % 
                        (distance_traveled, min_dist))
 
    def test_plot_delayed_feedback_velocity_profile(self):
        feedback_delay = 3 # dt steps
        aim_short_dist = 0.1
        max_vel = 1.0
        max_accel = 0.5
        dt = 0.1

        profile = VelocityProfiler( aim_short_dist, max_vel, max_accel, dt)

        loop_num = 0
        distance_to_travel = 10.0
        distance_left = distance_to_travel
        distance_traveled = 0.0
        velocity = 0.0

        loop_arr = [0]
        dist_left_arr = [distance_left]
        dist_traveled_arr = [0]
        vel_arr = [0]

        for x in range(0, feedback_delay):
            loop_arr.append(0)
            dist_left_arr.append(distance_left)
            dist_traveled_arr.append(0)
            vel_arr.append(0)

        finished = False

        while not finished:
            distance_left = dist_left_arr[loop_num]

            # print
            # print loop_num
            # print velocity
            # print distance_left

            (velocity, finished, goal_reached) = \
                    profile.next_profile_step_with_feedback(distance_left, 
                    velocity, 0.3, loop_num)

            dist = (velocity * dt)
            distance_traveled += dist
            distance_left = dist_left_arr[loop_num + feedback_delay] - dist
            loop_num += 1

            # print dist
            # print distance_left
            # print distance_traveled

            loop_arr.append(loop_num)
            vel_arr.append(velocity)
            dist_left_arr.append(distance_left)
            dist_traveled_arr.append(distance_traveled)

        abs(distance_to_travel)
        abs(distance_traveled)
        max_dist = distance_to_travel + (max_accel * dt)
        min_dist = distance_to_travel - (max_accel * dt)

        print 'Delayed Feedback (%g sec) Actual Distance %g, Expected %g' % 
                ((feedback_delay * dt), distance_traveled, distance_to_travel)

        self.assertLess(distance_traveled, max_dist,
                        "Distance Traveled: %g > Max Distance: %g" % 
                        (distance_traveled, max_dist))
 
        self.assertGreater(distance_traveled, min_dist,
                        "Distance Traveled: %g < Min Distance: %g!" % 
                        (distance_traveled, min_dist))
 
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'plot_velocity_computation', TestBareBones)



