#!/usr/bin/env python

# plot_velocity_computation.py 
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

"""Plots and displays velocity profile and distance profile for 
several states of operation.

Current states are ballisitic profile, distance feedback, and delayed distance 
feedback.

"""

# Changelog
# 2014-05-30 SJL: Created
#

# TODO
# centralize error checking

PKG='simple_base_movement'

import sys
import unittest
import roslib
import math

import matplotlib.pyplot as plotter

from simple_base_movement.velocity_profile_computation \
        import VelocityProfileComputation as VelocityProfiler 

## python unit test
class PlotProfiles(unittest.TestCase):
    def test_run_profiles(self):
        profiles = [
                    [10.0, 1.0, 0.5, 0.1, 0.1],
                    [10.0, 1.0, 0.5, 0.1, 0.0],
                    [math.pi, 5.0, 0.5, 0.05, 0.10],
                    [math.pi, 5.0, 0.5, 0.05, 0.05],
                    [math.pi, 5.0, 0.5, 0.05, 0.0],
                   ]

        for params in profiles:
            run_ballistic_velocity_profile(params[0], params[1], params[2], 
                                            params[3], params[4])

            run_feedback_velocity_profile(params[0], params[1], params[2], 
                                            params[3], params[4])

            run_delayed_feedback_velocity_profile(params[0], params[1], params[2], 
                                            params[3], params[4], int(0.25/params[3]))

            run_delayed_feedback_velocity_profile(params[0], params[1], params[2], 
                                            params[3], params[4], int(0.5/params[3]))

def run_ballistic_velocity_profile(distance_to_travel, max_vel, max_accel, dt, 
                                    aim_short_dist):
    ''' Runs and plots a ballisitic (open-loop) velocity profile. '''

    run_stats = 'Distance: %g Max Velocity: %g Max Acceleration: %g dt: %g Aim Short:%g '\
                    % (distance_to_travel,  max_vel, max_accel, dt, aim_short_dist)

    print '+++ Ballistic Profile +++'
    print run_stats

    profile = VelocityProfiler( aim_short_dist, max_vel, max_accel, dt)

    loop_num = 0
    distance_left = distance_to_travel
    distance_traveled = 0.0

    finished = False

    loop_arr = [0]
    dist_left_arr = [distance_left]
    dist_traveled_arr = [0]
    vel_arr = [0]

    error_loops = []
    error_vels = []

    while not finished:
        (velocity, finished, goal_reached) = \
            profile.next_profile_step_ballistic(distance_left, loop_num) 

        dist = (velocity * dt)
        distance_traveled += dist
        distance_left -= dist

        # whine but don't fail if we exceed accel limits
        loop_num += 1
        accel = abs(velocity - vel_arr[loop_num - 1]) / dt
        if ( accel > max_accel + 0.000001): # addition for floating point alaising
            print 'Loop %g acceleration %g is too high (max %g), velocity %g' % \
                    (loop_num, accel, max_accel, velocity)
            error_loops.append(loop_num - 1)
            error_loops.append(loop_num)
            error_vels.append(vel_arr[loop_num - 1])
            error_vels.append(velocity)
        # Store all the datapoints to display
        loop_arr.append(loop_num)
        vel_arr.append(velocity)
        # dist_left_arr.append(distance_left)
        dist_traveled_arr.append(distance_traveled)

    # clear last data point for pretty graphs ;)
    loop_arr.pop()
    vel_arr.pop()
    dist_traveled_arr.pop()

    title = 'Ballistic\n%s\nDistance to Target: %f' % (run_stats, distance_left)

    generate_plots( loop_arr, dist_traveled_arr, vel_arr, loop_num, 
                    distance_to_travel, max_vel, error_loops, error_vels,
                    title)

    print '+++ Ballistic Actual Distance %g, Expected %g +++\n' % \
            (distance_traveled, distance_to_travel)
    
    '''
    abs(distance_to_travel)
    abs(distance_traveled)
    max_dist = distance_to_travel + (max_accel * dt)
    min_dist = distance_to_travel - (max_accel * dt)

    self.assertLess(distance_traveled, max_dist,
                    "Distance Traveled: %g > Max Distance: %g" % 
                    (distance_traveled, max_dist))

    self.assertGreater(distance_traveled, min_dist,
                    "Distance Traveled: %g < Min Distance: %g!" % 
                    (distance_traveled, min_dist))

    '''

def run_feedback_velocity_profile(distance_to_travel, max_vel,
                                    max_accel, dt, aim_short_dist):
    ''' Runs and plots a ballisitic (open-loop) velocity profile. '''

    run_stats = 'Distance: %g Max Velocity: %g Max Acceleration: %g dt: %g Aim Short: %g'\
                    % (distance_to_travel,  max_vel, max_accel, dt, aim_short_dist)

    print '--- Feedback Profile ---'
    print run_stats

    profile = VelocityProfiler( aim_short_dist, max_vel, max_accel, dt)

    loop_num = 0
    distance_left = distance_to_travel
    distance_traveled = 0.0
    velocity = 0.0

    finished = False

    loop_arr = [0]
    dist_left_arr = [distance_left]
    dist_traveled_arr = [0]
    vel_arr = [0]
    error_loops = []
    error_vels = []

    while not finished:
        (velocity, finished, goal_reached) = \
            profile.next_profile_step_with_feedback(distance_left, velocity, 0, loop_num) 

        dist = (velocity * dt)
        distance_traveled += dist
        distance_left -= dist

        # whine but don't fail if we exceed accel limits
        loop_num += 1
        accel = abs(velocity - vel_arr[loop_num - 1]) / dt
        if ( accel > max_accel + 0.000001): # addition for floating point alaising
            print 'Loop %g acceleration %g is too high (max %g), velocity %g' % \
                    (loop_num, accel, max_accel, velocity)
            error_loops.append(loop_num - 1)
            error_loops.append(loop_num)
            error_vels.append(vel_arr[loop_num - 1])
            error_vels.append(velocity)

        # Store all the datapoints to display
        loop_arr.append(loop_num)
        vel_arr.append(velocity)
        dist_left_arr.append(distance_left)
        dist_traveled_arr.append(distance_traveled)

    # clear last data point for pretty graphs ;)
    loop_arr.pop()
    vel_arr.pop()
    dist_traveled_arr.pop()

    title = 'Feedback\n%s\nDistance to Target: %f' % (run_stats, distance_left)

    generate_plots( loop_arr, dist_traveled_arr, vel_arr, loop_num, 
                    distance_to_travel, max_vel, error_loops, error_vels,
                    title)

    print '--- Feedback Actual Distance %g, Expected %g ---\n' % \
            (distance_traveled, distance_to_travel)

    ''' 
    abs(distance_to_travel)
    abs(distance_traveled)
    max_dist = distance_to_travel + (max_accel * dt)
    min_dist = distance_to_travel - (max_accel * dt)

    self.assertLess(distance_traveled, max_dist,
                    "Distance Traveled: %g > Max Distance: %g" % 
                    (distance_traveled, max_dist))

    self.assertGreater(distance_traveled, min_dist,
                    "Distance Traveled: %g < Min Distance: %g!" % 
                    (distance_traveled, min_dist))

    '''

def run_delayed_feedback_velocity_profile(distance_to_travel, max_vel, max_accel,
                                            dt, aim_short_dist, delay_steps):
    ''' Runs and plots a ballisitic (open-loop) velocity profile. '''

    run_stats = 'Distance: %g Max Velocity: %g Max Acceleration: %g dt: %g Aim Short: %g'\
                    % (distance_to_travel,  max_vel, max_accel, dt, aim_short_dist)

    delay_secs = delay_steps * dt

    print '=== Delayed Feedback Profile ==='
    print run_stats, 'Feedback Delay:', delay_secs

    profile = VelocityProfiler(aim_short_dist, max_vel, max_accel, dt)

    loop_num = 0
    distance_left = distance_to_travel
    distance_traveled = 0.0
    velocity = 0.0

    loop_arr = [0]
    dist_left_arr = [distance_left]
    dist_traveled_arr = [0]
    vel_arr = [0]
    error_loops = []
    error_vels = []

    for x in range(0, delay_steps):
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
                velocity, delay_secs, loop_num) 

        dist = (velocity * dt)
        distance_traveled += dist
        distance_left = dist_left_arr[loop_num + delay_steps] - dist

        # whine but don't fail if we exceed accel limits
        loop_num += 1
        accel = abs(velocity - vel_arr[loop_num + delay_steps - 1]) / dt
        if ( accel > max_accel + 0.000001): # addition for floating point alaising
            print 'Loop %g acceleration %g is too high (max %g), velocity %g' % \
                    (loop_num, accel, max_accel, velocity)

            # record segments which violate
            error_loops.append(loop_num - 1)
            error_loops.append(loop_num)
            error_vels.append(vel_arr[loop_num + delay_steps - 1])
            error_vels.append(velocity)

        # print dist
        # print distance_left
        # print distance_traveled

        # Store all the datapoints to display
        loop_arr.append(loop_num)
        vel_arr.append(velocity)
        dist_left_arr.append(distance_left)
        dist_traveled_arr.append(distance_traveled)

    title = 'Delayed Feedback (%g sec)\n%s\nDistance to Target: %f' % \
            (delay_secs, run_stats, distance_left)

    generate_plots( loop_arr, dist_traveled_arr, vel_arr, loop_num, 
                    distance_to_travel, max_vel, error_loops, error_vels,
                    title )

    print '=== Delayed Feedback (%g sec) Actual Distance %g, Expected %g ===\n' % \
            (delay_secs, distance_traveled, distance_to_travel)
    
    '''
    abs(distance_to_travel)
    abs(distance_traveled)
    max_dist = distance_to_travel + (max_accel * dt)
    min_dist = distance_to_travel - (max_accel * dt)

    self.assertLess(distance_traveled, max_dist,
                    "Distance Traveled: %g > Max Distance: %g" % 
                    (distance_traveled, max_dist))

    self.assertGreater(distance_traveled, min_dist,
                    "Distance Traveled: %g < Min Distance: %g!" % 
                    (distance_traveled, min_dist))

    '''

def generate_plots(loop_arr, dist_arr, vel_arr, loop_num, distance, max_vel, 
                    error_loops, error_vels, profile_name):
    ''' Takes several filled lists which are plotted, then displayed are/or 
    saved to disk.

    loop_arr
        A list containing the incrementing loop counter values
    dist_arr
        A list containing the distance traversed at each step
    vel_arr
        A list containing the commanded velocities at each step

    loop_num
        The number of loops completed (used to scale plot axis)
    distance
        The final distance traveled (used to scale plot axis)
    max_vel
        The maximum velocity reached/set (used to scale plot axis)

    profile_name
        A string which will prefix the plot title
    '''

    # axis limits
    loop_skew = loop_num * 0.1
    dist_skew = distance * 0.1
    vel_skew = max_vel * 0.1

    fig, dist_plot = plotter.subplots()
   
    # plotter.suptitle(profile_name)

    # Plot the velocity profile
    dist_plot.set_xlim(-loop_skew, loop_num + loop_skew)
    dist_plot.set_ylim(-vel_skew, max_vel + vel_skew)
    dist_plot.plot(loop_arr, vel_arr, 'g', lw=4)
    dist_plot.set_title(profile_name)
    dist_plot.set_ylabel('velocity', color='g')
    dist_plot.set_xlabel('loop number')
    for tick in dist_plot.get_yticklabels():
        tick.set_color('g')
    dist_plot.hold(True)
    dist_plot.plot(error_loops, error_vels, 'r', lw=6)

    vel_plot = dist_plot.twinx()

    # Plot the position profile
    vel_plot.set_xlim(-loop_skew, loop_num + loop_skew)
    vel_plot.set_ylim(-dist_skew, distance + dist_skew)
    vel_plot.plot(loop_arr, dist_arr, 'b', lw=4)
    vel_plot.set_ylabel('distance', color='b')
    for tick in vel_plot.get_yticklabels():
        tick.set_color('b')

    plotter.tight_layout()
    plotter.show()

    '''
    # axis limits
    loop_skew = loop_num * 0.1
    dist_skew = distance * 0.1
    vel_skew = max_vel * 0.1
   
    # Plot the velocity profile
    plotter.xlim(-loop_skew, loop_num + loop_skew)
    plotter.ylim(-vel_skew, max_vel + vel_skew)
    plotter.plot(loop_arr, vel_arr)
    plotter.ylabel('velocity')
    plotter.xlabel('loop number')
    plotter.suptitle(profile_name)
    # plotter.suptitle(profile_name + ' Velocity')
    plotter.hold(True)

    # Plot the position profile
    # plotter.xlim(-loop_skew, loop_num + loop_skew)
    plotter.ylim(-dist_skew, distance + dist_skew)
    plotter.plot(loop_arr, dist_arr)
    plotter.ylabel('distance')
    plotter.yaxis.set_label_position("right")
    # plotter.xlabel('loop number')
    # plotter.suptitle(profile_name + ' Distance')
    plotter.show()
    
    '''

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'plot_velocity_computation', PlotProfiles)
