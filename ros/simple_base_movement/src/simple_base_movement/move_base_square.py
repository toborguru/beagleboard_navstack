#! /usr/bin/env python

# move_base_square.py 
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

"""Moves the robot base in a square.

"""

# Changelog
# 2014-03-10 SJL: Created
#

import roslib 
import rospy
import time
import actionlib
import math

from math import pi

from simple_base_movement.msg import MoveDistanceAction, MoveDistanceGoal

class MoveBaseSquare(object):
    def __init__(self, name):
        """Creates an ROS node 
        
        Arguments:
        name    --  ROS Node Name

        """

        # create ROS node
        rospy.init_node(name)

        # set rospy to exectute a shutdown function when terminating the    
        # script
        rospy.on_shutdown(self.shutdown)

        # length of each side of the square
        square_size = rospy.get_param("~size", 1.0) #  meters
        turn_clockwise  = rospy.get_param("~turn_clockwise", False) # CCW otherwise
        # time to drive one side of the square before timing out
        expected_duration = rospy.get_param("~side_time", 60.0) # seconds 

        movement_steps = list()

        turn_angle = pi/2

        if turn_clockwise:
            turn_angle = -turn_angle

        # store the actions required
        for x in range(0, 4):
            movement_steps.append(MoveDistanceGoal(square_size, 0))
            movement_steps.append(MoveDistanceGoal(0, turn_angle))

        # connect to the action server
        self._action_client = actionlib.SimpleActionClient("distance_controller",
                                                           MoveDistanceAction)

        rospy.loginfo("Connecting to distance_controller action server...")
       
        # wait for the server
        self._action_client.wait_for_server(rospy.Duration(60))
       
        rospy.loginfo("Connected")

        # Wait just a little longer for the action lib server, this has helped 
        # with problems on first goal locking or getting falsely finished.
        rospy.sleep(0.1)
           
        rospy.loginfo("Starting move_base_square task")

        self._is_running = True;
       
        i = 0
        num_steps = len(movement_steps)

        while i < num_steps and not rospy.is_shutdown():
            rospy.loginfo("Sending Goal:\tDistance %f, Angle %f (Degrees %f)" %
                            (movement_steps[i].distance,
                             movement_steps[i].angle,
                             math.degrees(movement_steps[i].angle)))

            # Send the goal pose to the MoveBaseAction server
            self._action_client.send_goal(movement_steps[i])
           
            # Allow 1 minute to get there
            finished_within_time = self._action_client.wait_for_result(rospy.Duration(60))
           
            # If we don't get there in time, abort the goal
            if not finished_within_time:
                self._action_client.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                # state = self._action_client.get_state()
                result = self._action_client.get_result()
                rospy.loginfo("Reached Goal:\tDistance %f, Angle %f (Degrees %f)" % 
                                (result.final_distance,
                                 result.final_angle,
                                 math.degrees(result.final_angle)))

                if (movement_steps[i].distance != 0):
                    dist_error = 100.0 * (abs(movement_steps[i].distance - 
                                                result.final_distance) /
                                          abs(movement_steps[i].distance))
                else:
                    dist_error = 0.0

                if (movement_steps[i].angle != 0):
                    angle_error = 100.0 * (abs(movement_steps[i].angle - 
                                                result.final_angle) / 
                                           abs(movement_steps[i].angle))
                else:
                    angle_error = 0

                rospy.loginfo("Goal Errors: \tDistance %f%% Angle %f%%\n" % 
                                (dist_error, angle_error))

            i += 1 

        self._is_running = False;


    def shutdown(self):
        rospy.loginfo("move_base_square shutdown()...")

        if self._is_running:
                self._action_client.cancel_goal()
                self._is_running = False;

        #  Using time instead of rospy in case the ROS server dies
        time.sleep(1) 


if __name__ == '__main__':
    try:
        MoveBaseSquare('move_base_square')
    except rospy.ROSInterruptException:
       pass

