#! /usr/bin/env python

# distance_controller.py 
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

"""Provides ROS ActionLib actions to move and turn specified distances
and angles.

"""

# Changelog
# 2014-02-25 SJL: Created.
# 2014-02-28 SJL: Internal functions added for straight line movement
#                 and turning in place.
# 2014-03-01 SJL: Changed velocity parameter names to match 
#                 base_local_planner node.
#                 Changed shutdown to use python sleep in case roscore
#                 dies.
#                 Changed node name to distance_controller
# 2014-03-05 SJL: Updated the node for dynamic reconfigure. 
#

# TODO
# Accept non-action goals (topics)
# publish whether we are moving the robot
# linear and angual movements at the same time

import roslib 
import time
import rospy
import actionlib
import math
import tf

from geometry_msgs.msg import Twist, Point, Quaternion
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

import simple_base_movement.msg
from simple_base_movement.cfg import DistanceControllerConfig as ConfigType

from velocity_profile_computation \
        import VelocityProfileComputation as VelocityProfiler

class DistanceController(object):
    def __init__(self, name):
        """Creates an ROS node and the publishers/listeners/parameters needed.
        
        Arguments:
        name    --  ROS Node Name

        """

        self.using_distance_feedback = True

        # Create ROS node
        rospy.init_node(name)

        # Set rospy to exectute a shutdown function when terminating the    
        # script
        rospy.on_shutdown(self.shutdown)

        # Create a dynamic reconfigure server.
        rospy.loginfo('Starting %s Dynamic Reconfigure Server', name)
        self._reconfigure_server = DynamicReconfigureServer(ConfigType, 
                                                            self.reconfigure)

        # create messages that are used to publish feedback/result
        self._action_feedback = simple_base_movement.msg.MoveDistanceFeedback()
        self._action_result   = simple_base_movement.msg.MoveDistanceResult()

        # create action server
        rospy.loginfo('Starting %s Action Server', name)
        self._action_name = name
        self._action_server = actionlib.SimpleActionServer(self._action_name,
                        simple_base_movement.msg.MoveDistanceAction, 
                        execute_cb=self.action_callback, auto_start = False)

        self._action_server.start()

        # create tf listener object
        self._tf_listener = tf.TransformListener()

        # create the topic publishers
        self._velocity_command = rospy.Publisher('cmd_vel', Twist)
#         self._velocity_command = rospy.Publisher('is_moving', bool)

    def reconfigure(self, config, level):
        """Fill in local variables with values received from dynamic reconfigure 
        clients (typically the GUI).
        
        """
        
        # Robot parameters
        self.max_speed      = config["max_vel_x"]
        self.min_speed      = config["min_vel_x"]
        self.accel_speed    = config["acc_lim_x"]
        self.max_turn       = config["max_rotational_vel"]
        self.min_turn       = config["min_rotational_vel"]
        self.accel_turn     = config["acc_lim_rotation"]

        # Controller parameters
        self.base_frame     = config["robot_base_frame"]
        self.global_frame   = config["global_frame_id"]
        self.update_rate    = config["controller_frequency"]
        self.truncate_angle = config["normalize_angle"]
        self.move_delay     = config["post_move_delay"]

        # Goal tolerances
        self.linear_tolerance   = config["xy_goal_tolerance"]
        self.angular_tolerance  = config["yaw_goal_tolerance"]
        self.latch_goal     = config["latch_xy_goal_tolerance"]

        # Precision endpoint parameters
        self.precision_speed    = config["precision_vel_x"]
        self.precision_turn     = config["precision_rotational_vel"] 
        self.precision_distance = config["precision_distance"]
        self.precision_angle    = config["precision_angle"]
        
        rospy.loginfo("""Reconfigure:
                       max_vel_x: %f
                       min_vel_x: %f
                       acc_lim_x: %f
                       max_rotational_vel: %f
                       min_in_place_rotational_vel: %f
                       acc_lim_th: %f
                       robot_base_frame: %s
                       global_frame_id: %s
                       controller_frequency: %f
                       normalize_angle: %f
                       post_move_delay: %f
                       xy_goal_tolerance: %f
                       yaw_goal_tolerance: %f
                       latch_xy_goal_tolerance: %d
                       precision_distance: %f
                       precision_angle: %f
                       precision_vel_x: %f
                       precision_rotational_vel: %f""" 
                % ( self.max_speed,
                    self.min_speed,      
                    self.accel_speed,    
                    self.max_turn,       
                    self.min_turn,       
                    self.accel_turn,     
                    self.base_frame,     
                    self.global_frame,   
                    self.update_rate,    
                    self.truncate_angle, 
                    self.move_delay,     
                    self.linear_tolerance,
                    self.angular_tolerance,
                    self.latch_goal,    
                    self.precision_distance, 
                    self.precision_angle,  
                    self.precision_speed,
                    self.precision_turn) )

        return config

    def action_callback(self, goal):
        """Action callback function which moves specified distances and angles.
        
        Blocks until the goal is reached or otherwise stopped.
            
        """

        # publish info to the console for the user
        rospy.loginfo('%s: Executing:\tX %f, Theta %f' % \
                      (self._action_name, goal.distance, goal.angle))

        success = True

        self._action_feedback.current_distance = 0; 
        self._action_feedback.current_angle = 0; 

        if (goal.distance != 0.0):
            # success = self.drive_forward_distance(goal.distance)
            success = self.drive_forward_trap_profile(goal.distance)

        if (goal.angle != 0.0) and success:
            # success = self.turn_angle_distance(goal.angle)
            success = self.turn_angle_trap_profile(goal.angle)

        self._action_result.final_distance = self._action_feedback.current_distance
        self._action_result.final_angle = self._action_feedback.current_angle

        if success:
            rospy.loginfo('%s: Succeeded:\tX %f, Theta %f\n' 
                    % (self._action_name, 
                    self._action_result.final_distance,
                    self._action_feedback.current_angle) )

            self._action_server.set_succeeded(self._action_result)
        else:
            rospy.loginfo('%s: Failed:\tX %f, Theta %f\n' 
                    % (self._action_name, 
                    self._action_result.final_distance,
                    self._action_feedback.current_angle) )

            self._action_server.set_aborted(self._action_result)

    def drive_forward_distance(self, goal_distance):
        """Continually sends commands to move a robot straight forward 
        until the specified distance has been reached.
        
        Uses tf transforms to determine the distance the robot has
        traveled. No obstacle avoidance or other safety checks are 
        performed. The robot will stop after the tf transforms indicate
        the requested distance has been traveled.

        """

        goal_reached = True;

        # make sure TF transform exists
        if not self.check_frames():
            goal_reached = False
            self._action_server.set_aborted()
            return goal_reached

        (start_position, start_rotation, time) = self.get_frame_transform()

        # setup the base command
        base_command = Twist()

        if goal_distance > 0:
            base_command.linear.x = self.max_speed;
        else:
            base_command.linear.x = -self.max_speed;

        base_command.angular.z = 0.0

        x_start = start_position.x
        y_start = start_position.y
        
        # keep track of the distance traveled
        distance = 0
        
        rate = rospy.Rate(self.update_rate)

        # publish the base velocities command
        self._velocity_command.publish(base_command)

        # enter the loop to move requested distance
        while abs(distance) < abs(goal_distance):
             # get the current position
            (position, rotation, time) = self.get_frame_transform()
           
            # check for ROS 
            if rospy.is_shutdown():
                rospy.loginfo('%s: ROS Shutdown' % self._action_name)
                self._action_server.set_aborted()
                goal_reached = False
                break

            # check for goal canceled
            if self._action_server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._action_server.set_preempted()
                goal_reached = False
                break

            # compute the Euclidean distance from the start
            distance = math.sqrt(pow((position.x - x_start), 2) + 
                            pow((position.y - y_start), 2))

            distance_left = abs(goal_distance) - abs(distance)

            # check if we are supposed to slow down before reaching the goal
            if (self.precision_distance > 0) and \
                    (distance_left < self.precision_distance):
                if goal_distance > 0:
                    base_command.linear.x = self.precision_speed;
                else:
                    base_command.linear.x = -self.precision_speed;
    
            # publish the base velocities command
            self._velocity_command.publish(base_command)
            
            # publish action feedback
            self._action_feedback.current_distance = distance
            self._action_server.publish_feedback(self._action_feedback)

            rate.sleep()

        # stop the robot when done
        base_command = Twist()
        self._velocity_command.publish(base_command)

        rospy.sleep(self.move_delay)

        # get the current position
        (position, rotation, time) = self.get_frame_transform()
           
        # compute the Euclidean distance from the start
        distance = math.sqrt(pow((position.x - x_start), 2) + 
                        pow((position.y - y_start), 2))

        # publish action feedback
        self._action_feedback.current_distance = distance
        self._action_server.publish_feedback(self._action_feedback)

        return goal_reached

    def drive_forward_trap_profile(self, goal_distance):
        """Continually sends commands to move a robot straight forward 
        until the specified distance has been reached, using a trapezoidal
        velocity profile.
        
        """

        rospy.loginfo('%s: Drive Distance: %.2f' % (self._action_name, goal_distance))

        goal_reached = False

        # make sure TF transform exists
        if not self.check_frames():
            goal_reached = False
            self._action_server.set_aborted()
            return goal_reached

        (start_position, start_rotation, time) = self.get_frame_transform()

        # setup the base command
        base_command = Twist()

        x_start = start_position.x
        y_start = start_position.y
        
        # keep track of the distance traveled
        distance = 0
        self.linear_vel = 0
        
        rate = rospy.Rate(self.update_rate)
        period = 1.0 / self.update_rate

        # setup the velocity controllers
        profile = VelocityProfiler(self.precision_distance, 
                                   self.max_speed, 
                                   self.min_speed, 
                                   self.accel_speed, 
                                   period)

        finished = False
        loop_num = 0
        
        # enter the loop to move requested distance
        while not finished:
            # get the current position
            (position, rotation, time) = self.get_frame_transform()
           
            # check for ROS 
            if rospy.is_shutdown():
                rospy.loginfo('%s: ROS Shutdown' % self._action_name)
                self._action_server.set_aborted()
                goal_reached = False
                break

            # check for goal canceled
            if self._action_server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._action_server.set_preempted()
                goal_reached = False
                break

            # compute the Euclidean distance from the start
            distance = math.sqrt(pow((position.x - x_start), 2) + 
                                 pow((position.y - y_start), 2))

            distance_left = abs(goal_distance) - abs(distance)

            rospy.loginfo('%s: Distance Left: %.2f' % (self._action_name, distance_left))

            #if (goal_distance < 0):
            #    distance_left = -distance_left
            #    distance = -distance

            # call different functions here
            if self.using_distance_feedback:
                now = rospy.Time.now() 
                distance_age = now - time

                # print(now)
                # print(time)

                #  distance_left is recalculated based on how old the measurement is
                measurement_secs = distance_age.to_sec()

                (command, finished, goal_reached) = \
                        profile.next_profile_step_with_feedback(distance_left, 
                                                                self.linear_vel,
                                                                measurement_secs,
                                                                loop_num)
            else: 
                (command, finished, goal_reached) = \
                        profile.next_profile_step_ballistic(distance_left, 
                                                            loop_num)

            # needed a fudge factor for floating point aliasing, I think 0.01% is close enough
            actual_accel = (0.9999 * abs(command - base_command.linear.x))/ period
            last_vel = self.linear_vel

            if (goal_distance < 0):
                command *= -1

            base_command.linear.x = command
            self.linear_vel = command

            if (actual_accel > self.accel_speed):
                rospy.logwarn('%s: Linear Acceleration limit exceeded! Limit %g, Acceleration %g, Velocity %g' % 
                                (self._action_name, self.accel_speed, 
                                actual_accel, last_vel))

            # publish the base velocities command
            self._velocity_command.publish(base_command)
            
            # publish action feedback
            self._action_feedback.current_distance = distance
            self._action_server.publish_feedback(self._action_feedback)

            loop_num += 1
            rate.sleep()

        # stop the robot when done
        base_command = Twist()
        self._velocity_command.publish(base_command)

        rospy.sleep(self.move_delay)

        # get the current position
        (position, rotation, time) = self.get_frame_transform()
           
        # compute the Euclidean distance from the start
        distance = math.sqrt(pow((position.x - x_start), 2) + 
                        pow((position.y - y_start), 2))

        if (goal_distance < 0):
            distance = -distance

        # publish action feedback
        self._action_feedback.current_distance = distance
        self._action_server.publish_feedback(self._action_feedback)

        return goal_reached

    def turn_angle_distance(self, goal_angle):
        """Continually sends commands to turn a robot until the 
        specified angle has been reached.
        
        Uses tf transforms to determine the angle the robot has
        turned. No obstacle avoidance or other safety checks are 
        performed. The robot will stop after the tf transforms indicate
        the requested angle has been turned.

        """

        goal_reached = True;

        # make sure TF transform exists
        if not self.check_frames():
            goal_reached = False
            self._action_server.set_aborted()
            return goal_reached

        (start_position, start_rotation, time) = self.get_frame_transform()

        # setup the base command
        base_command = Twist()

        if self.truncate_angle:
            goal_angle = self.normalize_angle(goal_angle)

        if goal_angle > 0:
            base_command.angular.z = self.max_turn
            angular_tolerance = self.angular_tolerance
        else:
            base_command.angular.z = -self.max_turn
            angular_tolerance = -self.angular_tolerance

        x_start = start_position.x
        y_start = start_position.y

        last_angle = start_rotation
        turn_angle = 0

        rate = rospy.Rate(self.update_rate)

        # publish the base velocities command
        self._velocity_command.publish(base_command)

        # enter the loop to move requested distance
        while abs(turn_angle + angular_tolerance) < abs(goal_angle):
            # get the current position
            (position, rotation, time) = self.get_frame_transform()
            
            # check for ROS 
            if rospy.is_shutdown():
                rospy.loginfo('%s: ROS Shutdown' % self._action_name)
                self._action_server.set_aborted()
                goal_reached = False
                break

            # check for goal canceled
            if self._action_server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._action_server.set_preempted()
                goal_reached = False
                break

            # compute the amount of rotation since the last loop
            delta_angle = self.normalize_angle(rotation - last_angle)
                
            turn_angle += delta_angle
            last_angle = rotation

            angle_left = abs(goal_angle) - abs(turn_angle)
            
            # check if we are supposed to slow down before reaching the goal
            if (self.precision_angle > 0) and \
                    (angle_left < self.precision_angle):
                if goal_angle > 0:
                    base_command.angular.z = self.precision_turn;
                else:
                    base_command.angular.z = -self.precision_turn;
                   
            # publish the base velocities command
            self._velocity_command.publish(base_command)

            # publish action feedback
            self._action_feedback.current_angle = turn_angle
            self._action_server.publish_feedback(self._action_feedback)

            rate.sleep()
    
        # Stop the robot when done
        base_command = Twist()
        self._velocity_command.publish(base_command)

        rospy.sleep(self.move_delay)

        # get the current position
        (position, rotation, time) = self.get_frame_transform()

        # compute the amount of rotation since the last loop
        delta_angle = self.normalize_angle(rotation - last_angle)
        turn_angle += delta_angle

        # publish action feedback
        self._action_feedback.current_angle = turn_angle
        self._action_server.publish_feedback(self._action_feedback)

    def turn_angle_trap_profile(self, goal_angle):
        """Continually sends commands to move a robot straight forward 
        until the specified distance has been reached, using a trapezoidal
        velocity profile.
        
        """

        rospy.loginfo('%s: Drive Angle: %.2f' % (self._action_name, goal_angle))

        goal_reached = False

        # make sure TF transform exists
        if not self.check_frames():
            goal_reached = False
            self._action_server.set_aborted()
            return goal_reached

        (start_position, start_rotation, time) = self.get_frame_transform()

        # constrain angle for shortest turn if configured
        if self.truncate_angle:
            goal_angle = self.normalize_angle(goal_angle)

        # setup the base command
        base_command = Twist()

        x_start = start_position.x
        y_start = start_position.y
        
        # keep track of the distance traveled
        last_angle = start_rotation
        turn_angle = 0
        self.angular_vel = 0
        
        rate = rospy.Rate(self.update_rate)
        period = 1.0 / self.update_rate

        # setup the velocity controllers
        profile = VelocityProfiler(self.precision_angle, self.max_turn, self.min_turn,
                                    self.accel_turn, period)

        finished = False
        loop_num = 0
        
        # enter the loop to move requested distance
        while not finished:
            # get the current position
            (position, rotation, time) = self.get_frame_transform()
           
            # check for ROS 
            if rospy.is_shutdown():
                rospy.loginfo('%s: ROS Shutdown' % self._action_name)
                self._action_server.set_aborted()
                goal_reached = False
                break

            # check for goal canceled
            if self._action_server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._action_server.set_preempted()
                goal_reached = False
                break

            # compute the amount of rotation since the last loop
            delta_angle = self.normalize_angle(rotation - last_angle)
                
            turn_angle += delta_angle
            last_angle = rotation

            angle_left = abs(goal_angle) - abs(turn_angle)

            rospy.loginfo('%s: Angle Left: %.2f' % (self._action_name, angle_left))

            if (goal_angle < 0):
                angle_left = -angle_left
            
            # call different functions here
            if self.using_distance_feedback:
                angle_age = rospy.Time.now() - time

                #  distance_left is recalculated based on how old the measurement is
                measurement_secs = angle_age.to_sec()

                (command, finished, goal_reached) = \
                        profile.next_profile_step_with_feedback(angle_left, 
                                                                self.angular_vel,
                                                                measurement_secs,
                                                                loop_num)
            else: 
                (command, finished, goal_reached) = \
                        profile.next_profile_step_ballistic(angle_left, loop_num)

            # rospy.loginfo('goal_angle %f, turn_angle %f, angle left %f' % 
            #                 (goal_angle, turn_angle, angle_left))

            actual_accel = abs(command - base_command.angular.z) / period
            last_vel = self.angular_vel

            if (actual_accel > self.accel_turn):
                rospy.logwarn('%s: Angular acceleration limit exceeded! Limit %g, Acceleration %g, Velocity %g' % 
                                (self._action_name, self.accel_turn, 
                                actual_accel, last_vel))

            base_command.angular.z = command;
            self.angular_vel = command;

            # publish the base velocities command
            self._velocity_command.publish(base_command)
            
            # publish action feedback
            self._action_feedback.current_angle = turn_angle
            self._action_server.publish_feedback(self._action_feedback)

            loop_num += 1
            rate.sleep()

        # stop the robot when done
        base_command = Twist()
        self._velocity_command.publish(base_command)

        rospy.sleep(self.move_delay)

        # get the current position
        (position, rotation, time) = self.get_frame_transform()
           
        # compute the amount of rotation since the last loop
        delta_angle = self.normalize_angle(rotation - last_angle)
        turn_angle += delta_angle

        # publish action feedback
        self._action_feedback.current_angle = turn_angle
        self._action_server.publish_feedback(self._action_feedback)
        
        return goal_reached

    def check_frames(self):
        """Checks to see if an ROS tf transform exists between the 
        specified frames.

        Returns True if the transform exists, False otherwise.
        
        """

        try:
            self._tf_listener.waitForTransform(self.base_frame, 
                self.global_frame, rospy.Time(0), rospy.Duration(1.0))

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between frames %s, %s",
                          self.global_frame, self.base_frame)
            rospy.signal_shutdown("check_for_frames: TF Exception")
            return False

        return True

    def get_frame_transform(self):
        """Get the current transform between the odom and base frames.

        Returns trans   => X, Y, and Z offsets (meters)
        Returns rot     => Z-Axis angle (radians)

        """

        try:
            (trans, rot)  = self._tf_listener.lookupTransform(self.global_frame,
                                                                self.base_frame,
                                                                rospy.Time(0))

            time = self._tf_listener.getLatestCommonTime(self.global_frame, 
                                                            self.base_frame)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            rospy.signal_shutdown("get_frame_transform: TF Exception")
            return False, False, False

        return (Point(*trans), self.quat_to_angle(rot), time)

    def quat_to_angle(self, quat):
        """Returns the rotation around the Z-Axis (in radians) defined 
        by the quaternion.
        
        """
        angles = tf.transformations.euler_from_quaternion(quat)
        return angles[2]

    def normalize_angle(self, angle):
        """Constrains an angle to between +- pi."""

        res = angle
        while res > math.pi:
            res -= 2.0 * math.pi
        while res < -math.pi:
            res += 2.0 * math.pi
        return res

    def shutdown(self):
        """Stops the robot when shutting down the node."""

        rospy.loginfo("Stopping the robot...")
        self._velocity_command.publish(Twist())
        # Using time instead of rospy in case the ROS server dies
        time.sleep(1)


if __name__ == '__main__':
    distance_controller = DistanceController('distance_controller')
    rospy.spin()

#     try:
#     except rospy.ROSInterruptException:
#         # cleanly exit if shutdown called while sleeping
#         sys.exit(0)        

