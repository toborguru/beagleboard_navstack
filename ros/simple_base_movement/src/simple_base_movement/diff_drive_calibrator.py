#! /usr/bin/env python

# diff_drive_calibrator.py 
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

"""Program to help calibrate a differentially driven robot for ROS

"""

# Changelog
# 2020-04-16 SJL: Created
#

import roslib 
import rospy
import time
import actionlib
import math

from math import pi

from simple_base_movement.msg import MoveDistanceAction, MoveDistanceGoal

MM_TO_M = (1.0 / 1000.0)
M_TO_MM = 1000.0

CM_TO_M = (1.0 / 100.0)
M_TO_CM = 100.0

IN_TO_M = 0.0254
M_TO_IN = 39.3701

FT_TO_M = 0.3048
M_TO_FT = 3.28084

YD_TO_M = 0.9144
M_TO_YD = 1.09361


class CalibrateDiffDrive(object):
    def __init__(self, name):
        """Creates an ROS node 
        
        Arguments:
        name    --  ROS Node Name

        """

        self._is_running = False
        self._test_log = []
        self._test_log.append(["test_name", "reported_value", "measured_value", "correction_factor"])

        # create ROS node
        rospy.init_node(name)

        # set rospy to exectute a shutdown function when terminating the    
        # script
        rospy.on_shutdown(self.shutdown)

        rospy.logdebug("Connecting to distance_controller action server...")

        # connect to the action server
        self._action_client = actionlib.SimpleActionClient("distance_controller",
                                                           MoveDistanceAction)
        # wait for the server
        self._action_client.wait_for_server(rospy.Duration(60))
       
        rospy.loginfo("Connected to action server")

        print("")

        correction = 1.0
        while (correction != 0.0) :
            print("")
            print("Would you like to test your distance calibration?")
            test_dist = raw_input("[Y/n] ")
            test_split = test_dist.split()
            split_len = len(test_split)

            run_test = False

            if (split_len == 0) :
                run_test = True
            else :
                if ( (test_dist[0] == "y") or (test_dist[0] == "Y") ) :
                    run_test = True

            if (run_test) :
                correction = self.distanceCalibration()
            else :
                correction = 0.0

            print("")
            print("------------------------------------------------")

        correction = 1.0
        while (correction != 0.0) :
            print("")
            print("Would you like to test your angle calibration?")
            test_dist = raw_input("[Y/n] ")
            test_split = test_dist.split()
            split_len = len(test_split)

            run_test = False

            if (split_len == 0) :
                run_test = True
            else :
                if ( (test_dist[0] == "y") or (test_dist[0] == "Y") ) :
                    run_test = True

            if (run_test) :
                correction = self.angleCalibration()
            else :
                correction = 0.0

            print("")
            print("------------------------------------------------")

        correction = 0.0
        while (correction != 1.0) :
            print("")
            print("Would you like to test your wheel ratio calibration - spinning?")
            test_dist = raw_input("[Y/n] ")
            test_split = test_dist.split()
            split_len = len(test_split)

            run_test = False

            if (split_len == 0) :
                run_test = True
            else :
                if ( (test_dist[0] == "y") or (test_dist[0] == "Y") ) :
                    run_test = True

            if (run_test) :
                correction = self.ratioSpinCalibration()
            else :
                correction = 1.0

            print("")
            print("------------------------------------------------")

        self.printLogs()

        turn_angle = pi/2


    def distanceCalibration( self ) :
        print("How far would you like the robot to travel? [-]dist [unit]")
        print("  unit - m, cm, mm, ft, yd, in")
        print("Ex: 1.25 m , 10 cm , -12 in, 4 yd")
       
        cmd_length, cmd_units, unit_length = self.promptDistance( 1.0, "m" )

        if (cmd_units == "m") :
            print("Distance to travel: %.2f %s" % (cmd_length, cmd_units) )
        else :
            print("Distance to travel: %.2f %s (%.2f m)" % (unit_length, cmd_units, cmd_length) )

        # Distance traveled as seen by the robot
        reported_length = self.requestMovementAction(cmd_length, 0)

        reported_units = self.metersToUnits(reported_length, cmd_units)
        
        if (cmd_units == "m") :
            print("The robot thinks it moved %.2f %s" % (reported_units, cmd_units))
        else :
            print("The robot thinks it moved %.2f %s (%.2f m)" % (reported_units, cmd_units, reported_length))
            
        print("How far did the robot actually move?")
        
        # Distance traveled as measured by a ruler
        meas_length, meas_units, meas_unit_length = self.promptDistance( reported_units, cmd_units )

        if (meas_length != reported_length) :
            corr_factor = (meas_length / reported_length)
            print("The length needs a correction factor of %.4f applied (wheel diameter)" % corr_factor)
        else :
            corr_factor = 1.0

        self._test_log.append(["distance_test", reported_length, meas_length, corr_factor])

        return corr_factor


    def angleCalibration( self ) :
        print("How many times would you like the robot to spin? [-]spins")
        spins = 0

        while (spins == 0) :
            spins_txt = raw_input("[3]: ")

            spins_split = spins_txt.split()
            num_args = len(spins_split)

            if (num_args == 0) :
                spins = 3
            elif (num_args == 1) :
                try :
                    spins = int(spins_split[0])
                except :
                    spins = 0

            else:
               spins = 0    
                
            if (spins == 0) :
                print("Invalid input: %s" % spins_txt)

        print("Times to spin around: %d" % spins )

        turn_angle = spins * pi * 2.0

        # Distance traveled as seen by the robot
        reported_angle = self.requestMovementAction(0, turn_angle)

        overshoot = math.degrees(reported_angle - turn_angle)

        print("The robot thinks it completed the turn with %.2f degrees of overshoot" % (overshoot))
            
        print("How many degrees did the robot actually overshoot?")
        meas_txt = raw_input("[%.4f] " % overshoot)
        meas_split = meas_txt.split()
        num_args = len(meas_split)

        if (num_args == 0) :
            meas_angle = reported_angle
        elif (num_args == 1) :
            try :
                meas_angle = float(meas_split[0])

                meas_angle = math.radians(meas_angle)

                if (turn_angle >= 0) :
                    meas_angle += turn_angle
                else :
                    meas_angle -= turn_angle

            except :
                meas_angle = 0.0
        else :
            meas_angle = 0.0
        
        # Distance traveled as measured by a ruler

        if (meas_angle != reported_angle) :
            corr_factor = abs( reported_angle / meas_angle )
            print("The angle needs a correction factor of %.4f applied (wheel base)" % corr_factor)
        else :
            corr_factor = 1.0

        self._test_log.append(["angle_test", reported_angle, meas_angle, corr_factor])

        return corr_factor


    def ratioSpinCalibration( self ) :
        print("How many times would you like the robot to spin in each direction? [-]spins")
        spins = 0

        while (spins == 0) :
            spins_txt = raw_input("[3]: ")

            spins_split = spins_txt.split()
            num_args = len(spins_split)

            if (num_args == 0) :
                spins = 3
            elif (num_args == 1) :
                try :
                    spins = int(spins_split[0])
                except :
                    spins = 0

            else:
               spins = 0    
                
            if (spins == 0) :
                print("Invalid input: %s" % spins_txt)

        print("Times to spin around: %d" % spins )

        turn_angle = spins * pi * 2.0

        corr_factor = []

        for direction in range(2) :
            # Distance traveled as seen by the robot
            reported_angle = self.requestMovementAction(0, turn_angle)

            overshoot = math.degrees(reported_angle - turn_angle)

            print("The robot thinks it completed the turn with %.2f degrees of overshoot" % (overshoot))
                
            print("How many degrees did the robot actually overshoot?")
            meas_txt = raw_input("[%.4f] " % overshoot)
            meas_split = meas_txt.split()
            num_args = len(meas_split)

            if (num_args == 0) :
                meas_angle = reported_angle
            elif (num_args == 1) :
                try :
                    meas_angle = float(meas_split[0])

                    meas_angle = math.radians(meas_angle)

                    if (turn_angle >= 0) :
                        meas_angle += turn_angle
                    else :
                        meas_angle -= turn_angle

                except :
                    meas_angle = 0.0
            else :
                meas_angle = 0.0
            
            # Distance traveled as measured by a ruler
            print("The angle needs a correction factor of %.4f applied (wheel base)" % ( abs(reported_angle / meas_angle) ))
            print("Reported: %f Measured: %f Requested: %f" % (reported_angle, meas_angle, turn_angle))

            if (meas_angle != reported_angle) :
                corr_factor.append( abs(reported_angle / meas_angle) )
            else :
                corr_factor.append( 1.0 )

            if (direction == 0) :
                turn_angle *= -1.0

                print("Please reset the robot for the other direction")
                raw_input("[] ")

        corr_total = corr_factor[1] - corr_factor[0]

        corr_total += 1.0

        corr_ratio = corr_total

        #if (corr_factor[1] != 0.0) :
        #    corr_ratio = corr_factor[0] / corr_factor[1]
        #else :
        #    corr_ratio = 0.0 # Error

        corr_ratio = corr_factor[0] / corr_factor[1]

        print("The wheel ratio needs a correction factor of %.4f applied" % corr_ratio)
        self._test_log.append(["ratio_spin_test", corr_factor[0], corr_factor[1], corr_ratio])

        return corr_factor


    def metersToUnits( self, meters, units ) :
        """ Converts meters into several different units.
        """

        if (units == "m") :
            unit_length = meters;
        elif (units == "cm") :
            unit_length = meters * M_TO_CM
        elif (units == "mm") :
            unit_length = meters * M_TO_MM
        elif (units == "in") :
            unit_length = meters * M_TO_IN
        elif (units == "ft") :
            unit_length = meters * M_TO_FT
        elif (units == "yd") :
            unit_length = meters * M_TO_YD
        else :
            unit_length = 0.0

        return unit_length
        

    def unitsToMeters( self, unit_length, units ) :
        """ Converts several different units into meters.
        """

        if (units == "m") :
            meters = unit_length;
        elif (units == "cm") :
            meters = unit_length * CM_TO_M
        elif (units == "mm") :
            meters = unit_length * MM_TO_M
        elif (units == "in") :
            meters = unit_length * IN_TO_M
        elif (units == "ft") :
            meters = unit_length * FT_TO_M
        elif (units == "yd") :
            meters = unit_length * YD_TO_M
        else :
            meters = 0.0

        return meters

    
    def requestMovementAction(self, distance, angle) :
        """ Uses ROS action services from simple_base_movement distance_controller to execute a movement
        """
        self._is_running = True

        retval = 0.0

        movement_step = MoveDistanceGoal(distance, angle)

        # Wait just a little longer for the action lib server, this has helped 
        # with problems on first goal locking or getting falsely finished.
        rospy.sleep(0.1)
           
        self._is_running = True
       
        rospy.logdebug("Sending Goal:\tDistance %f, Angle %f (Degrees %f)" %
                        (movement_step.distance,
                         movement_step.angle,
                         math.degrees(movement_step.angle)))

        # Send the goal pose to the MoveBaseAction server
        self._action_client.send_goal(movement_step)
       
        # Allow 1 minute to get there
        finished_within_time = self._action_client.wait_for_result(rospy.Duration(60))
       
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self._action_client.cancel_goal()
# no result if no finish             rospy.loginfo("Timed out achieving goal:\tDistance %f, Angle %f (Degrees %f)" %
# no result if no finish                             (result.final_distance,
# no result if no finish                             result.final_angle,
# no result if no finish                             math.degrees(result.final_angle)))
            rospy.loginfo("Timed out achieving goal.")
        else:
            # We made it!
            # state = self._action_client.get_state()
            retval = 0
            result = self._action_client.get_result()
            rospy.logdebug("Reached Goal:\tDistance %f, Angle %f (Degrees %f)" % 
                            (result.final_distance,
                             result.final_angle,
                             math.degrees(result.final_angle)))

            if (movement_step.distance != 0):
                retval = result.final_distance
                dist_error = 100.0 * (abs(movement_step.distance - 
                                            result.final_distance) /
                                      abs(movement_step.distance))
            else:
                dist_error = 0.0

            if (movement_step.angle != 0):
                retval = result.final_angle
                angle_error = 100.0 * (abs(movement_step.angle - 
                                            result.final_angle) / 
                                       abs(movement_step.angle))
            else:
                angle_error = 0

            rospy.logdebug("Goal Errors: \tDistance %f%% Angle %f%%\n" % 
                            (dist_error, angle_error))

        self._is_running = False
        return retval


    def promptDistance(self, default_dist, default_unit):
        length = 0.0
        
        while (length == 0.0):
            command_text = raw_input("[%.2f %s]: " % ( default_dist, default_unit ))
            
            cmd_split = command_text.split()
            num_args = len(cmd_split)

            if (num_args == 0) :
                unit_length = default_dist
                cmd_units   = default_unit
            elif (num_args == 1) :
                try :
                    unit_length  = float(cmd_split[0])
                except :
                    unit_length = 0.0

                cmd_units   = "m"

            elif (num_args == 2) :
                try :
                    unit_length  = float(cmd_split[0])
                except :
                    unit_length = 0.0
                
                cmd_units   = cmd_split[1]

            length = self.unitsToMeters(unit_length, cmd_units)

            if (length == 0.0) :
                print("Invalid input: %s" % command_text)

        return length, cmd_units, unit_length


    def printLogs(self) :
        num_logs = len(self._test_log)

        print("Tests Run: %d" % (num_logs - 1))

        if (num_logs > 1) :
            print("\n%s, %s, %s, %s" % ( self._test_log[0][0], self._test_log[0][1], \
                        self._test_log[0][2], self._test_log[0][3]))
        
            for i in range(1, num_logs) :
                print("%s, %.2f, %.2f, %.4f" % ( self._test_log[i][0], self._test_log[i][1], \
                            self._test_log[i][2], self._test_log[i][3]))
    

    def shutdown(self) :
        if self._is_running:
# SJL        rospy.loginfo("diff_drive_calibrator shutdown()...")
# SJL                self._action_client.cancel_goal()
                self._is_running = False;

        #  Using time instead of rospy in case the ROS server dies
        time.sleep(1) 


if __name__ == '__main__':
# SJL    try:
        CalibrateDiffDrive('diff_drive_calibrated')
# SJL    except rospy.ROSInterruptException:
# SJL       pass

