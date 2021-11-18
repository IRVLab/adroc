#!/usr/bin/python3

# This code is a part of the LoCO AUV project.
# Copyright (C) The Regents of the University of Minnesota

# Maintainer: Junaed Sattar <junaed@umn.edu> and the Interactive Robotics and Vision Laboratory

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
import actionlib
import roslaunch

import subprocess, signal, os

import adroc.msg
from loco_pilot.srv import Yaw, YawRequest, YawResponse
from loco_pilot.msg import Command
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class ADROCState:
    INIT = 0
    SEARCH = 1
    APPROACH = 2
    GREET = 3
    WAIT_FOR_INPUT = 4
    CONCLUDE = 5
    SHUTDOWN = 6

    def id_to_string(id):
        if id == 0:
            return "INIT"
        elif id == 1:
            return "SEARCH"
        elif id == 2:
            return "APPROACH"
        elif id == 3:
            return "GREET"
        elif id == 4:
            return "WAIT_FOR_INPUT"
        elif id == 5:
            return "CONCLUDE"
        elif id == 6:
            return "SHUTDOWN"

class ADROC_Action:
    _feedback = adroc.msg.ApproachDiverFeedback()
    _result = adroc.msg.ApproachDiverResult()

    def __init__(self, name):
        self.rate = 10
        self.state = ADROCState.INIT

        self.x_error_tolerance = rospy.get_param('adroc/x_error_tolerance', 0.01)
        self.y_error_tolerance = rospy.get_param('adroc/y_error_tolerance', 0.01)
        self.pd_error_tolerance = rospy.get_param('adroc/pd_error_tolerance', 0.1)
        self.drp_active_timeout = rospy.get_param('adroc/drp_active_timeout', 1)

        rospy.Subscriber('/drp/drp_target', adroc.msg.DiverRelativePosition, self.drp_cb)
        self.drp_msgs = list()
        self.last_drp_msg = 0

        self.activate_drp_controller = rospy.ServiceProxy('drp_reactive_controller/start', Trigger)
        self.deactivate_drp_controller = rospy.ServiceProxy('drp_reactive_controller/stop', Trigger)
        
        self.cmd_pub = rospy.Publisher('/loco/command', Command, queue_size=10)
        self.search_yaw_speed = 0.2
        self.search_it_count = 0

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, adroc.msg.ApproachDiverAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.bag_launch = None
        self.lf_cmd = "roslaunch /home/irvlab/catkin_ws/src/adroc/launch/data_record.launch"
        self.dg_name = "/data/adroc_digest.txt"
        self.initated_time = None

    def begin_data_recoding(self):
        pass

    def end_data_recording(self):
        # Write to digest file.
        current_time = rospy.get_time()
        duration = current_time - self.initated_time
        final_drp = self.drp_msgs[-1]

        # Write trial ID, DRP (x,y,pd), duration, and final state.
        with open(self.dg_name, 'a+') as f:
            f.write("Trial at %f,%d,%d,%f,%f,%s\n"%(self.initated_time, final_drp.target_x, final_drp.target_y, final_drp.pseudo_distance, duration, ADROCState.id_to_string(self.state)))

    def drp_cb(self, msg):
        if len(self.drp_msgs) == 5:
            self.drp_msgs.pop(0)
        
        self.drp_msgs.append(msg)
        self.last_drp_msg = rospy.Time.now().to_sec()

        return

    # Return true if there's an active DRP value.
    def drp_active(self):
        return ((rospy.Time.now().to_sec() - self.last_drp_msg) < self.drp_active_timeout)
    
    # Return true if there's a stable DRP estimation (we're in the appropriate relative postion)
    def drp_stable(self):
        if len(self.drp_msgs) >0:
            x_errs = list()
            y_errs = list()
            pd_errs = list()

            image_setpoint_x = self.drp_msgs[0].image_w/2.0
            image_setpoint_y = self.drp_msgs[0].image_h/2.0 #########
        
            for drp_msg in self.drp_msgs:
                x_errs.append((drp_msg.target_x - image_setpoint_x)/ float(drp_msg.image_w)) #Pixel difference between target point and DRP point, normalized by image size.
                y_errs.append((drp_msg.target_y - image_setpoint_y)/ float(drp_msg.image_h))
                pd_errs.append(1.0 - drp_msg.pseudo_distance)

        
            x_err_mean = abs(sum(x_errs)/len(x_errs))
            y_err_mean = abs(sum(y_errs)/len(y_errs))
            pd_err_mean = abs(sum(pd_errs)/len(pd_errs))

            return (x_err_mean < self.x_error_tolerance) and (y_err_mean < self.y_error_tolerance) and (pd_err_mean < self.pd_error_tolerance)
        else:
            return False

    # Search for a diver
    def perform_search(self):
        msg = Command() 
        msg.roll = 0
        msg.pitch = 0
        msg.yaw = self.search_yaw_speed
        msg.throttle = 0 
        msg.heave = 0

        if(self.search_it_count<10):
            self.cmd_pub.publish(msg)
            self.search_it_count+=1
            rospy.loginfo("ADROC Searching...yawing at %f", msg.yaw)
        elif(self.search_it_count<20):
            self.search_it_count+=1
            rospy.loginfo("ADROC Searching...waiting")
        else:
            self.search_it_count = 0

        return

    # State change function
    def change_state(self, state):
        #TODO check for illegal state transitions.
        self.state = state 

    # Based on current state, check for state change. Change state if required, process state if not.
    def run(self):
        if self.state == ADROCState.INIT:
            if not self.drp_active():
                rospy.loginfo("ADROC State -> SEARCH")
                self.change_state(ADROCState.SEARCH) # If we don't see anyone, go to search
            else:
                rospy.loginfo("ADROC State -> APPROACH")
                self.change_state(ADROCState.APPROACH) #If we see someone, go to them.
                req = TriggerRequest()
                self.activate_drp_controller(req)


        elif self.state == ADROCState.SEARCH:
            if not self.drp_active():
                rospy.loginfo("ADROC searching...")
                self.perform_search() #If we're still searching and haven't found yet, continue search operations.
            else:
                self.activate_drp_controller()
                rospy.loginfo("ADROC State -> APPROACH")
                req = TriggerRequest()
                self.activate_drp_controller(req)
                self.change_state(ADROCState.APPROACH) # If we find someone, switch to approach.

        elif self.state == ADROCState.APPROACH:
            if not self.drp_stable():
                rospy.loginfo("ADROC waiting for stable DRP...")
                if not self.drp_active():
                    rospy.loginfo("ADROC approach failed, returning to search")
                    req = TriggerRequest()
                    self.deactivate_drp_controller(req)
                    rospy.loginfo("ADROC State -> SEARCH")
                    self.change_state(ADROCState.SEARCH) # If we don't see anyone, go to search
                else:
                    return #If DRP isn't stable yet, keep waiting for DRP to handle it. We don't need to do anything extra.
            else:
                req = TriggerRequest()
                self.deactivate_drp_controller(req)
                rospy.loginfo("ADROC State -> CONCLUDE")
                self.change_state(ADROCState.CONCLUDE) # TODO instead of going to ADROCState.CONCLUDE, go to GREET.

        elif self.state == ADROCState.GREET:
            rospy.logerr("ADROC state not implemented.")
            return

        elif self.state == ADROCState.WAIT_FOR_INPUT:
            rospy.logerr("ADROC state not implemented.")
            return

        elif self.state == ADROCState.CONCLUDE:
            # Print out last stuff or whatever, then switch to shutdown.
            rospy.loginfo("ADROC State -> SHUTDOWN")
            self.change_state(ADROCState.SHUTDOWN)

        else:
            rospy.logerr("ADROC state not recognized.")
            return

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(self.rate)
        success = True
        
        self.initated_time = rospy.get_time()
        self.begin_data_recoding()

        # start executing the action
        while not rospy.is_shutdown() and self.state != ADROCState.SHUTDOWN:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            # Actually do the processing of ADROC states and such
            adroc.run()

            # publish the feedback
            self._feedback.adroc_state_id = self.state
            self._feedback.adroc_state_name = ADROCState.id_to_string(self.state)
            self._as.publish_feedback(self._feedback)

            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)  
            self._result.success = success
            self._result.final_relative_position = self.drp_msgs[-1]
            self._as.set_succeeded(self._result)

        self.end_data_recording()

        #Reset for new ADROC, regardless of what the previous ADROC finished as.
        self.state = ADROCState.INIT
        req = TriggerRequest()
        self.deactivate_drp_controller(req)
    

if __name__ == '__main__':
    rospy.init_node('adroc', anonymous=False)
    adroc = ADROC_Action(rospy.get_name())
    rospy.spin()

else:
    pass
