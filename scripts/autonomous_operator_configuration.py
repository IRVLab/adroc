#!/usr/bin/python3
import rospy

from adroc.msg import DiverRelativePosition
from loco_pilot.srv import Yaw, YawRequest, YawResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class ADROCState:
    INIT = 0
    SEARCH = 1
    APPROACH = 2
    GREET = 3
    WAIT_FOR_INPUT = 4
    CONCLUDE = 5
    SHUTDOWN = 6

class ADROC_Manager:
    def __init__(self):
        rospy.init_node('adroc_manager', anonymous=False)
        self.rate = 10
        self.state = ADROCState.INIT

        self.x_error_tolerance = rospy.get_param('adroc/x_error_tolerance', 0.01)
        self.y_error_tolerance = rospy.get_param('adroc/y_error_tolerance', 0.01)
        self.pd_error_tolerance = rospy.get_param('adroc/pd_error_tolerance', 0.1)
        self.drp_active_timeout = rospy.get_param('adroc/drp_active_timeout', 1)

        rospy.Subscriber('/drp/drp_target', DiverRelativePosition, self.drp_cb)
        self.drp_msgs = list()
        self.last_drp_msg = 0

        self.activate_drp_controller = rospy.ServiceProxy('drp_reactive_controller/start', Trigger)
        self.deactivate_drp_controller = rospy.ServiceProxy('drp_reactive_controller/stop', Trigger)
        
        self.search_yaw_speed = 0.1

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
        # Using LoCO Pilot motion primatives, ask for small Yaw (for duration appropriate to the rate)
        req = YawRequest()
        req.duration = 1/self.rate
        req.thrust = self.search_yaw_speed

        rospy.loginfo("ADROC Searching...yawing at %f for %f seconds", req.thrust, req.duration)
        # self.yaw_service(req)

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

if __name__ == '__main__':
    adroc = ADROC_Manager()
    r = rospy.Rate(adroc.rate)
    rospy.loginfo("INITIATING ADROC!")

    while not rospy.is_shutdown() and adroc.state != ADROCState.SHUTDOWN:
        adroc.run()
        r.sleep()

else:
    pass
