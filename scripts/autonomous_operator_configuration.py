import rospy

from auv_aoc.msg import DiverRelativePosition
from loco_pilot.srv import Yaw, YawRequest, YawResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class AOCState:
    INIT = 0
    SEARCH = 1
    APPROACH = 2
    GREET = 3
    WAIT_FOR_INPUT = 4
    CONCLUDE = 5
    SHUTDOWN = 6

class AOC_Manager:
    def __init__(self):
        rospy.init_node('aoc_manager', anonymous=False)
        self.rate = 10
        self.state = AOCState.INIT

        self.x_error_tolerance = rospy.get_param('aoc/x_error_tolerance', 0.01)
        self.y_error_tolerance = rospy.get_param('aoc/y_error_tolerance', 0.01)
        self.pd_error_tolerance = rospy.get_param('aoc/pd_error_tolerance', 0.1)
        self.drp_active_timeout = rospy.get_param('aoc/drp_active_timeout', 1)

        rospy.Subscriber('/drp/drp_target', DiverRelativePosition, self.drp_cb)
        self.drp_msgs = list()
        self.last_drp_msg = None

        self.activate_drp_controller = rospy.ServiceProxy('drp_reactive_controller/start', Trigger)
        self.deactivate_drp_controller = rospy.ServiceProxy('drp_reactive_controller/stop', Trigger)
        self.yaw_service = rospy.ServiceProxy('/loco/controller/yaw', Yaw)

        self.search_yaw_speed = 0.1

    def drp_cb(self, msg):
        if len(drp_msgs) == 5:
            self.drp_msgs.pop(0)
        
        self.drp_msgs.append(msg)
        self.last_drp_msg = rospy.Time.now().to_sec()

        return

    # Return true if there's an active DRP value.
    def drp_active(self):
        return ((rospy.Time.now().to_sec - self.last_drp_msg) < self.drp_active_timeout)
    
    # Return true if there's a stable DRP estimation (we're in the appropriate relative postion)
    def drp_stable(self):
        x_errs = list()
        y_errs = list()
        pd_errs = list()

        image_setpoint_x = self.drp_msgs[0].image_w/2.0
        image_setpoint_y = self.drp_msgs[0].image_h/2.0 #########
        
        for drp_msg in self.drp_msgs:
            x_errs.append(tx - image_setpoint_x)/ float(self.image_w) #Pixel difference between target point and DRP point, normalized by image size.
            y_errs.append(ty - image_setpoint_y)/ float(self.image_h)
            pd_errs.append(1.0 - pd)

        
        x_err_mean = abs(sum(x_errs)/len(x_errs))
        y_err_mean = abs(sum(y_errs)/len(y_errs))
        pd_err_mean = abs(sum(pd_errs)/len(pd_errs))

        return (x_err_mean < self.x_error_tolerance) and (y_err_mean < self.y_error_tolerance) and (pd_err_mean < self.pd_error_tolerance)

    # Search for a diver
    def perform_search(self):
        # Using LoCO Pilot motion primatives, ask for small Yaw (for duration appropriate to the rate)
        req = YawRequest
        req.duration = 1/self.rate
        req.thrust = self.search_yaw_speed

        rospy.loginfo("AOC Searching...yawing at %f for %f seconds", req.thrust, req.duration)
        self.yaw_service(req)

        return

    # State change function
    def change_state(self, state):
        #TODO check for illegal state transitions.
        self.state = state 

    # Based on current state, check for state change. Change state if required, process state if not.
    def run_aoc(self):
        if self.state == AOCState.INIT:
            if not self.drp_active():
                rospy.loginfo("AOC State -> SEARCH")
                self.change_state(AOCState.SEARCH) # If we don't see anyone, go to search
            else:
                rospy.loginfo("AOC State -> APPROACH")
                self.change_state(AOCState.APPROACH) #If we see someone, go to them.


        elif self.state == AOCState.SEARCH:
            if not self.drp_active():
                rospy.loginfo("AOC searching...")
                self.perform_search() #If we're still searching and haven't found yet, continue search operations.
            else:
                self.activate_drp_controller()
                rospy.loginfo("AOC State -> APPROACH")
                self.change_state(AOCState.APPROACH) # If we find someone, switch to approach.

        elif self.state == AOCState.APPROACH:
            if not self.drp_stable():
                rospy.loginfo("AOC waiting for stable DRP...")
                return #If DRP isn't stable yet, keep waiting for DRP to handle it. We don't need to do anything extra.
            else:
                self.deactivate_drp_controller()
                rospy.loginfo("AOC State -> CONCLUDE")
                self.change_state(AOCState.CONCLUDE) # TODO instead of going to AOCState.CONCLUDE, go to GREET.

        elif self.state == AOCState.GREET:
            rospy.logerr("AOC state not implemented.")
            return

        elif self.state == AOCState.WAIT_FOR_INPUT:
            rospy.logerr("AOC state not implemented.")
            return

        elif self.state == AOCState.CONCLUDE:
            # Print out last stuff or whatever, then switch to shutdown.
            rospy.loginfo("AOC State -> SHUTDOWN")
            self.change_state(AOCState.SHUTDOWN)

        else:
            rospy.logerr("AOC state not recognized.")
            return

if __name__ == '__main__':
    aoc = AOC_Manager()
    r = rospy.Rate(aoc.rate)

    rospy.wait_for_message('drp/drp_target', DiverRelativePosition)
    rospy.loginfo("DRP messages seen...")
    rospy.wait_for_service('loco/controller/yaw', Yaw)
    rospy.loginfo("Yaw service available...")
    rospy.loginfo("INITIATING AOC!")

    while not rospy.is_shutdown() and aoc.state != AOCState.SHUTDOWN:
        aoc.run()
        r.sleep()

else:
    pass