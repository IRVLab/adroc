#!/usr/bin/python
import rospy
import math

from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse

from auv_aoc.msg import DiverRelativePosition
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from openpose_ros_msgs.msg import BodypartDetection, PersonDetection


# Dynamic reconfigure stuff.
from dynamic_reconfigure.server import Server
from auv_aoc.cfg import DRP_ParamsConfig


class DRP_Processor:
    def __init__(self):
        rospy.init_node('drp_node', anonymous=True)

        # Topic variables
        self.base_image_topic = rospy.get_param('base_image_topic', '/loco_cams/left/image_raw')
        self.bbox_topic = rospy.get_param('bbox_topic','/darknet_ros/bounding_boxes')
        self.pose_topic = rospy.get_param('pose_topic','/detected_poses_keypoints')
        self.drp_topic = rospy.get_param('drp_topic','drp/drp_target')

        # Option variables
        self.visualize = rospy.get_param('vizualize', default=False)


        if self.visualize:
            self.base_image_sub = rospy.Subscriber(self.base_image_topic, Image, self.base_image_cb)
            
            self.drp_image_topic = rospy.get_param('drp_image_topic', 'drp/drp_image')
            self.drp_image_pub = rospy.Publisher(self.drp_image_topic, Image)

            self.bridge_object = CvBridge()
            self.cv_image = None


        ## Actual node stuff starting to happen here.
        image_msg = rospy.wait_for_message(self.base_image_topic, Image)
        self.image_w = image_msg.width
        self.image_h = image_msg.height
        rospy.loginfo('Aquired base image dimmensions')

        # This subscriber provides us with the bounding boxes that we'll use to compute our relative position target.
        self.bbox_sub = rospy.Subscriber(self.bbox_topic, BoundingBoxes, self.bbox_msg_cb, queue_size=5)
        self.bbox_observation = [0,0,0,0]
        self.bbox_conf = 0.0
        self.bbox_ts = 0

        # This subscriber provides us with the pose information that we'll use to compute our relative position target.
        self.pose_sub = rospy.Subscriber(self.pose_topic, PersonDetection, self.pose_msg_cb, queue_size=5)
        self.rs_observation = [0,0]
        self.rs_conf = 1.0
        self.rs_ts = 0
        self.ls_observation = [0,0]
        self.ls_conf = 0.0
        self.ls_ts = 0

        # This publisher is how we send out the calculated diver-relative-position
        self.drp_pub = rospy.Publisher(self.drp_topic, DiverRelativePosition, queue_size=5)

        # These services allow other nodes to turn DRP on and off, which keeps it from publishing DRP targets when we're doing other stuff.
        self.drp_active = True # This boolean turns DRP processing on and off.
        rospy.Service('drp/start', Trigger, self.start_service_handler)
        rospy.Service('drp/stop', Trigger, self.stop_service_handler)

        #Dynamic reconfigure server
        srv = Server(DRP_ParamsConfig, self.cfg_callback)

        # DRP Configuration variables
        #TODO add dynamic_reconfigure parameter setting for this
        self.observation_timeout = 1
        self.bbox_target_ratio = 0.6
        self.shoulder_target_ratio= 0.6
        

    '''
        ROS handlers for messages and services
    '''
    #Receieves a bbox message and stores it in the DRP_Processor object.
    def bbox_msg_cb(self, msg):
        boxes = list()
        max_conf = -1
        max_idx = None

        for idx, b in enumerate(msg.bounding_boxes):
            boxes.append(b)
            if b.probability > max_conf:
                max_conf = b.probability
                max_idx = idx

        if not max_idx is None:
            selbox = boxes[max_idx]
            self.bbox_observation = [selbox.xmin, selbox.ymin, selbox.xmax, selbox.ymax]
            self.bbox_conf = max_conf
            self.bbox_ts = rospy.Time.now().to_sec()

    #Receieves a Pose message and stores it in the DRP_Processor object.
    def pose_msg_cb(self, msg):
        self.rs_observation = [msg.right_shoulder.x, msg.right_shoulder.y]
        self.rs_conf = msg.right_shoulder.confidence
        self.rs_ts = rospy.Time.now().to_sec()

        self.ls_observation = [msg.left_shoulder.x, msg.left_shoulder.y]
        self.ls_conf = msg.left_shoulder.confidence
        self.ls_ts = rospy.Time.now().to_sec()

    # Stores the base image in a variable. Shouldn't be called unless the visualize option is set.
    def base_image_cb(self, msg):
        self.cv_image = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding="bgr8")


    #Dynamic reconfigure callback
    def cfg_callback(self, config, level):
        self.observation_timeout = config.obs_timeout
        self.bbox_target_ratio = config.bbox_target_ratio
        self.shoulder_target_ratio = config.shoulder_target_ratio
        return config

    def start_service_handler(self, request):
        self.drp_active = True
        t = TriggerResponse()
        t.success=True
        t.message="DRP started"
        return t

    def stop_service_handler(self, request):
        self.drp_active = False
        t = TriggerResponse()
        t.success=True
        t.message="DRP stopped"
        return t

    '''
        DRP processing functions, which produce a single DRP (center point and pseudo distance) based on bbox and pose detections of a human target
    '''

    # If there is a recent enough Pose to work off of, return true, otherwise false.
    def pose_valid(self, now):
        #TODO could add confidence filtering here?
        #rospy.loginfo('now:%d  rs:%d   diff:%d  timeout:%d', now, self.rs_ts, (now-self.rs_ts), self.observation_timeout)
        #rospy.loginfo('now:%d  ls%d    diff%d   timeout:%d', now, self.ls_ts, (now-self.ls_ts), self.observation_timeout)
        time_term= ((now - self.rs_ts) < self.observation_timeout) and ((now - self.ls_ts) < self.observation_timeout) # we need to check that both of the shoulders are recent enough, since we might get detections with only one or the other.


        zero_r_term = not (self.rs_observation[0] == 0 and self.rs_observation[1] ==0)
        zero_l_term = not (self.ls_observation[0] == 0 and self.ls_observation[1] ==0)

        return time_term and zero_r_term and zero_l_term

    # If there is a recent enough BBox to work off of, return true, otherwise false.
    def bbox_valid(self, now):
        #rospy.loginfo('now: %d   bbox:%d    diff:%d   timeout:%d', now, self.bbox_ts, (now-self.bbox_ts), self.observation_timeout)
        time_term= ((now- self.bbox_ts) < self.observation_timeout)
        coord_term= (self.bbox_observation[0] >=0 and self.bbox_observation[1] >=0 and self.bbox_observation[2] >=0 and self.bbox_observation[3])
        return time_term and coord_term
        

    # Make a DRP (centerpoint and pseudo-distance) out of a bbox observation.
    def bbox_to_drp(self):
        cp_x, cp_y, pd = None, None, None

        xmin, ymin, xmax, ymax = self.bbox_observation
        bbox_w = xmax-xmin
        bbox_h = ymax-ymin

        cp_x = int(xmin+bbox_w/2.0)
        cp_y = int(ymin+bbox_h/2.0)

        bbox_area = bbox_w * bbox_h
        image_area = self.image_w * self.image_h

        # This is calculation from target_following, gotta make sure it works for us.
        pd = self.bbox_target_ratio * (1.0-bbox_area/float(image_area))

        return (cp_x, cp_y), pd

    # Make a DRP (centerpoint and pseudo-distance) out of a pose observation.
    def pose_to_drp(self):
        cp_x, cp_y, pd = None, None, None

        rx, ry = self.rs_observation
        lx, ly = self.ls_observation

        cp_x = int((lx+rx)/2)
        cp_y = int((ly+ry)/2)

        dist = math.sqrt( (lx-rx)**2 + (ly-ry)**2 )
        pd = dist/self.shoulder_target_ratio #Ratio betwen target shoulder pixel distance and actual pixel distance.

        return (cp_x, cp_y), pd

    def draw_drp_image(self, centerpoint, pseudo_distance, draw_pose, draw_bbox):

        # Image to draw on is available at self.cv_image.
        # Right shoulder x,y list is self.rs_observation
        # Left shoulder x,y list is self.ls_observation
        # Bounding box [xmin, ymin, xmax, ymax] is self.bbox_observation
        # DRP data is passed in as centerpoint (x,y), and PD

        # draw_pose and draw_bbox will be set true or false based on whether or not you should draw them
        # Either convert the image to a sensor_msgs/Image here, or after return. Line 248-249 is when you need to publish
        return

    def process(self):
        if self.drp_active:
            now = rospy.Time.now().to_sec() #Get current ros time.
            cp_pose, cp_bbox, pdist_pose, pdist_bbox = None, None, None, None
            centerpoint = []
            pseudo_distance = 0

            pose_valid = self.pose_valid(now)
            bbox_valid = self.bbox_valid(now)
            if pose_valid:
                cp_pose, pdist_pose = self.pose_to_drp()

            if bbox_valid:
                cp_bbox, pdist_bbox = self.bbox_to_drp()
            
            # Set up DRP message.
            msg = DiverRelativePosition()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()


            if (not cp_pose is None) and (not cp_bbox is None): #Both are available.
                rospy.loginfo('Estimating DRP based on bbox and pose')
                rospy.loginfo('CP_POSE:(%d, %d), CP_BBOX:(%d,%d), PD_POSE:%f, PD_BBOX:%f', cp_pose[0], cp_pose[1], cp_bbox[0], cp_bbox[1], pdist_pose, pdist_bbox)
                #Average of center point from bounding box and pose
                centerpoint[0] = int(cp_pose[0] + cp_bbox[0])/2
                centerpoint[1] = int(cp_pose[1] + cp_bbox[1])/2

                pseudo_distance = pdist_pose #We always used pose pseudo-distance when it's available, because it's more accurate.

            elif (not cp_pose is None): # Pose center point is available.
                rospy.loginfo('Estimating DRP based on pose only')
                rospy.loginfo('CP_POSE:(%d,%d), PD_POSE:%f', cp_pose[0], cp_pose[1], pdist_pose)
                centerpoint[0] = cp_pose[0]
                centerpoint[1] = cp_pose[1]

                pseudo_distance = pdist_pose

            elif (not cp_bbox is None): # BBox center point is avalable.
                rospy.loginfo('Estimating DRP based on bbox only.')
                rospy.loginfo('CP_BBOX:(%d, %d), PD_BBOX:%f', cp_bbox[0], cp_bbox[1], pdist_bbox)
                centerpoint[0] = cp_bbox[0]
                centerpoint[1] = cp_bbox[1]

                pseudo_distance = pdist_bbox

            else: #Nothing available, so we're going to give up
                rospy.loginfo('No messages recent enough, so no DRP estimate')
                return
            
            rospy.loginfo('DRP: X=%d,Y=%d, PD=%d', msg.target_x, msg.target_y, msg.pseudo_distance)
            #Assuming we're here, we should have a filled DRP message, so we just need to publish.
            msg.target_x = centerpoint[0]
            msg.target_y = centerpoint[1]
            msg.pseudo_distance = pseudo_distance

            self.drp_pub.publish(msg)

            # TODO update this to whatever you need to.
            if self.visualize:
                image = self.draw_drp_image(centerpoint, pseudo_distance, pose_valid, bbox_valid)
                self.drp_image_pub.publish(image)

        else:
            return

if __name__ == '__main__':
    drp = DRP_Processor()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        drp.process()
        r.sleep()

else:
    pass
