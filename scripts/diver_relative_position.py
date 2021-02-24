import rospy

from std_srvs.srv import Trigger

from auv_aoc.msg import DiverRelativePosition
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from open_pose_ros_msgs.msg import BodypartDetection, PersonDetection


class DRP_Processor(object):
    def __init__(self):
        rospy.init_node('drp_node', anonymous=True)

        # This subscriber provides us with the bounding boxes that we'll use to compute our relative position target.
        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_msg_cb, queue_size=5)
        self.bbox_observation = [0,0,0,0]
        self.bbox_conf = 0.0
        self.bbox_ts = 0

        # This subscriber provides us with the pose information that we'll use to compute our relative position target.
        self.pose_sub = rospy.Subscriber('/trt_pose/detections', PersonDetection, self.pose_msg_cb, queue_size=5)
        self.rs_observation = [0,0]
        self.rs_conf = 1.0
        self.rs_ts = 0
        self.ls_observation = [0,0]
        self.ls_conf = 0.0
        self.ls_ts = 0

        # This publisher is how we send out the calculated diver-relative-position
        self.drp_pub = rospy.Publisher('drp/drp_target', DiverRelativePosition, queue_size=5)

        # These services allow other nodes to turn DRP on and off, which keeps it from publishing DRP targets when we're doing other stuff.
        self.drp_active = False # This boolean turns DRP processing on and off.
        rospy.Service('drp/start', Trigger, self.start_service_handler)
        rospy.Service('drp/stop', Trigger, self.stop_service_handler)
        

    def bbox_msg_cb(self, msg):
        boxes = list()
        confs = list()

        for b in msg.bounding_boxes:
            boxes.append(b)
            confs.append(b.probability)
        
        selbox = boxes(index(max(confs)))
        self.bbox_observation = [selbox.xmin, selbox.ymin, selbox.xmax, selbox.ymax]
        self.bbox_conf = max(confs)
        self.bbox_ts = msg.header.stamp

    
    def pose_msg_cb(self, msg):
        self.rs_observation = [msg.right_shoulder.x, msg.right_shoulder.y]
        self.rs_conf = msg.right_shoulder.confidence
        self.rs_ts = rospy.Time.now()

        self.ls_observation = [msg.left_shoulder.x, msg.left_shoulder.y]
        self.ls_conf = msg.left_shoulder.confidence
        self.ls_ts = rospy.Time.now()

    def start_service_handler(self, request):
        self.drp_active = True
        return True

    def stop_service_handler(self, request):
        self.drp_active = False
        return True


    def process(self):
        if self.drp_active:


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