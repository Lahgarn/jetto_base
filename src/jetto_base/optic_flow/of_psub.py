import cv2
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError
from jetto_base.constants import Topics
from sensor_msgs.msg import Image


bridge = CvBridge()


class OpticFlowPubSub():

    def __init__(self, sub_topic=Topics.CameraRaw.value, pub_topic=Topics.OpticFlow.value):
        self._sub = rospy.Subscriber(sub_topic, Image, callback=self.msg_callback, queue_size=10)
        # self._pub = rospy.Publisher(pub_topic, String, queue_size=10)

        self._prev_img = None
        self._curr_img = None
        self._t0 = time.time()

    def run(self):
        rospy.spin()

    def msg_callback(self, data):
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.logerror(e)
            return

        self._prev_img = self._curr_img
        self._curr_img = cv_image

        # print("CALLBACK: ", time.time() - self._t0)
        self._t0 = time.time()

        if self._prev_img is not None:
            t0 = time.time()
            flow = cv2.calcOpticalFlowFarneback(prev=self._prev_img, next=self._curr_img, flow=None, pyr_scale=0.5,
                                                levels=3, winsize=15, iterations=3, poly_n=5, poly_sigma=1.2, flags=0)
            print("FLOW: ", time.time() - t0)
