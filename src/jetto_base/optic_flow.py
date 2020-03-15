import rospy
from jetto_base.constants import Topics
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class OpticFlowPubSub():

    def __init__(self, node_name, sub_topic=Topics.Motor.value, pub_topic=Topics.):
        self._node_name = node_name
        self._topic = topic
        self._motor_driver = MotorDriver()

    def run(self):
        rospy.init_node(self._node_name, anonymous=True)
        rospy.Subscriber(self._topic, Image, callback=self.msg_callback, queue_size=1)

        try:
            rospy.spin()
        finally:
            self._motor_driver.stop()

    def msg_callback(self, msg):
        info = "Moving robot L: {lvalue:+.2f} R: {rvalue:+.2f}".format(
            lvalue=msg.lvalue, rvalue=msg.rvalue)
        rospy.loginfo(info)
        self._motor_driver.write(msg.lvalue, msg.rvalue)
