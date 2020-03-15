import rospy
from jetto_base.constants import Topics
from jetto_base.motors.motors import MotorDriver
from jetto_base.msg import RawMotor


class RawMotorSubscriber():

    def __init__(self, node_name, topic=Topics.Motor.value):
        self._node_name = node_name
        self._topic = topic
        self._motor_driver = MotorDriver()

    def run(self):
        rospy.init_node(self._node_name, anonymous=True)
        rospy.Subscriber(self._topic, RawMotor, callback=self.msg_callback, queue_size=1)

        try:
            rospy.spin()
        finally:
            self._motor_driver.stop()

    def msg_callback(self, msg):
        info = "Moving robot L: {lvalue:+.2f} R: {rvalue:+.2f}".format(
            lvalue=msg.lvalue, rvalue=msg.rvalue)
        rospy.loginfo(info)
        self._motor_driver.write(msg.lvalue, msg.rvalue)
