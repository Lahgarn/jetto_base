#!/usr/bin/env python
import rospy
from jetto_base.optic_flow import OpticFlowPubSub


if __name__ == '__main__':
    of_pubsub = OpticFlowPubSub()
    rospy.init_node("optical_flow", anonymous=True)
    of_pubsub.run()
