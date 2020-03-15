#!/usr/bin/env python
import rospy
from inputs import get_gamepad

from jetto_base.constants import Topics
from jetto_base.msg import RawMotor


def dir_to_motor(direction, maxvel=1.0):
    # Convert from (0, 255) to (0, 1)
    dx = float(direction[0]) / 255
    # Convert from (0, 255) to (1, -1)
    dy = float(direction[1]) / 255
    dy = 1 - 2*dy

    # Compute speed and direction
    a = 0.2
    k = 1.0 / (1 + a)
    lval = dy * (1 if dx < -a else k * (1 + dx))
    rval = dy * (1 if dx > a else k * (1 - dx))

    # Renormalize to maxvel
    lval *= maxvel
    rval *= maxvel

    # Truncate to 0.05
    lval = float(int(20 * lval)) / 20
    rval = float(int(20 * rval)) / 20

    return [lval, rval]


def control_pub():
    rospy.init_node('controller', anonymous=True)
    pub = rospy.Publisher(Topics.Motor.value, RawMotor, queue_size=10)
    # r = rospy.Rate(10)

    direction = [128, 128]
    while not rospy.is_shutdown():
        events = get_gamepad()
        if events:
            for event in events[-2:]:
                if event.code == 'ABS_X':
                    direction[0] = event.state
                elif event.code == 'ABS_Y':
                    direction[1] = event.state
            motor_signal = dir_to_motor(direction, 0.5)
            # Build message
            motor_msg = RawMotor()
            motor_msg.lvalue = motor_signal[0]
            motor_msg.rvalue = motor_signal[1]

            # Log info
            info = "Got direction {}".format(motor_signal)
            rospy.loginfo(info)
            pub.publish(motor_msg)
        # r.sleep()


if __name__ == '__main__':
    control_pub()
